"""Bumi payload_mass card: static sampling and empirical calibration only.

No imports of SDK or ROS at module load. No control backend is implemented.
The MCP response publication gate prevents completion racing its action_id.
"""
from __future__ import annotations

import copy
import json
import os
from pathlib import Path
import tempfile
import threading
import time
from uuid import uuid4

from payload_mass_sampling import SharedSampler, MassError, finite
from payload_mass_estimator import (VERSION, digest, validate_profile, summarize,
                                    check_baseline, fit_model, estimate)

ACTIONS = ("start", "info", "tare", "calibrate", "fit", "estimate", "stop", "config")
TASKS = ("tare", "calibrate", "fit", "estimate")
DEFAULT_ROOT = "/opt/phanthy-motus/data/bumi/payload_mass"


def notify_completion(action_id, state, result):
    import ssl
    import urllib.request
    url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
    data = json.dumps({"action_id":action_id,"status":state,"result":result,
                       "tool":"payload_mass","ts":time.time()},allow_nan=False).encode()
    context = ssl._create_unverified_context() if url == "https://localhost:15678" else None
    req = urllib.request.Request(url+"/api/acp/complete",data=data,
                                 headers={"Content-Type":"application/json"},method="POST")
    try:
        with urllib.request.urlopen(req,timeout=5,context=context) as response:
            response.read()
    except Exception as exc:
        print(f"[payload_mass] ACP completion delivery failed: {exc}",flush=True)


class Store:
    """Checksummed immutable trial/model documents and atomic profile pointers."""
    def __init__(self, root):
        self.root=Path(root)

    def _path(self, kind, name):
        if kind not in ("profiles","trials","calibrations") or not name or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in name):
            raise MassError("STORAGE", "invalid document name")
        return self.root/kind/(name+".json")

    def read(self, kind, name, *, optional=False):
        path=self._path(kind,name)
        try:
            record=json.loads(path.read_text())
            data=record["data"]
            if record["sha256"]!=digest(data):
                raise ValueError("checksum mismatch")
            return data
        except FileNotFoundError:
            if optional:
                return None
            raise MassError("NOT_CALIBRATED", f"missing {kind}/{name}")
        except (ValueError,KeyError,TypeError) as exc:
            raise MassError("STORAGE_CORRUPT",f"{path}: {exc}") from exc

    def write(self, kind, name, data, *, immutable=False):
        path=self._path(kind,name)
        path.parent.mkdir(parents=True,exist_ok=True)
        encoded=json.dumps({"sha256":digest(data),"data":data},allow_nan=False,sort_keys=True).encode()
        if immutable:
            # Unique UUID name + O_EXCL: an existing raw trial is never rewritten.
            with path.open("xb") as stream:
                stream.write(encoded)
                stream.flush()
                os.fsync(stream.fileno())
            parent_fd=os.open(path.parent,os.O_RDONLY)
            try:
                os.fsync(parent_fd)
            finally:
                os.close(parent_fd)
            return
        fd,tmp=tempfile.mkstemp(prefix=".payload-",dir=path.parent)
        try:
            with os.fdopen(fd,"wb") as stream:
                stream.write(encoded)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(tmp,path)
            parent_fd=os.open(path.parent,os.O_RDONLY)
            try:
                os.fsync(parent_fd)
            finally:
                os.close(parent_fd)
        finally:
            if os.path.exists(tmp):
                os.unlink(tmp)

    def trials(self, profile_id):
        paths=sorted((self.root/"trials").glob(profile_id+"_*.json"))
        if len(paths)>1000:
            raise MassError("STORAGE", "trial count exceeds 1000; archive this profile offline")
        return [self.read("trials",p.stem) for p in paths]


class PayloadMassPlugin:
    PREFIX="payload_mass"

    def __init__(self, plugin_config, namespace, executor=None, sampler=None,
                 *, publish=None, notify=None):
        self._cfg=dict(plugin_config)
        self._sampler=sampler if sampler is not None else SharedSampler()
        self._store=Store(self._cfg.get("data_dir",DEFAULT_ROOT))
        self._file=self._cfg.get("profiles_file",str(self._store.root/"engineering_profiles.json"))
        self._hardware_id=self._cfg.get("hardware_id") or os.environ.get("BUMI_PAYLOAD_ROBOT_ID","")
        self._sdk_version=self._cfg.get("sdk_version") or os.environ.get("BUMI_PAYLOAD_SDK_VERSION","")
        self._topic=f"/{namespace}/payload_mass/result"
        self._executor=executor
        self._node=None
        self._publish=publish or (lambda result: None)
        self._notify=notify or notify_completion
        self._lock=threading.RLock()
        self._job=None
        self._closed=False
        self._last={"state":"idle","valid":False,"mass_kg":None,"unit":"kg"}
        self._last_profile=None
        self._last_summary=None
        if executor is not None:
            # Delay ROS imports so estimator, replay and tests run on x86 hosts.
            from rclpy.node import Node
            from std_msgs.msg import String
            self._node=Node("bumi_payload_mass")
            publisher=self._node.create_publisher(String,self._topic,10)
            def publish_result(result):
                msg=String()
                msg.data=json.dumps(result,allow_nan=False,ensure_ascii=False)
                publisher.publish(msg)
            self._publish=publish_result
            self._node.create_timer(0.5,self._expire)
            executor.add_node(self._node)

    def get_tool(self):
        common=["arm","mode","profile_id","sample_duration_s"]
        properties={"action":{"type":"string","enum":list(ACTIONS)},
                    "arm":{"type":"string","enum":["left","right"],"default":"left"},
                    "mode":{"type":"string","enum":["static","slow_probe"],"default":"static"},
                    "profile_id":{"type":"string","description":"已确认的固定挂具/姿态工程配置 ID"},
                    "sample_duration_s":{"type":"number","minimum":1,"maximum":30,"default":5},
                    "confirm_empty":{"type":"boolean","default":False,"description":"已卸下物体，保留同一挂具；每次独立空载试次确认"},
                    "known_mass_kg":{"type":"number","minimum":0,"description":"仅标定：独立电子秤真值，不能用于 estimate"},
                    "role":{"type":"string","enum":["train","validation"],"default":"train"},
                    "independent_trial_confirmed":{"type":"boolean","default":False,"description":"已独立重新挂载，非同一段数据拆分"},
                    "trial_note":{"type":"string","description":"独立挂载/卸载试次说明与真值来源"}}
        descriptions={"start":"准备只读估重卡，不抬臂、不使能、不切换模式", "info":"能力、原始反馈诊断、标定及最近结果",
                      "tare":"采集同一挂具的独立空载试次，成功后使旧模型失效", "calibrate":"采集独立已知质量试次；不自动替代有效模型",
                      "fit":"按完整训练/验证试次拟合并检验，成功才替换模型", "estimate":"未知质量静态估计；不接受待测真值", "stop":"取消本卡采样，不发送电机或模式指令", "config":"更新工程配置文件路径"}
        params={"start":[],"info":[],"tare":common+["confirm_empty","trial_note"],
                "calibrate":common+["known_mass_kg","role","independent_trial_confirmed","trial_note"],
                "fit":["arm","mode","profile_id"],"estimate":common,"stop":[],"config":[]}
        return {"name":self.PREFIX,"type":"processor","multiInstance":False,
                "description":"手臂载荷估重：只读静态反馈、空载与已知质量经验标定；slow_probe 尚无已验证运动后端。",
                "inputSchema":{"type":"object","properties":properties,"required":["action"],
                               "x-action-params":{a:{"params":params[a],"description":descriptions[a]} for a in ACTIONS},
                               "x-completion":{"actions":list(TASKS),"timeout":100},
                               "x-resource":["arm_l","arm_r"]},
                "configSchema":{"type":"object","properties":{
                    "profiles_file":{"type":"string","scope":"shared","default":self._file,
                                     "description":"现场工程人员核验的配置 JSON 路径；不会提供默认载荷上限"}}},
                "topic_out":[{"topic":self._topic,"format":"data/json"}]}

    def get_tools(self):
        return [self.get_tool()]

    def start(self):
        return True

    def stop(self):
        with self._lock:
            self._closed=True
            job=self._job
            if job:
                job["cancel"].set()
                if not job["publication"].is_set():
                    job["notify"]=False
                    job["publication"].set()
        if job and job["thread"] is not threading.current_thread():
            job["thread"].join(timeout=7)
        with self._lock:
            if job and job["thread"].is_alive():
                raise RuntimeError("payload mass worker did not terminate")
            if self._node is not None:
                self._executor.remove_node(self._node)
                self._node.destroy_node()
                self._node=None

    def _emit(self, result):
        self._last=copy.deepcopy(result)
        try:
            self._publish(copy.deepcopy(result))
        except Exception as exc:
            print(f"[payload_mass] result publication failed: {exc}",flush=True)

    def _error(self, code, message, **extra):
        return {"state":"cancelled" if code=="CANCELLED" else "error","valid":False,
                "mass_kg":None,"unit":"kg","error_code":code,"message":message,**extra}

    def _load_profile(self,args,loading):
        profile_id=args.get("profile_id","")
        try:
            data=json.loads(Path(self._file).read_text())
            raw=data["profiles"][profile_id]
        except (FileNotFoundError,KeyError):
            raise MassError("NOT_CALIBRATED","missing engineering profile; configure hook/pose and verified conditions first")
        except (ValueError,TypeError) as exc:
            raise MassError("CONFIG_CORRUPT",str(exc)) from exc
        try:
            p=validate_profile(profile_id,raw,args.get("arm","left"),loading=loading)
        except (ValueError,TypeError,KeyError) as exc:
            if isinstance(exc,MassError):
                raise
            raise MassError("CONFIG_CORRUPT",str(exc)) from exc
        if not self._hardware_id or not self._sdk_version or p["hardware_id"]!=self._hardware_id or p["sdk_version"]!=self._sdk_version:
            raise MassError("IDENTITY_MISMATCH","robot identity and installed SDK version must match deployment configuration")
        return p

    def _state(self,p):
        return self._store.read("profiles",p["profile_id"],optional=True) or {"baseline":None,"model":None}

    def _expire(self):
        with self._lock:
            if not self._last.get("valid"):
                return
            p=self._last_profile
            diag=self._sampler.diagnostics()
            code=None
            if time.monotonic()-self._last["completed_monotonic_s"] > p["result_max_age_s"]:
                code="RESULT_EXPIRED"
            elif diag["host_age_s"] is None or diag["host_age_s"]>p["max_receive_gap_s"] or diag["read_error_count"] != self._last["read_error_count"]:
                code="STALE_SOURCE"
            else:
                try:
                    from payload_mass_sampling import check_frame
                    check_frame(diag["latest"],p)
                    if self._last_summary and any(abs(diag["latest"]["tau"][j]-self._last_summary["tau"][i])>p["max_torque_span"] for i,j in enumerate(p["motor_ids"])):
                        code="CONDITIONS_CHANGED"
                    current=self._load_profile({"profile_id":p["profile_id"],"arm":p["arm"]},True)
                    if current["fingerprint"]!=p["fingerprint"]:
                        code="CONFIG_CHANGED"
                    state=self._state(p)
                    check_baseline(state["baseline"],summarize([diag["latest"]],p),p,time.time())
                except (MassError,TypeError,KeyError):
                    code="CONDITIONS_CHANGED"
            if code:
                self._emit(self._error(code,"previous measurement is no longer current; collect again"))

    def info(self):
        self._expire()
        with self._lock:
            result=copy.deepcopy(self._last)
            result.update(topic_out=self.get_tool()["topic_out"],
                          modes={"static":"requires verified profile/calibration","slow_probe":"UNSUPPORTED: no verified arm motion backend"},
                          version=VERSION, profiles_file=self._file,
                          deployment_identity={"hardware_id":self._hardware_id,"sdk_version":self._sdk_version},
                          diagnostics=self._sampler.diagnostics(), active=bool(self._job and self._job["thread"].is_alive()))
            try:
                data=json.loads(Path(self._file).read_text())
                result["profiles"]={}
                for key,raw in data["profiles"].items():
                    try:
                        p=self._load_profile({"profile_id":key,"arm":raw.get("arm")},False)
                        state=self._state(p)
                        b=state["baseline"]
                        matching=bool(b and b["profile_fingerprint"]==p["fingerprint"])
                        result["profiles"][key]={"arm":raw.get("arm"),"baseline_id":b["id"] if b else None,
                            "zero_trials":len(b["zero_trial_ids"]) if b else 0,
                            "calibration_id":state["model"] if matching else None,"context_matches":matching}
                    except MassError as exc:
                        result["profiles"][key]={"valid":False,"error_code":exc.code,"message":str(exc)}
            except (OSError,ValueError,KeyError,TypeError) as exc:
                result["configuration_status"]="not_configured: "+str(exc)
            return result

    def dispatch(self,action,args):
        args=dict(args)
        args.pop("_tool_name",None)
        args.pop("instance_id",None)
        gate=args.pop("_await_response_publication",False)
        if action not in ACTIONS:
            return None
        if action=="info":
            return self.info()
        with self._lock:
            if action=="stop":
                if self._job and self._job["thread"].is_alive():
                    self._job["cancel"].set()
                    self._emit(self._error("CANCELLED","cancellation requested",state="stopping"))
                    return copy.deepcopy(self._last)
                self._emit(self._error("CANCELLED","sampling stopped; no motor command sent",state="idle"))
                return copy.deepcopy(self._last)
            if self._closed:
                return self._error("CLOSED","driver plugin has been shut down")
            if self._job and self._job["thread"].is_alive():
                return self._error("BUSY","one payload task already owns this card")
            if action=="start":
                return {"state":"ready","valid":False,"mass_kg":None,"topic_out":self.get_tool()["topic_out"]}
            if action=="config":
                if set(args)!={"profiles_file"} or not isinstance(args["profiles_file"],str) or not Path(args["profiles_file"]).is_absolute():
                    return self._error("BAD_PARAMETER","config requires an absolute profiles_file path")
                self._file=args["profiles_file"]
                self._emit(self._error("CONFIG_CHANGED","engineering config changed; recheck calibration",state="configured"))
                return copy.deepcopy(self._last)
            try:
                self._validate_args(action,args)
                if args.get("mode","static")!="static":
                    raise MassError("UNSUPPORTED","slow_probe has no verified single-arm backend; no movement was sent")
                p=self._load_profile(args,action!="tare")
                state=self._state(p)
                if action!="tare" and not state["baseline"]:
                    raise MassError("NOT_CALIBRATED","collect independent empty baseline first")
                if action=="estimate" and not state["model"]:
                    raise MassError("NOT_CALIBRATED","fit and validate this arm/profile first")
                if action=="calibrate" and args["known_mass_kg"]>p["max_mass_kg"]:
                    raise MassError("OUT_OF_RANGE","calibration mass exceeds engineer-confirmed experiment range")
            except (MassError, OSError) as exc:
                result=self._error(getattr(exc,"code","STORAGE"),str(exc))
                self._emit(result)
                return result
            job={"id":"payload_"+uuid4().hex,"cancel":threading.Event(),"publication":threading.Event(),
                 "notify":True,"gate":gate,"args":args,"profile":p,"action":action}
            if not gate:
                job["publication"].set()
            job["thread"]=threading.Thread(target=self._worker,args=(job,),name="bumi_payload_mass",daemon=True)
            self._job=job
            result={"state":"running","valid":False,"mass_kg":None,"unit":"kg","action_id":job["id"],
                    "arm":p["arm"],"profile_id":p["profile_id"],"mode":"static"}
            self._emit(result)
            try:
                job["thread"].start()
            except RuntimeError as exc:
                self._emit(self._error("THREAD_START",str(exc)))
                return copy.deepcopy(self._last)
            return result

    def _validate_args(self,action,args):
        permitted={"arm","mode","profile_id"}
        if action!="fit":
            permitted.add("sample_duration_s")
        if action=="tare":
            permitted.update(("confirm_empty","trial_note"))
            if args.get("confirm_empty") is not True:
                raise MassError("CONFIRM_EMPTY","explicitly confirm object removed, same fixture retained")
        if action=="calibrate":
            permitted.update(("known_mass_kg","role","independent_trial_confirmed","trial_note"))
            args["known_mass_kg"]=finite(args.get("known_mass_kg"),"known_mass_kg",0)
            if args.get("role","train") not in ("train","validation") or args.get("independent_trial_confirmed") is not True:
                raise MassError("BAD_PARAMETER","calibration requires role and independent re-hanging confirmation")
        # The current generic canvas sends hidden fields and instance_id too.
        # Ignore unrelated declared fields; estimate MUST refuse any truth input.
        declared=set(self.get_tool()["inputSchema"]["properties"])-{"action"}
        if action=="estimate" and "known_mass_kg" in args:
            raise MassError("BAD_PARAMETER","estimate cannot accept known_mass_kg; clear the calibration truth field")
        unknown=set(args)-declared
        if unknown:
            raise MassError("BAD_PARAMETER","unexpected parameters: "+", ".join(sorted(unknown)))
        for key in set(args)-permitted:
            args.pop(key)
        if set(args)-permitted:
            raise MassError("BAD_PARAMETER","unexpected parameters: "+", ".join(sorted(set(args)-permitted)))
        if action in ("tare","calibrate") and (not isinstance(args.get("trial_note"),str) or not args["trial_note"].strip()):
            raise MassError("BAD_PARAMETER","independent trial_note is required")
        if action!="fit":
            args["sample_duration_s"]=finite(args.get("sample_duration_s",5),"sample_duration_s",1,30)

    def response_published(self,action_id,delivered=True):
        with self._lock:
            job=self._job
            if job and job["id"]==action_id and not job["publication"].is_set():
                job["notify"]=bool(delivered)
                if not delivered:
                    job["cancel"].set()
                job["publication"].set()

    def _worker(self,job):
        p,args=job["profile"],job["args"]
        try:
            if not job["publication"].wait(5):
                job["notify"]=False
                raise MassError("RESPONSE_TIMEOUT","MCP response was not delivered")
            if job["cancel"].is_set():
                raise MassError("CANCELLED","task cancelled before sampling")
            state=self._state(p)
            if job["action"]=="fit":
                check_baseline(state["baseline"],state["baseline"]["summary"],p,time.time())
                model=fit_model(self._store.trials(p["profile_id"]),state["baseline"],p)
                model_id=p["profile_id"]+"_"+uuid4().hex
                model.update(id=model_id,created_s=time.time())
                with self._lock:
                    self._guard_commit(job)
                    self._store.write("calibrations",model_id,model,immutable=True)
                    state["model"]=model_id
                    self._store.write("profiles",p["profile_id"],state)
                result={"state":"completed","valid":False,"mass_kg":None,"calibration_id":model_id,
                        "validation":model["validation"],"mass_range_kg":model["mass_range_kg"],"resolution_kg":model["resolution_kg"]}
            else:
                frames=self._sampler.collect(args["sample_duration_s"],p,job["cancel"])
                summary=summarize(frames,p)
                trial_id=p["profile_id"]+"_"+uuid4().hex
                trial={"id":trial_id,"version":VERSION,"profile_fingerprint":p["fingerprint"],"profile":p,
                       "action":job["action"],"created_s":time.time(),"summary":summary,"raw_frames":frames,
                       "trial_note":args.get("trial_note"),"sampling_conditions":{"duration_s":args["sample_duration_s"]}}
                result={"state":"completed","valid":False,"mass_kg":None,"trial_id":trial_id,
                        "sample_count":summary["sample_count"],"effective_sample_count":None,
                        "pose_match":True,"freshness_status":summary["freshness_status"],
                        "dispersion":summary["noise"],"dispersion_definition":summary["dispersion_definition"]}
                if job["action"]=="tare":
                    old=state["baseline"]
                    same=bool(old and old["profile_fingerprint"]==p["fingerprint"] and 0<=time.time()-old["created_s"]<=p["baseline_max_age_s"])
                    zeros=(old["zero_trial_ids"] if same else [])+[trial_id]
                    # Each successful tare anchors the newest observed pose and
                    # torque. Previous zero trials are checked for drift first.
                    if same:
                        check_baseline(old,summary,p,time.time())
                        if any(abs(a-b)>p["max_torque_span"] for a,b in zip(summary["tau"],old["summary"]["tau"])):
                            raise MassError("BASELINE_DRIFT","empty response changed; create a new profile or inspect fixture")
                    baseline={"id":"tare_"+uuid4().hex,"profile_fingerprint":p["fingerprint"],
                              "created_s":time.time(),"zero_trial_ids":zeros,"summary":summary}
                    with self._lock:
                        self._guard_commit(job)
                        self._store.write("trials",trial_id,trial,immutable=True)
                        self._store.write("profiles",baseline["id"],baseline,immutable=True)
                        self._store.write("profiles",p["profile_id"],{"baseline":baseline,"model":None})
                    result.update(baseline_id=baseline["id"],zero_trials=len(zeros),message="tare includes the same fixture; previous model invalidated")
                else:
                    check_baseline(state["baseline"],summary,p,time.time())
                    trial["baseline_id"]=state["baseline"]["id"]
                    if job["action"]=="calibrate":
                        trial.update(known_mass_kg=args["known_mass_kg"],role=args.get("role","train"))
                        with self._lock:
                            self._guard_commit(job)
                            self._store.write("trials",trial_id,trial,immutable=True)
                        result.update(message="independent calibration trial saved; use fit after training and held-out trials")
                    else:
                        model=self._store.read("calibrations",state["model"])
                        measured=estimate(summary,state["baseline"],model,p,time.time())
                        trial["calibration_id"]=model["id"]
                        with self._lock:
                            self._guard_commit(job)
                            self._store.write("trials",trial_id,trial,immutable=True)
                        result.update(measured,valid=True,calibration_id=model["id"],
                                      completed_monotonic_s=time.monotonic(),read_error_count=self._sampler.diagnostics()["read_error_count"])
            result.update(unit="kg",arm=p["arm"],mode="static",profile_id=p["profile_id"],torque_units=p["torque_units"],
                          warnings=["Empirical fixed-pose net mass; no guaranteed hardware accuracy.",
                                    "Getter receipt cannot prove source freshness; independent sample count is unknown.",
                                    "External support, hook slippage and unobserved gain changes cannot always be detected."])
        except Exception as exc:
            result=self._error(getattr(exc,"code","STORAGE" if isinstance(exc,OSError) else "INTERNAL"),str(exc),
                               arm=p["arm"],mode="static",profile_id=p["profile_id"])
        with self._lock:
            if job["cancel"].is_set():
                result=self._error("CANCELLED","sampling cancelled; no motion/disable command sent")
            result["action_id"]=job["id"]
            self._last_profile=p
            self._last_summary=summary if result.get("valid") else None
            self._emit(result)
        if job["notify"]:
            status="cancelled" if result["state"]=="cancelled" else "error" if result["state"]=="error" else "completed"
            try:
                self._notify(job["id"],status,copy.deepcopy(result))
            except Exception as exc:
                print(f"[payload_mass] completion callback failed: {exc}",flush=True)

    def _guard_commit(self,job):
        if job["cancel"].is_set() or self._closed:
            raise MassError("CANCELLED","cancelled before persistence commit")
        current=self._load_profile(job["args"],job["action"]!="tare")
        if current["fingerprint"]!=job["profile"]["fingerprint"]:
            raise MassError("CONFIG_CHANGED","engineering file changed while sampling")
