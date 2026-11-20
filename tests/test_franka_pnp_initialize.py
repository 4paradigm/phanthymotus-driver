import json
from pathlib import Path
import runpy
import sys
import tempfile
import threading
from types import SimpleNamespace
from unittest import TestCase, main
from unittest.mock import MagicMock, patch
from franka.fr3.pnp_remote_teleop import PnpRemoteTeleopPlugin
from franka.fr3 import pnp_initialize

class InitializationTests(TestCase):
    def test_config_does_not_return_password(self):
        p=PnpRemoteTeleopPlugin()
        result=p.dispatch('config', {'pnp_host':'172.16.0.200','ssh_user':'pnp','ssh_password':'test-secret'})
        self.assertNotIn('test-secret', json.dumps(result))
        self.assertEqual(result['state'],'configured')
        self.assertEqual(p.get_tools()[0]['configSchema']['properties']['ssh_password']['format'],'password')

    def test_start_installs_without_follow(self):
        p=PnpRemoteTeleopPlugin()
        p.dispatch('config', {'ssh_password':'test-secret', 'pnp_host':'172.16.0.200'})
        entered, release=threading.Event(),threading.Event()
        def install(host,password,progress,**kwargs):
            entered.set()
            release.wait(2)
            return {'state':'ready'}
        with patch.object(pnp_initialize,'initialize', side_effect=install), patch.object(p,'_ssh') as ssh:
            self.assertEqual(p.dispatch('start',{})['state'],'initializing')
            self.assertTrue(entered.wait(1))
            self.assertEqual(p._password,'')
            p.dispatch('follow',{})
            ssh.assert_not_called()
            release.set()
            p._init_thread.join(3)
            self.assertEqual(p._init['state'],'ready')

    def test_failure_redacts_password(self):
        p=PnpRemoteTeleopPlugin()
        p.dispatch('config',{'ssh_password':'test-secret', 'pnp_host':'172.16.0.200'})
        with patch.object(pnp_initialize,'initialize', side_effect=RuntimeError('test-secret failed')):
            p.dispatch('start',{})
            p._init_thread.join(3)
        self.assertNotIn('test-secret',json.dumps(p._init))
        self.assertEqual(p._init['state'],'error')

    def test_remote_program_compiles(self):
        compile(pnp_initialize.REMOTE,'remote-installer','exec')


class VendorIPTests(TestCase):
    def test_vendor_assignments_and_ambiguous_side(self):
        path = Path(__file__).resolve().parents[1] / "franka/fr3/deploy/pnp-remote/read_vendor_ips.py"
        read = runpy.run_path(str(path))["read_vendor_ips"]
        with tempfile.TemporaryDirectory() as directory:
            home = Path(directory)
            root = home / "frankateleop"
            marker = root / "right_franka/1_launch_robot.sh"
            marker.parent.mkdir(parents=True)
            marker.touch()
            conf = root / "polymetis/polymetis/python/polymetis/conf"
            for side, address in (("right", "172.16.0.22"), ("left", "172.16.0.23")):
                for kind, name in (("robot_client", f"{side}_franka_hardware"),
                                   ("gripper", f"{side}_franka_hand")):
                    file = conf / kind / (name + ".yaml")
                    file.parent.mkdir(parents=True, exist_ok=True)
                    file.write_text(f'  robot_ip: "{address}"\n')
            self.assertEqual(read(home)["ips"], {"right": "172.16.0.22", "left": "172.16.0.23"})
            (conf / "gripper/left_franka_hand.yaml").write_text('robot_ip: "172.16.0.99"\n')
            result = read(home)
            self.assertEqual(result["ips"], {"right": "172.16.0.22"})
            self.assertIn("left", result["errors"])


class VendorMasterTests(TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[1] / "franka/fr3/deploy/pnp-remote/read_vendor_masters.py"
        self.read = runpy.run_path(str(path))["read_vendor_masters"]
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.home = Path(self.directory.name)
        self.root = self.home / "frankateleop"
        self.usb = self.home / "usb"
        self.usb.mkdir()
        marker = self.root / "teleop/experiments/run_env.py"
        marker.parent.mkdir(parents=True)
        marker.touch()
        self.agent = self.root / "teleop/teleop/agents/teleop_agent.py"
        self.agent.parent.mkdir(parents=True)
        self.ports = {}
        for side, serial in (("right", "NEW_RIGHT_123"), ("left", "NEW_LEFT_456")):
            name = f"usb-FTDI_USB__-__Serial_Converter_{serial}-if00-port0"
            self.ports[side] = "/dev/serial/by-id/" + name
            (self.usb / name).symlink_to("/dev/null")
            script = self.root / f"{side}_franka/4_run_env.sh"
            script.parent.mkdir(parents=True)
            script.write_text("python3 run_env.py " + chr(92) + "\n --teleop_port=" + self.ports[side] + "\n")
        self.agent.write_text("PORT_CONFIG_MAP = " + repr({p: None for p in self.ports.values()}))

    def test_new_serials_keep_vendor_left_right_roles(self):
        result = self.read(self.home, self.usb)
        self.assertEqual(result["arms"]["right"]["serial"], "NEW_RIGHT_123")
        self.assertEqual(result["arms"]["left"]["serial"], "NEW_LEFT_456")
        self.assertTrue(all(arm["ready"] for arm in result["arms"].values()))

    def test_unmapped_serial_is_not_assigned_by_order(self):
        self.agent.write_text("PORT_CONFIG_MAP = " + repr({self.ports["right"]: None}))
        result = self.read(self.home, self.usb)
        self.assertNotIn("left", result["arms"])
        self.assertIn("left", result["errors"])
        self.assertEqual(len(result["unmatched_usb_devices"]), 1)

    def test_duplicate_master_is_rejected(self):
        (self.root / "left_franka/4_run_env.sh").write_text("python3 run_env.py --teleop_port=" + self.ports["right"])
        result = self.read(self.home, self.usb)
        self.assertEqual(result["arms"], {})
        self.assertIn("configuration", result["errors"])


class AlignmentCompletionTests(TestCase):
    def test_options_and_automatic_initialization(self):
        plugin = PnpRemoteTeleopPlugin()
        definition = plugin.get_tools()[0]
        self.assertEqual(definition["inputSchema"]["properties"]["action"]["enum"],
                         ["start_trajectory", "stop_trajectory", "stop_motion", "list_trajectories", "delete_trajectory", "play", "pause", "resume", "status"])
        self.assertNotIn("auto_initialize", definition["configSchema"]["properties"])
        self.assertEqual(definition["inputSchema"]["x-completion"]["actions"],
                         ["start_trajectory", "play", "stop_trajectory", "stop_motion", "delete_trajectory"])

    def test_visible_stop_preserves_pause_behavior(self):
        plugin = PnpRemoteTeleopPlugin()
        plugin._loaded = True
        schema = plugin.get_tools()[0]["inputSchema"]
        self.assertTrue(all(hook["action"] == "stop_motion" for hook in schema["x-hooks"].values()))
        with patch.object(plugin, "_stop_host") as stop_host:
            self.assertEqual(plugin.dispatch("Stop", {}), {"state": "stop"})
            with patch.object(plugin, "_ssh", return_value="LoadState=loaded\nActiveState=inactive\n"):
                status = plugin.dispatch("Status", {})
            self.assertEqual(status["state"], "stop")
            self.assertEqual(status["hosts"]["right"]["state"], "stop")
            self.assertTrue(plugin._loaded)
            stop_host.assert_called_once_with("right")
            self.assertEqual(plugin.dispatch("stop", {}), {"state": "idle"})
            self.assertFalse(plugin._loaded)

    def test_completed_alignment_posts_acp(self):
        completed = []
        plugin = PnpRemoteTeleopPlugin(acp_callback=lambda *args: completed.append(args))
        plugin._acp = {"action_id": "align-test", "state": "waiting_for_motion"}
        with patch.object(plugin, "_ssh", return_value=self._status_output()):
            plugin._watch_alignment("align-test", threading.Event())
        self.assertEqual(completed[0][:2], ("align-test", "completed"))
        self.assertEqual(plugin._acp["state"], "accepted")

    @staticmethod
    def _status_output(left_stage="vendor_aligned", fresh=True, include_execution=True):
        service = "LoadState=loaded\nActiveState=active\nSubState=running\nResult=success\nMainPID=123\nControlPID=0\n"
        masters = {"right": {"ready": True}, "left": {"ready": True}}
        execution = {"stage": "vendor_starting_arms", "arms": {
            "right": {"stage": "vendor_aligned", "heartbeat_fresh": True},
            "left": {"stage": left_stage, "heartbeat_fresh": fresh},
        }}
        output = service + "Arms=" + json.dumps(masters) + "\n"
        if include_execution:
            output += "Execution=" + json.dumps(execution) + "\n"
        return output

    def test_status_waits_for_both_fresh_arm_snapshots(self):
        plugin = PnpRemoteTeleopPlugin()
        for output in (self._status_output(include_execution=False),
                       self._status_output(left_stage="worker_starting"),
                       self._status_output(fresh=False)):
            with self.subTest(output=output), patch.object(plugin, "_ssh", return_value=output):
                status = plugin._host_status("right")
                self.assertEqual(status["state"], "aligning")
                self.assertTrue(status["masters"]["right"]["ready"])

    def test_stop_requires_confirmed_inactive_service(self):
        plugin = PnpRemoteTeleopPlugin()
        with patch.object(plugin, "_ssh", side_effect=["", self._status_output()]) as ssh:
            with self.assertRaisesRegex(RuntimeError, "停止未确认"):
                plugin._stop_host("right")
            self.assertEqual(ssh.call_args_list[0].args[1:4], ("systemctl", "--user", "stop"))

    def test_follow_requires_confirmed_alignment(self):
        plugin = PnpRemoteTeleopPlugin()
        plugin._loaded = True
        with patch.object(plugin, "_status", return_value={"state": "starting_follow"}), patch.object(plugin, "_ssh") as ssh:
            result = plugin.dispatch("Follow", {})
        self.assertEqual(result["state"], "error")
        ssh.assert_not_called()

class PlaybackStartupTests(TestCase):
    def test_waits_through_empty_buffer_and_stale_feedback_before_starting_gripper(self):
        class EmptyBuffer(Exception):
            def details(self):
                return str(self)

        now = 1000
        stale = SimpleNamespace(seconds=now - 2, nanos=0)
        fresh = SimpleNamespace(seconds=now, nanos=0)
        events = []

        def robot_state(_, timeout):
            self.assertEqual(timeout, .3)
            events.append("arm feedback")
            if events.count("arm feedback") == 1:
                raise EmptyBuffer("Cannot retrieve robot state from empty buffer!")
            stamp = stale if events.count("arm feedback") == 2 else fresh
            return SimpleNamespace(timestamp=stamp, joint_positions=[0.] * 7,
                                   joint_velocities=[0.] * 7)

        def gripper_state(_, timeout):
            self.assertEqual(timeout, .3)
            events.append("gripper feedback")
            stamp = stale if events.count("gripper feedback") == 1 else fresh
            return SimpleNamespace(timestamp=stamp)

        def start(command, **kwargs):
            events.append(command[-1])
            return SimpleNamespace(poll=lambda: None)

        vendor = SimpleNamespace(SCRIPTS=("arm.sh", "hand.sh"), VENDOR_ROOT=Path("/vendor"),
                                 stop_vendor=MagicMock(), vendor_environment=lambda arm: {})
        rpc = SimpleNamespace(
            PolymetisControllerServerStub=lambda channel: SimpleNamespace(GetRobotState=robot_state),
            GripperServerStub=lambda channel: SimpleNamespace(GetState=gripper_state))
        modules = {
            "execution_status": SimpleNamespace(report=MagicMock()),
            "run_vendor": vendor,
            "vendor_recordings": SimpleNamespace(GRIPPER_OPEN_WIDTH_M=.08, NAME=None,
                                                 PLAYBACK_TIMESTAMP="fr3_observed_monotonic_ns"),
            "grpc": SimpleNamespace(RpcError=EmptyBuffer, insecure_channel=MagicMock()),
            "polymetis_pb2": SimpleNamespace(Empty=object),
            "polymetis_pb2_grpc": rpc,
        }
        path = Path(__file__).resolve().parents[1] / "franka/fr3/deploy/pnp-remote/playback.py"
        with patch.dict(sys.modules, modules):
            launch = runpy.run_path(str(path))["_start_backends"]
            with patch.dict(launch.__globals__, {
                "time": SimpleNamespace(time=lambda: now, monotonic=lambda: now, sleep=lambda _: None),
                "subprocess": SimpleNamespace(Popen=start, DEVNULL=-3),
            }):
                launch(SimpleNamespace(side="right", arm_port=50051, hand_port=50053), [])
        self.assertEqual(events, ["arm.sh", "arm feedback", "arm feedback", "arm feedback",
                                  "hand.sh", "gripper feedback", "gripper feedback"])


class DualPlaybackTests(TestCase):
    left = "franka-left-20261119T192346+0800"
    right = "fr3-right-20261119T112346.714284Z"

    def test_play_routes_only_selected_arms_and_preserves_legacy_input(self):
        plugin = PnpRemoteTeleopPlugin()
        plugin._loaded = True
        remote = {"hosts": {"right": {"remote_service": {"ActiveState": "inactive"}}}}
        schema = plugin.get_tools()[0]["inputSchema"]
        self.assertEqual(schema["x-action-params"]["play"]["params"],
                         ["left_trajectory", "right_trajectory"])
        with patch.object(plugin, "_trajectory_hosts", return_value={self.left: "right", self.right: "right"}), \
                patch.object(plugin, "_status", return_value=remote), \
                patch.object(plugin, "_ssh") as ssh, patch("threading.Thread"):
            result = plugin.dispatch("play", {"left_trajectory": self.right})
            self.assertEqual(result["state"], "error")
            ssh.assert_not_called()

            result = plugin.dispatch("play", {"left_trajectory": self.left, "right_trajectory": self.right})
            self.assertEqual(result["selected_arms"], ["left", "right"])
            ssh.assert_called_once_with("right", "pnp-play-recording", self.left, self.right)

            plugin._playback = None
            ssh.reset_mock()
            result = plugin.dispatch("play", {"trajectory": self.right})
            self.assertEqual(result["selected_arms"], ["right"])
            ssh.assert_called_once_with("right", "pnp-play-recording", self.right)

    def test_group_releases_when_both_align_and_waits_for_both_completions(self):
        runtime_path = Path(__file__).resolve().parents[1] / "franka/fr3/deploy/pnp-remote"
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            gate = root / "playback.start"
            approach = root / "playback.approach"
            state_trace = [
                {"left": "playback_entry_ready", "right": "playback_starting"},
                {"left": "playback_entry_ready", "right": "playback_entry_ready"},
                {"left": "playback_aligned", "right": "playback_approaching"},
                {"left": "playback_aligned", "right": "playback_aligned"},
                {"left": "playback_paused", "right": "playback_completed"},
                {"left": "playback_completed", "right": "playback_completed"},
            ]
            step = 0
            reports = []

            def snapshot(path):
                side = path.stem.removeprefix("execution-")
                return {"stage": state_trace[step][side], "duration_s": 2.0}

            def tick(_):
                nonlocal step
                self.assertEqual(approach.exists(), step >= 1)
                self.assertEqual(gate.exists(), step >= 3)
                step += 1

            def spawn(args):
                side = "left" if "-left-" in args[-1] else "right"
                return SimpleNamespace(poll=lambda: 0 if state_trace[step][side] == "playback_completed"
                                       else None, wait=lambda timeout: None)

            execution = SimpleNamespace(report=lambda stage, **_: reports.append(stage), snapshot=snapshot,
                                        run_checked=MagicMock())
            modules = {
                "execution_status": execution,
                "arm_config": SimpleNamespace(ARMS={"left": object(), "right": object()}, detect=MagicMock()),
                "run_vendor": SimpleNamespace(run_vendor=MagicMock()),
                "playback": SimpleNamespace(START=gate, APPROACH=approach, load_trajectory=MagicMock()),
            }
            with patch.dict(sys.modules, modules), patch.object(sys, "path", [str(runtime_path), *sys.path]):
                launch = runpy.run_path(str(runtime_path / "run_right.py"))["launch_playback"]
                with patch.dict(launch.__globals__, {
                    "RUNTIME": root, "check": MagicMock(),
                    "time": SimpleNamespace(monotonic=lambda: 1., sleep=tick),
                    "subprocess": SimpleNamespace(Popen=spawn),
                }):
                    launch([self.left, self.right])
        self.assertEqual(reports, ["playback_starting", "playback_starting", "playback_starting",
                                   "playback_approaching", "playback_running",
                                   "playback_paused", "playback_completed"])


if __name__=='__main__':
    main()
