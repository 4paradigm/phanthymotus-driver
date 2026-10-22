#pragma once
#include <nlohmann/json.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace motus::openxr_capture {
using IkPoint = std::array<float,3>;
using IkChains = std::vector<std::vector<IkPoint>>;
// Robot FLU (+X forward, +Y left, +Z up) to head-relative OpenXR
// Rear eye-level view: forward goes away, left stays left, height goes up.
// The shoulder reference sits at eye level so above/below remains unambiguous.
inline IkPoint IkOperatorViewPoint(IkPoint p) {
  return {-.7f*p[1], .7f*(p[2]-.29178f), -1.3f-.7f*p[0]};
}

struct IkVisualization {
  bool available{false}, feedback_fresh{false}, stale{false};
  std::string mode, state, reason, control_reason;
  double intent_age_ms{1e9};
  IkChains measured, ik, command, body, held_ik;
  bool tianyi{false}, unified{false}, teleop{false}, alignment_ready{false};
  std::string alignment_state{"unavailable"};
  std::array<double,2> position_error_m{}, orientation_error_rad{};
  double feedback_age_ms{0};
  bool ready() const {
    return fresh() && feedback_fresh && current_ik() && alignment_ready;
  }
  std::string alignment_label() const {
    if(!fresh() || !feedback_fresh || alignment_state=="unavailable")return "ALIGNMENT UNAVAILABLE";
    if(alignment_state=="unreachable")return "TARGET UNREACHABLE";
    if(!current_ik())return "ALIGNMENT UNAVAILABLE";
    if(ready())return "READY";
    if(alignment_state=="active")return "ACTIVE";
    return "NOT ALIGNED";
  }
  float shoulder_height{.29178f};
  std::vector<IkPoint> targets, requested_targets;
  IkChains workspace_bounds;
  bool target_limited{false}, boundary_search_pending{false};
  std::chrono::steady_clock::time_point received{};
  bool control_fresh() const {
    return teleop && !state.empty() && feedback_age_ms + std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-received).count() < 750;
  }
  bool control_released{false};
  bool reset_failed() const {
    return control_reason.starts_with("reset_") || control_reason=="release_sdk_error"
      || control_reason=="release_timeout" || control_reason=="release_action_stop_unconfirmed";
  }
  std::string control_label(bool connected=true, bool assigned=true) const {
    if(!connected)return "等待设备连接";
    if(!assigned)return "尚未启动智能控制";
    if(!control_fresh())return !teleop || reason=="feedback_unavailable" ? "等待机器人反馈" : "机器人反馈已失效";
    if(control_reason=="waiting_baseline_confirmation")return control_released ? "控制权已交还" : "准备建立基准";
    if(control_reason=="release_buttons_then_regrip")return "请松开组合键";
    if(control_reason=="regrip_required")return "双握把接通";
    if(control_reason=="resetting")return "正在交还控制权";
    if(reset_failed())return "交权未完成";
    if(state=="active")return "遥操已接通";
    if(state=="hold")return "遥操已暂停";
    if(state=="fault" || state=="error")return "遥操异常";
    return "等待机器人就绪";
  }
  std::string control_hint(bool connected=true, bool assigned=true) const {
    if(!connected)return "请返回 App 检查连接";
    if(!assigned)return "请在 Canvas 上启动智能控制";
    if(!control_fresh())return "反馈有效后再按提示操作；当前姿态未知";
    if(control_reason=="waiting_baseline_confirmation")return control_released
      ? "本轮已结束。下次遥操按 X+A 重新确认基准"
      : "松开双握把，确认双方双臂下垂；长按 X+A 一秒并保持，等待基准准备完成";
    if(control_reason=="release_buttons_then_regrip")return "松开 X+A 后，重新按下双握把接通";
    if(control_reason=="regrip_required")return "重新按下双握把接通，松开任一握把暂停";
    if(control_reason=="resetting")return "请保持松开双握把，等待控制权交还确认";
    if(reset_failed())return "控制权尚未确认交还；请查看错误原因";
    if(state=="fault" || state=="error")return "请查看错误原因；交权失败时不可视为本轮已结束";
    if(state=="hold")return "松开双握把；需结束本轮时长按 Y+B 一秒交还控制权";
    return "松开任一握把暂停；结束本轮时松开双握把，长按 Y+B 一秒交还控制权";
  }
  bool current_ik() const {
    return fresh() && intent_age_ms + std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-received).count() < (teleop ? 300 : 750) && !ik.empty();
  }
  const IkChains& displayed_ik() const {
    if(teleop) return ik; // Historical display never restores freshness or readiness.
    return current_ik() || !tianyi ? ik : (held_ik.empty() ? ik : held_ik);
  }
  bool fresh() const {
    const auto age=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-received).count();
    return available && !stale && age + (teleop ? feedback_age_ms : 0) < 750;
  }
};
inline IkPoint ParseIkPoint(const nlohmann::json& value) {
  if (!value.is_array() || value.size()!=3) throw std::invalid_argument("ik_point");
  IkPoint result{};
  for (int i=0;i<3;++i) {
    if (!value[i].is_number()) throw std::invalid_argument("ik_coordinate");
    double x=value[i].get<double>();
    if (!std::isfinite(x) || std::abs(x)>4) throw std::invalid_argument("ik_coordinate");
    result[i]=static_cast<float>(x);
  }
  return result;
}
inline IkChains ParseIkChains(const nlohmann::json& value, std::size_t count=6) {
  if (!value.is_array() || (value.size()!=0 && value.size()!=2)) throw std::invalid_argument("ik_chains");
  IkChains result;
  for (const auto& chain:value) {
    if (!chain.is_array() || chain.size()!=count) throw std::invalid_argument("ik_chain");
    std::vector<IkPoint> points;
    for (const auto& point:chain) points.push_back(ParseIkPoint(point));
    result.push_back(std::move(points));
  }
  return result;
}
inline IkVisualization ParseIkVisualization(const nlohmann::json& envelope) {
  const bool teleop=envelope.value("schema",std::string{})=="motus.teleop.feedback/1";
  const bool motion=teleop || envelope.value("schema",std::string{})=="motus.motion.feedback/1";
  if(motion && !envelope.value("available",true)) {
    IkVisualization empty; empty.teleop=teleop;empty.unified=teleop; empty.reason=envelope.contains("reason") && envelope["reason"].is_string()?envelope["reason"].get<std::string>():"feedback_unavailable";return empty;
  }
  if(motion) {
    if(envelope.dump().size()>65536)throw std::invalid_argument("feedback_size");
    IkVisualization r;r.teleop=teleop;r.tianyi=true;r.unified=true;r.received=std::chrono::steady_clock::now();
    r.mode=envelope.value("mode",std::string("shadow"));r.state=envelope.value("state",std::string{});
    if(envelope.contains("reason") && envelope["reason"].is_string())r.control_reason=envelope["reason"].get<std::string>();
    if(teleop && envelope.contains("execution") && envelope.at("execution").is_object()) {
      const auto& execution=envelope.at("execution");
      // Only the confirmed handback receipt ends this round.
      r.control_released=r.state=="ready" && r.control_reason=="waiting_baseline_confirmation"
        && execution.value("state",std::string{})=="idle"
        && execution.contains("reason") && execution.at("reason")=="release_confirmed"
        && !execution.value("ownership_held",true) && !execution.value("output_active",true);
    }
    r.feedback_age_ms=envelope.value("feedback_age_ms",0.0);
    if(!std::isfinite(r.feedback_age_ms)||r.feedback_age_ms<0)throw std::invalid_argument("feedback_age");
    if(!envelope.contains("visualization"))return r;
    const auto& v=envelope.at("visualization");
    r.feedback_age_ms=v.value("feedback_age_ms",r.feedback_age_ms);
    if(!std::isfinite(r.feedback_age_ms)||r.feedback_age_ms<0)throw std::invalid_argument("feedback_age");
    r.stale=v.value("stale",false);
    r.available=v.value("available",false);r.reason=v.value("reason",std::string{});
    if(!r.available)return r;
    r.feedback_fresh=v.value("feedback_fresh",false);
    r.intent_age_ms=v.value("intent_age_ms",1e9);
    if(!std::isfinite(r.intent_age_ms)||r.intent_age_ms<0)throw std::invalid_argument("feedback_age");
    auto chains=[&](const char* key,bool paired) {
      IkChains out;if(!v.contains(key))return out;
      const auto& paths=v.at(key);
      if(!paths.is_array()||paths.size()>(paired?2:16))throw std::invalid_argument("feedback_chains");
      for(const auto& path:paths){
        if(!path.is_array()||path.size()<2||path.size()>32)throw std::invalid_argument("feedback_chain");
        std::vector<IkPoint> points;for(const auto& point:path)points.push_back(ParseIkPoint(point));out.push_back(std::move(points));
      }
      return out;
    };
    r.measured=chains("measured",true);r.ik=chains("ik",true);r.held_ik=chains("held_ik",true);r.body=chains("torso",false);
    if(v.contains("targets")){
      if(!v.at("targets").is_array()||v.at("targets").size()>2)throw std::invalid_argument("feedback_targets");
      for(const auto& point:v.at("targets"))r.targets.push_back(ParseIkPoint(point));
    }
    r.shoulder_height=v.value("shoulder_height",.29178f);
    if(!std::isfinite(r.shoulder_height)||std::abs(r.shoulder_height)>2)throw std::invalid_argument("feedback_shoulder");
    if(teleop) {
      if(envelope.contains("alignment")){
        const auto& a=envelope.at("alignment");
        r.alignment_state=a.at("state").get<std::string>();
        r.alignment_ready=a.at("ready").get<bool>();
        if(r.alignment_state!="ready" && r.alignment_state!="unaligned" && r.alignment_state!="active"
           && r.alignment_state!="unreachable" && r.alignment_state!="unavailable")throw std::invalid_argument("alignment_state");
        if(r.alignment_ready!=(r.alignment_state=="ready"))throw std::invalid_argument("alignment_ready");
        if(r.alignment_state!="unavailable")for(int i=0;i<2;++i){
          const auto& arm=a.at(i==0?"left":"right");
          auto number=[&](const char* key,double max){double x=arm.at(key).get<double>();if(!std::isfinite(x)||x<0||x>max)throw std::invalid_argument("alignment_error");return x;};
          r.position_error_m[i]=number("position_error_m",8);
          r.orientation_error_rad[i]=number("orientation_error_rad",3.141592653589793);
          number("position_tolerance_m",8);number("orientation_tolerance_rad",3.141592653589793);
        }
        if(r.alignment_ready && (!r.feedback_fresh || r.measured.size()!=2 || r.ik.size()!=2))throw std::invalid_argument("alignment_skeletons");
      }
    }
    if(!r.feedback_fresh)r.measured.clear();
    return r;
  }
  const auto& value=envelope;
  if (value.dump().size()>16384 || (value.at("schema")!="motus.g1-visualization.v1" && value.at("schema")!="motus.tianyi-visualization.v1")) throw std::invalid_argument("ik_schema");
  IkVisualization result;
  result.tianyi=value.at("schema")=="motus.tianyi-visualization.v1";
  result.available=value.at("available").get<bool>();
  result.received=std::chrono::steady_clock::now();
  auto text=[&](const char* name){auto s=value.value(name,std::string{});if(s.size()>160)throw std::invalid_argument("ik_text");return s;};
  result.reason=text("reason");
  if (!result.available) return result;
  result.mode=text("mode");result.state=text("state");
  if(result.mode!="live" && result.mode!="shadow")throw std::invalid_argument("ik_mode");
  result.feedback_fresh=value.at("feedback_fresh").get<bool>();
  if(!value.at("intent_age_ms").is_null())result.intent_age_ms=value.at("intent_age_ms").get<double>();
  if(!std::isfinite(result.intent_age_ms) || result.intent_age_ms<0)throw std::invalid_argument("ik_age");
  result.measured=ParseIkChains(value.at("measured"),result.tianyi?8:6);
  result.ik=ParseIkChains(value.at("ik"),result.tianyi?8:6);result.command=ParseIkChains(value.at("command"),result.tianyi?8:6);
  if(result.tianyi && value.contains("held_ik"))result.held_ik=ParseIkChains(value.at("held_ik"),8);
  const auto& targets=value.at("targets");
  if(!targets.is_array() || (targets.size()!=0 && targets.size()!=2))throw std::invalid_argument("ik_targets");
  for(const auto& target:targets)result.targets.push_back(ParseIkPoint(target));
  if(result.tianyi){
    if(value.contains("requested_targets")){
      const auto& points=value.at("requested_targets");
      if(!points.is_array() || (points.size()!=0 && points.size()!=2))throw std::invalid_argument("ik_requested_targets");
      for(const auto& point:points)result.requested_targets.push_back(ParseIkPoint(point));
    }
    if(value.contains("workspace_bounds")){
      result.workspace_bounds=ParseIkChains(value.at("workspace_bounds"),2);
      for(const auto& box:result.workspace_bounds)for(int i=0;i<3;++i)
        if(box[0][i]>=box[1][i])throw std::invalid_argument("ik_workspace_bounds");
    }
    if(value.contains("projection") && !value.at("projection").is_null()){
      const auto& p=value.at("projection");
      const auto& limited=p.at("limited");const auto& reasons=p.at("reasons");
      if(!limited.is_array() || limited.size()!=2 || !reasons.is_array() || reasons.size()!=2)throw std::invalid_argument("ik_projection");
      for(int i=0;i<2;++i){result.target_limited |= limited[i].get<bool>();
        if(reasons[i]=="search_budget")result.boundary_search_pending=true;}
    }
    result.shoulder_height=value.at("shoulder_height").get<float>();
    if(!std::isfinite(result.shoulder_height) || std::abs(result.shoulder_height)>2)throw std::invalid_argument("ik_shoulder");
    const auto& body=value.at("body");
    if(!body.is_array() || body.size()>16)throw std::invalid_argument("ik_body");
    for(const auto& path:body){
      if(!path.is_array() || path.size()<2 || path.size()>16)throw std::invalid_argument("ik_body_path");
      std::vector<IkPoint> points;for(const auto& point:path)points.push_back(ParseIkPoint(point));
      result.body.push_back(std::move(points));
    }
  }
  if(!result.feedback_fresh)result.measured.clear();
  return result;
}
// Transient optional-preview failures must not erase the last complete model.
// Mark it stale immediately; never borrow an old model after calibration removal.
inline void UpdateIkVisualization(IkVisualization& current, const nlohmann::json& value) {
  auto next=ParseIkVisualization(value);
  if(!next.teleop && !next.available && current.tianyi && current.available &&
     (next.reason=="visualization_unavailable" || next.reason=="feedback_unavailable" || (current.unified && next.reason=="feedback_stale"))) {
    current.reason=next.reason;
    current.feedback_fresh=false;
    current.received=std::chrono::steady_clock::now()-std::chrono::seconds(1);
    return;
  }
  current=std::move(next);
}
} // namespace motus::openxr_capture
