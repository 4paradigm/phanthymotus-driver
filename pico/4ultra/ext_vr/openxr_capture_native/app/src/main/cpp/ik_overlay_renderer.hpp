#pragma once
#include "ik_visualization.hpp"
#include "overlay_text.hpp"
#include <GLES3/gl3.h>
#include <openxr/openxr.h>
#include <algorithm>
#include <cstdio>

namespace motus::openxr_capture {
// A binocular, head-relative model panel. No world/robot extrinsic is implied.
class IkOverlayRenderer {
 public:
  void Destroy() {
    if(program_)glDeleteProgram(program_);
    if(buffer_)glDeleteBuffers(1,&buffer_);
    if(vao_)glDeleteVertexArrays(1,&vao_);
    program_=buffer_=vao_=0;
  }
  void Draw(const IkVisualization& value,const XrView& eye,const XrPosef& center,bool connected,bool show_model=false,bool assigned=true) {
    if(!program_) Initialize();
    std::vector<float> vertices, text_outline, text_fill;
    auto* destination=&vertices;
    auto rotate=[](XrQuaternionf q,IkPoint p){
      IkPoint u{q.x,q.y,q.z};
      auto cross=[](IkPoint a,IkPoint b){return IkPoint{a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};};
      auto uv=cross(u,p),uuv=cross(u,uv);
      for(int i=0;i<3;++i)p[i]+=2*(q.w*uv[i]+uuv[i]);
      return p;
    };
    const auto inverse=XrQuaternionf{-eye.pose.orientation.x,-eye.pose.orientation.y,-eye.pose.orientation.z,eye.pose.orientation.w};
    float alpha=1.f;
    auto vertex=[&](IkPoint p,IkPoint color){
      auto world=rotate(center.orientation,p);
      world[0]+=center.position.x-eye.pose.position.x;
      world[1]+=center.position.y-eye.pose.position.y;
      world[2]+=center.position.z-eye.pose.position.z;
      auto view=rotate(inverse,world);
      const float left=std::tan(eye.fov.angleLeft),right=std::tan(eye.fov.angleRight);
      const float down=std::tan(eye.fov.angleDown),up=std::tan(eye.fov.angleUp);
      float depth=-view[2];
      destination->insert(destination->end(),{2*view[0]/(right-left)-(right+left)/(right-left)*depth,
                                     2*view[1]/(up-down)-(up+down)/(up-down)*depth,0,depth,
                                     color[0],color[1],color[2],alpha});
    };
    auto line=[&](IkPoint a,IkPoint b,IkPoint color){vertex(a,color);vertex(b,color);};
    const IkPoint green{.25f,1.f,.65f},orange{1.f,.65f,.2f},pink{1.f,.35f,.85f},gray{.94f,.95f,.96f};
    auto model=[&](IkPoint p){p[2]+=.29178f-value.shoulder_height;return IkOperatorViewPoint(p);};
    auto quad=[&](float left,float top,float right,float bottom,IkPoint color){
      vertex({left,top,-1.3f},color);vertex({left,bottom,-1.3f},color);vertex({right,top,-1.3f},color);
      vertex({right,top,-1.3f},color);vertex({left,bottom,-1.3f},color);vertex({right,bottom,-1.3f},color);
    };
    auto text=[&](const std::string& s,float x,float y,IkPoint color,int lines=1){
      constexpr float pixel=.0012f, outline=.0012f;
      LayoutOverlayText(s,566,lines,[&](const OverlayGlyph& glyph,int gx,int gy){
        for(int row=0;row<24;++row)for(int col=0;col<32;) {
          if(!(glyph.rows[row] & (1u<<col))) { ++col; continue; }
          int start=col++;
          while(col<32 && (glyph.rows[row] & (1u<<col)))++col;
          float left=x+(gx+start)*pixel,right=x+(gx+col)*pixel,top=y-(gy+row)*pixel;
          destination=&text_outline;
          quad(left-outline,top+outline,right+outline,top-pixel-outline,{.035f,.04f,.045f});
          destination=&text_fill;
          quad(left,top,right,top-pixel,color);
        }
      });
      destination=&vertices;
    };
    if(!show_model){
      for(const auto& pixel:kOperatorLabelPixels){
        const bool status=pixel[0]==(connected?0:1);
        if(!status && pixel[0]!=2)continue;
        const float x=(status?-.12f:-.20f)+pixel[1]*.002f;
        const float y=(status?.52f:.44f)-pixel[2]*.002f;
        line({x,y,-1.3f},{x+.0016f,y,-1.3f},connected?green:gray);
      }
    }
    std::size_t measured_begin=0,measured_count=0;
    if(show_model) {
    if(value.teleop || !value.available){
      text(value.control_label(connected,assigned),-.34f,.39f,value.control_fresh() && value.state=="active"?green:
        value.control_fresh() && value.state!="fault" && value.state!="error"?orange:pink);
      text(value.control_hint(connected,assigned),-.34f,.62f,gray,3);
      if(connected && assigned && value.control_fresh() && (value.state=="hold" || value.state=="fault" || value.state=="error"))
        text("原因："+value.control_reason,-.34f,-.60f,pink,2);
      if(connected && assigned && value.fresh() && value.feedback_fresh && value.alignment_state!="unavailable"){
        for(int i=0;i<2;++i){
          char error[48];std::snprintf(error,sizeof(error),"%s %d 毫米  %d 度",i==0?"左":"右",
            static_cast<int>(std::round(value.position_error_m[i]*1000)),
            static_cast<int>(std::round(value.orientation_error_rad[i]*180/3.141592653589793)));
          text(error,-.34f,.48f-i*.04f,gray);
        }
      }
    }
    text(value.teleop || !value.available ? "机器人背面视角" : "ROBOT REAR VIEW",-.34f,.34f,gray);
    text(value.teleop || !value.available ? "左臂" : "LEFT",-.23f,-.08f,gray);text(value.teleop || !value.available ? "右臂" : "RIGHT",.16f,-.08f,gray);
    if((value.teleop && (!connected || !assigned)) || !value.available || (!value.fresh() && !value.tianyi)){
      text(value.teleop || !value.available ? "暂无手臂预览" : "NO ARM PREVIEW",-.16f,.08f,gray);
    }else{
      if(!value.teleop)text(value.mode+" / "+value.state,-.34f,.30f,gray);
      if(!value.teleop)text("MEASURED",-.34f,.25f,green);
      text(value.teleop ? "橙色：映射目标" : "MAPPED TARGET",-.10f,.25f,orange);
      // Fixed orientation reference, not tracked torso/head feedback.
      auto body=[&](IkPoint a,IkPoint b){line(model(a),model(b),gray);};
      if(value.tianyi){
        for(const auto& path:value.body)for(std::size_t i=1;i<path.size();++i)body(path[i-1],path[i]);
      }else{
      body({0,.10022f,.29178f},{0,.075f,0});
      body({0,-.10021f,.29178f},{0,-.075f,0});
      body({0,.075f,0},{0,-.075f,0});
      body({0,0,0},{0,0,.34f}); // back centre and neck
      body({0,-.045f,.34f},{0,.045f,.34f});
      body({0,.045f,.34f},{0,.06f,.37f});
      body({0,.06f,.37f},{0,.06f,.45f});
      body({0,.06f,.45f},{0,.035f,.48f});
      body({0,.035f,.48f},{0,-.035f,.48f});
      body({0,-.035f,.48f},{0,-.06f,.45f});
      body({0,-.06f,.45f},{0,-.06f,.37f});
      body({0,-.06f,.37f},{0,-.045f,.34f});
      }
      text(value.teleop ? "身体方向参照" : "BODY REFERENCE",-.13f,-.36f,gray);
      // Calibrated outer bounds are an aid, never a solid reachable volume.
      const IkPoint boundary{.25f,.45f,.5f};
      for(const auto& box:value.workspace_bounds){
        for(int mask=0;mask<8;++mask)for(int axis=0;axis<3;++axis)if(!(mask&(1<<axis))){
          IkPoint a{},b{};for(int k=0;k<3;++k)a[k]=box[(mask>>k)&1][k];
          b=a;b[axis]=box[1][axis];line(model(a),model(b),boundary);
        }
      }
      if(!value.workspace_bounds.empty())text("OUTER SAFETY BOUNDS",-.34f,-.55f,boundary);
      auto draw=[&](const IkChains& chains,IkPoint color){for(const auto& chain:chains)for(std::size_t i=1;i<chain.size();++i)line(model(chain[i-1]),model(chain[i]),color);};
      if(!value.teleop && value.fresh() && value.feedback_fresh){
        measured_begin=vertices.size()/8;
        draw(value.measured,green);
        measured_count=vertices.size()/8-measured_begin;
      }
      else if(!value.teleop)text("MEASURED STALE",-.34f,-.47f,pink);
      double age=value.intent_age_ms+std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-value.received).count();
      if(value.tianyi){
        const bool current=value.current_ik();
        // Keep geometry brightness stable across transient solve failures.
        // Explicit HELD/STALE labels distinguish history from a current result.
        alpha=1.f;
        draw(value.displayed_ik(),orange);
        alpha=1.f;
        if(!value.teleop && !current && !value.displayed_ik().empty())text("LAST VALID - HELD",-.34f,-.39f,orange);
        if(!value.fresh())text(value.teleop ? "反馈已过期" : "NO FRESH DATA",-.16f,.08f,pink);
      }
      if(value.fresh() && age<(value.teleop?300:750)){
        if(!value.tianyi)draw(value.ik,orange);
        for(auto p:value.targets){auto t=model(p);line({t[0]-.009f,t[1],t[2]},{t[0]+.009f,t[1],t[2]},pink);line({t[0],t[1]-.009f,t[2]},{t[0],t[1]+.009f,t[2]},pink);}
      }else text(value.teleop ? "目标已过期" : "TARGET STALE",-.34f,-.43f,pink);
      if(!value.teleop && !value.reason.empty())text(value.reason,-.34f,-.60f,pink,2);
      if((!value.teleop || (value.fresh() && value.feedback_fresh)) && value.measured.size()==2)line(model(value.measured[0][0]),model(value.measured[1][0]),gray);
    }
    }
    const auto line_count=vertices.size()/8;
    vertices.insert(vertices.end(),text_outline.begin(),text_outline.end());
    vertices.insert(vertices.end(),text_fill.begin(),text_fill.end());
    glUseProgram(program_);glBindVertexArray(vao_);glBindBuffer(GL_ARRAY_BUFFER,buffer_);
    glBufferData(GL_ARRAY_BUFFER,vertices.size()*sizeof(float),vertices.data(),GL_STREAM_DRAW);
    glEnableVertexAttribArray(0);glVertexAttribPointer(0,4,GL_FLOAT,GL_FALSE,8*sizeof(float),nullptr);
    glEnableVertexAttribArray(1);glVertexAttribPointer(1,4,GL_FLOAT,GL_FALSE,8*sizeof(float),reinterpret_cast<void*>(4*sizeof(float)));
    glDisable(GL_DEPTH_TEST);glEnable(GL_BLEND);glBlendFuncSeparate(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA,GL_ONE,GL_ONE_MINUS_SRC_ALPHA);glLineWidth(2);
    // Measured outline remains visible when the thinner IK/command overlaps it.
    if(measured_count){
      glLineWidth(6);
      glDrawArrays(GL_LINES,static_cast<GLint>(measured_begin),static_cast<GLsizei>(measured_count));
    }
    glLineWidth(2);
    glDrawArrays(GL_LINES,0,static_cast<GLsizei>(line_count));
    glDrawArrays(GL_TRIANGLES,static_cast<GLint>(line_count),
      static_cast<GLsizei>((text_outline.size()+text_fill.size())/8));
    glDisable(GL_BLEND);
    glBindVertexArray(0);glBindBuffer(GL_ARRAY_BUFFER,0);glUseProgram(0);
  }
 private:
  GLuint program_{0},buffer_{0},vao_{0};
  void Initialize(){
    auto shader=[](GLenum type,const char* source){GLuint id=glCreateShader(type);glShaderSource(id,1,&source,nullptr);glCompileShader(id);GLint ok=0;glGetShaderiv(id,GL_COMPILE_STATUS,&ok);if(!ok){glDeleteShader(id);throw std::runtime_error("IK overlay shader compile failed");}return id;};
    GLuint vs=shader(GL_VERTEX_SHADER,"#version 300 es\nlayout(location=0) in vec4 p;layout(location=1) in vec4 c;out vec4 color;void main(){gl_Position=p;color=c;}");
    GLuint fs=shader(GL_FRAGMENT_SHADER,"#version 300 es\nprecision mediump float;in vec4 color;out vec4 frag;void main(){frag=color;}");
    program_=glCreateProgram();glAttachShader(program_,vs);glAttachShader(program_,fs);glLinkProgram(program_);glDeleteShader(vs);glDeleteShader(fs);
    GLint ok=0;glGetProgramiv(program_,GL_LINK_STATUS,&ok);if(!ok){Destroy();throw std::runtime_error("IK overlay shader link failed");}
    glGenBuffers(1,&buffer_);glGenVertexArrays(1,&vao_);
  }
};
} // namespace
