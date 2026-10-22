#include "overlay_text.hpp"
#include <cassert>
#include <string>
using namespace motus::openxr_capture;
int main() {
  for(char c:std::string("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789._:/+-?")) {
    const auto& glyph=kOverlayGlyphs[c-32];
    unsigned int pixels=0; for(auto row:glyph.rows)pixels|=row;
    assert(pixels && glyph.advance>0 && glyph.advance<=32);
  }
  std::string reason="tracking_lost: release both grips, then hold X+A for 1.0 sec to confirm arms down";
  int count=0, last_y=0;
  LayoutOverlayText(reason,200,8,[&](const OverlayGlyph& glyph,int x,int y){
    assert(x>=0 && x+glyph.advance<=200 && y>=last_y);
    last_y=y; ++count;
  });
  assert(count==static_cast<int>(reason.size()) && last_y>0);
  count=0;
  LayoutOverlayText("A\nB",200,2,[&](const OverlayGlyph&,int x,int y){assert(x==0 && y==30*count++);});
  assert(count==2);
  count=0;
  bool complete=LayoutOverlayText(std::string(60000,'E'),566,3,[&](const OverlayGlyph&,int x,int y){
    assert(x<566 && y<=60); ++count;
  });
  assert(!complete && count<180);
  for(const auto& hint:{"GRIPS OFF  HOLD X AND A 1 SEC", "THEN PRESS BOTH GRIPS",
      "WAIT THEN CONFIRM ARMS DOWN", "GRIPS OFF  HOLD Y AND B 1 SEC TO RESET"}) {
    assert(LayoutOverlayText(hint,566,3,[](const OverlayGlyph&,int,int){}));
  }
  for(const auto& edge:{"", "A\n", "A\nB\nC", "long_reason_without_spaces"}) {
    LayoutOverlayText(edge,50,1,[](const OverlayGlyph& glyph,int x,int y){
      assert(x>=0 && x+glyph.advance<=50 && y==0);
    });
  }
  count=0;
  assert(LayoutOverlayText("松开双握把，长按 X+A 一秒",566,3,[&](const OverlayGlyph& glyph,int,int){
    assert(&glyph!=&OverlayCharacter('?')); ++count;
  }));
  assert(count==15);
  for(const auto& hint:{"本轮已结束。下次遥操按 X+A 重新确认基准",
      "松开任一握把暂停；结束本轮时松开双握把，长按 Y+B 一秒交还控制权",
      "请查看错误原因；交权失败时不可视为本轮已结束"})
    assert(LayoutOverlayText(hint,566,3,[](const OverlayGlyph& glyph,int,int){assert(&glyph!=&OverlayCharacter('?'));}));
  for(const std::string malformed:{"\xff", "\xe4", "\xc0\xaf", "\xed\xa0\x80", "\xf4\x90\x80\x80"})
    LayoutOverlayText(malformed,50,1,[](const OverlayGlyph& glyph,int x,int y){assert(x+glyph.advance<=50 && y==0);});
  assert(!LayoutOverlayText("中文",1,1,[](const OverlayGlyph&,int,int){assert(false);}));

}
