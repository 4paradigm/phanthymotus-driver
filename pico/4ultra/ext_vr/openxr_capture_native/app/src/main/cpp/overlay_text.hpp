#pragma once
#include "operator_labels.hpp"
#include <string_view>

namespace motus::openxr_capture {
inline const OverlayGlyph& OverlayCharacter(unsigned char c) {
  return kOverlayGlyphs[(c>=32 && c<=126 ? c : '?')-32];
}
// Bound both work and height for untrusted robot reasons. Ellipsis is explicit.
template<class Emit>
bool LayoutOverlayText(std::string_view text, int width, int lines, Emit emit) {
  int x=0, row=0;
  std::size_t end=0;
  for(;end<text.size();++end) {
    unsigned char c=text[end];
    const auto& glyph=OverlayCharacter(c);
    if(c=='\n' || (x && x+glyph.advance>width)) {
      if(row+1>=lines)break;
      x=0; ++row;
    }
    if(c!='\n')x+=glyph.advance;
  }
  const bool complete=end==text.size();
  if(!complete) {
    const int suffix=3*OverlayCharacter('.').advance;
    while(end && x+suffix>width) x-=OverlayCharacter(text[--end]).advance;
  }
  x=0; row=0;
  for(unsigned char c:text.substr(0,end)) {
    const auto& glyph=OverlayCharacter(c);
    if(c=='\n' || (x && x+glyph.advance>width)) { x=0; ++row; }
    if(c=='\n')continue;
    emit(glyph,x,row*30); x+=glyph.advance;
  }
  if(!complete)for(int i=0;i<3;++i) {
    const auto& dot=OverlayCharacter('.'); emit(dot,x,row*30); x+=dot.advance;
  }
  return complete;
}
}
