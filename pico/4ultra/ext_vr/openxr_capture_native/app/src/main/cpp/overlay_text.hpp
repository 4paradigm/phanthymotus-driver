#pragma once
#include "operator_labels.hpp"
#include <algorithm>
#include <string_view>

namespace motus::openxr_capture {
inline const OverlayGlyph& OverlayCharacter(unsigned int code) {
  const auto found=std::lower_bound(kOverlayCodepoints.begin(),kOverlayCodepoints.end(),code);
  return kOverlayGlyphs[found!=kOverlayCodepoints.end() && *found==code
    ? found-kOverlayCodepoints.begin() : '?'-32];
}
inline unsigned int OverlayCodepoint(std::string_view text, std::size_t& offset) {
  const unsigned char first=text[offset++];
  if(first<128)return first;
  const int count=first>=0xc2 && first<=0xdf ? 2 : first>=0xe0 && first<=0xef ? 3 : first>=0xf0 && first<=0xf4 ? 4 : 0;
  if(!count || offset+count-1>text.size())return '?';
  unsigned int code=first & ((1u<<(7-count))-1);
  for(int i=0;i<count-1;++i) {
    const unsigned char c=text[offset+i];
    if((c&0xc0)!=0x80)return '?';
    code=(code<<6)|(c&0x3f);
  }
  if(code<(count==2?0x80u:count==3?0x800u:0x10000u) || code>0x10ffff || (code>=0xd800 && code<=0xdfff))return '?';
  offset+=count-1;
  return code;
}
// Bound both scanning and geometry by the visible area, including untrusted reasons.
template<class Emit>
bool LayoutOverlayText(std::string_view text, int width, int lines, Emit emit) {
  if(width<=0 || lines<=0)return text.empty();
  struct Placed { const OverlayGlyph* glyph; int x,row; };
  std::vector<Placed> placed;
  int x=0,row=0;
  bool complete=true;
  for(std::size_t offset=0;offset<text.size();) {
    const auto code=OverlayCodepoint(text,offset);
    if(code=='\n') {
      if(++row>=lines){complete=false;break;}
      x=0;continue;
    }
    const auto& glyph=OverlayCharacter(code);
    if(glyph.advance>width){complete=false;break;}
    if(x+glyph.advance>width){x=0;++row;}
    if(row>=lines){complete=false;break;}
    placed.push_back({&glyph,x,row});x+=glyph.advance;
  }
  if(!complete) {
    const auto& dot=OverlayCharacter('.');
    row=placed.empty()?0:placed.back().row;
    x=placed.empty()?0:placed.back().x+placed.back().glyph->advance;
    while(!placed.empty() && placed.back().row==row && x+3*dot.advance>width) {
      x=placed.back().x;placed.pop_back();
    }
    for(int i=0;i<3 && x+dot.advance<=width;++i) {
      placed.push_back({&dot,x,row});x+=dot.advance;
    }
  }
  for(const auto& item:placed)emit(*item.glyph,item.x,item.row*30);
  return complete;
}
}
