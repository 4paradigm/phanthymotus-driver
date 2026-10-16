// Development-only: swift scripts/generate_operator_labels.swift > app/src/main/cpp/operator_labels.hpp
import AppKit
let labels=["已配对连接","等待设备连接","按住双握把开始遥操"]
print("#pragma once\n#include <array>\n#include <vector>\nnamespace motus::openxr_capture {\ninline const std::vector<std::array<int,3>> kOperatorLabelPixels = {")
for (n,label) in labels.enumerated() {
 let width=220, height=24
 let rep=NSBitmapImageRep(bitmapDataPlanes:nil,pixelsWide:width,pixelsHigh:height,bitsPerSample:8,samplesPerPixel:4,hasAlpha:true,isPlanar:false,colorSpaceName:.deviceRGB,bytesPerRow:width*4,bitsPerPixel:32)!
 NSGraphicsContext.saveGraphicsState();NSGraphicsContext.current=NSGraphicsContext(bitmapImageRep:rep)
 NSColor.black.setFill();NSRect(x:0,y:0,width:width,height:height).fill()
 (label as NSString).draw(at:NSPoint(x:0,y:0),withAttributes:[.font:NSFont.systemFont(ofSize:20),.foregroundColor:NSColor.white])
 NSGraphicsContext.restoreGraphicsState()
 for y in 0..<height {for x in 0..<width {
  if rep.colorAt(x:x,y:y)!.redComponent > 0.45 {print("{\(n),\(x),\(y)},")}
 }
}
}
print("};")
// Printable ASCII covers status, decimal errors and machine-readable reasons.
print("struct OverlayGlyph { int advance; std::array<unsigned int,24> rows; };\ninline constexpr std::array<OverlayGlyph,95> kOverlayGlyphs = {{")
let font=NSFont.systemFont(ofSize:20, weight:.semibold)
for code in 32...126 {
 let label=String(UnicodeScalar(code)!) as NSString
 let rep=NSBitmapImageRep(bitmapDataPlanes:nil,pixelsWide:32,pixelsHigh:24,bitsPerSample:8,samplesPerPixel:4,hasAlpha:true,isPlanar:false,colorSpaceName:.deviceRGB,bytesPerRow:128,bitsPerPixel:32)!
 NSGraphicsContext.saveGraphicsState();NSGraphicsContext.current=NSGraphicsContext(bitmapImageRep:rep)
 NSColor.black.setFill();NSRect(x:0,y:0,width:32,height:24).fill()
 label.draw(at:NSPoint(x:0,y:0),withAttributes:[.font:font,.foregroundColor:NSColor.white])
 NSGraphicsContext.restoreGraphicsState()
 let advance=Int(ceil(label.size(withAttributes:[.font:font]).width))+1
 precondition(advance <= 32)
 var rows=[String]()
 for y in 0..<24 {
  var bits:UInt32=0
  for x in 0..<32 {if rep.colorAt(x:x,y:y)!.redComponent > 0.45 {bits |= UInt32(1) << x}}
  rows.append(String(bits)+"u")
 }
 print("{\(advance), {"+rows.joined(separator:",")+"}},")
}
print("}};\n}")
