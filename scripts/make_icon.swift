// Renders the macOS app icon master (1024x1024) from a square source image:
// the artwork is clipped to the standard macOS icon shape (824x824 rounded
// square centred on the 1024 canvas, transparent margin) with a soft shadow.
// Usage: swift make_icon.swift <source.png> <master-1024.png>
import AppKit
import CoreGraphics

let args = CommandLine.arguments
guard args.count == 3, let src = NSImage(contentsOfFile: args[1]),
      let srcCG = src.cgImage(forProposedRect: nil, context: nil, hints: nil) else {
    FileHandle.standardError.write("usage: make_icon.swift <source.png> <master.png>\n".data(using: .utf8)!)
    exit(1)
}
let canvas = 1024, shape = 824, radius = 185.0
let cs = CGColorSpace(name: CGColorSpace.sRGB)!
let ctx = CGContext(data: nil, width: canvas, height: canvas, bitsPerComponent: 8, bytesPerRow: 0, space: cs,
                    bitmapInfo: CGImageAlphaInfo.premultipliedLast.rawValue)!
ctx.interpolationQuality = .high
let rect = CGRect(x: (canvas - shape) / 2, y: (canvas - shape) / 2, width: shape, height: shape)
let path = CGPath(roundedRect: rect, cornerWidth: radius, cornerHeight: radius, transform: nil)

// Shadow: drawn from the shape itself, then the artwork goes on top of it.
ctx.saveGState()
ctx.setShadow(offset: CGSize(width: 0, height: -12), blur: 24, color: CGColor(gray: 0, alpha: 0.35))
ctx.addPath(path); ctx.setFillColor(CGColor(gray: 0, alpha: 1)); ctx.fillPath()
ctx.restoreGState()

ctx.saveGState()
ctx.addPath(path); ctx.clip()
ctx.draw(srcCG, in: rect)
ctx.restoreGState()

let out = NSBitmapImageRep(cgImage: ctx.makeImage()!)
try! out.representation(using: .png, properties: [:])!.write(to: URL(fileURLWithPath: args[2]))
