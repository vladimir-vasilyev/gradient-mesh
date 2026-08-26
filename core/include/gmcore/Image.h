// Image.h — a plain double-precision RGB raster used both as the
// optimization target and as the mesh's rendered reconstruction.
//
// File I/O is deliberately dependency-free: a built-in binary PPM (P6)
// reader/writer always works. If GMCORE_USE_LIBPNG is defined at compile
// time, PNG read/write is also available (used by the CLI test tool on
// platforms where libpng is present). The macOS app never needs this --
// it loads/saves images via AppKit's native ImageIO instead and feeds
// raw pixels into Image directly (see mac/DocumentModel.mm).
#pragma once
#include "gmcore/Color.h"
#include <string>
#include <vector>

namespace gmcore {

struct ColorGrad {
    Color dx, dy; // d(color)/dx, d(color)/dy in pixel units
};

class Image {
public:
    int width = 0, height = 0;
    std::vector<double> data; // size width*height*3, row-major, RGB interleaved

    Image() = default;
    Image(int w, int h);

    bool valid() const { return width > 0 && height > 0; }

    Color at(int x, int y) const;
    void set(int x, int y, const Color& c);

    // Bilinear sample with clamp-to-edge addressing. (x,y) in pixel-center
    // coordinates, i.e. pixel (0,0)'s center is at (0.5, 0.5).
    Color sampleBilinear(double x, double y) const;

    // Analytic gradient of the bilinear-sampled field at (x,y).
    ColorGrad sampleGradient(double x, double y) const;

    // 5-tap Gaussian blur followed by 2x decimation (one Gaussian-pyramid level).
    Image downsampled2x() const;

    // Builds a Gaussian pyramid, level 0 = original (finest) ... level N-1 = coarsest.
    static std::vector<Image> buildPyramid(const Image& full, int numLevels);

    // Dependency-free formats.
    static Image loadPPM(const std::string& path);
    void savePPM(const std::string& path) const;

#ifdef GMCORE_USE_LIBPNG
    static Image loadPNG(const std::string& path);
    void savePNG(const std::string& path) const;
#endif

    // Loads by extension, trying PPM always and PNG when compiled in.
    static Image load(const std::string& path);
    void save(const std::string& path) const;
};

} // namespace gmcore
