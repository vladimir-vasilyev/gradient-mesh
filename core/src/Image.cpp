#include "gmcore/Image.h"
#include <algorithm>
#include <cmath>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cstdint>

#ifdef GMCORE_USE_LIBPNG
#include <png.h>
#endif

namespace gmcore {

Image::Image(int w, int h) : width(w), height(h), data(size_t(w) * h * 3, 0.0) {}

static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

Color Image::at(int x, int y) const {
    x = clampi(x, 0, width - 1);
    y = clampi(y, 0, height - 1);
    size_t i = (size_t(y) * width + x) * 3;
    return {data[i], data[i + 1], data[i + 2]};
}

void Image::set(int x, int y, const Color& c) {
    if (x < 0 || y < 0 || x >= width || y >= height) return;
    size_t i = (size_t(y) * width + x) * 3;
    data[i] = c.r; data[i + 1] = c.g; data[i + 2] = c.b;
}

Color Image::sampleBilinear(double x, double y) const {
    // pixel centers at (i+0.5, j+0.5)
    double fx = x - 0.5, fy = y - 0.5;
    int x0 = (int)std::floor(fx), y0 = (int)std::floor(fy);
    double tx = fx - x0, ty = fy - y0;
    Color c00 = at(x0, y0), c10 = at(x0 + 1, y0);
    Color c01 = at(x0, y0 + 1), c11 = at(x0 + 1, y0 + 1);
    Color top = c00 * (1 - tx) + c10 * tx;
    Color bot = c01 * (1 - tx) + c11 * tx;
    return top * (1 - ty) + bot * ty;
}

ColorGrad Image::sampleGradient(double x, double y) const {
    // Analytic derivative of the bilinear interpolant above.
    double fx = x - 0.5, fy = y - 0.5;
    int x0 = (int)std::floor(fx), y0 = (int)std::floor(fy);
    double tx = fx - x0, ty = fy - y0;
    Color c00 = at(x0, y0), c10 = at(x0 + 1, y0);
    Color c01 = at(x0, y0 + 1), c11 = at(x0 + 1, y0 + 1);
    // d/dx: linear in ty between (c10-c00) and (c11-c01)
    Color dx = (c10 - c00) * (1 - ty) + (c11 - c01) * ty;
    Color dy = (c01 - c00) * (1 - tx) + (c11 - c10) * tx;
    return {dx, dy};
}

Image Image::downsampled2x() const {
    // Separable 5-tap Gaussian (1,4,6,4,1)/16 blur, then take every other sample.
    static const double k[5] = {1.0 / 16, 4.0 / 16, 6.0 / 16, 4.0 / 16, 1.0 / 16};
    Image blurredH(width, height);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            Color acc;
            for (int t = -2; t <= 2; ++t) acc += at(x + t, y) * k[t + 2];
            blurredH.set(x, y, acc);
        }
    }
    Image blurred(width, height);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            Color acc;
            for (int t = -2; t <= 2; ++t) acc += blurredH.at(x, clampi(y + t, 0, height - 1)) * k[t + 2];
            blurred.set(x, y, acc);
        }
    }
    int nw = std::max(1, width / 2), nh = std::max(1, height / 2);
    Image out(nw, nh);
    for (int y = 0; y < nh; ++y)
        for (int x = 0; x < nw; ++x)
            out.set(x, y, blurred.at(std::min(2 * x, width - 1), std::min(2 * y, height - 1)));
    return out;
}

std::vector<Image> Image::buildPyramid(const Image& full, int numLevels) {
    std::vector<Image> levels;
    levels.push_back(full);
    for (int i = 1; i < numLevels; ++i) {
        const Image& prev = levels.back();
        if (prev.width <= 8 || prev.height <= 8) break;
        levels.push_back(prev.downsampled2x());
    }
    return levels;
}

// ---------------- PPM (P6 binary) ----------------

Image Image::loadPPM(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Image::loadPPM: cannot open " + path);
    std::string magic;
    f >> magic;
    if (magic != "P6") throw std::runtime_error("Image::loadPPM: not a binary PPM: " + path);
    auto skipComments = [&]() {
        int c;
        while (true) {
            c = f.peek();
            if (c == '#') { std::string line; std::getline(f, line); }
            else if (std::isspace(c)) f.get();
            else break;
        }
    };
    int w, h, maxv;
    skipComments(); f >> w;
    skipComments(); f >> h;
    skipComments(); f >> maxv;
    f.get(); // single whitespace before binary data
    Image img(w, h);
    std::vector<unsigned char> raw(size_t(w) * h * 3);
    f.read(reinterpret_cast<char*>(raw.data()), raw.size());
    for (size_t i = 0; i < raw.size(); ++i) img.data[i] = raw[i] / (double)maxv;
    return img;
}

void Image::savePPM(const std::string& path) const {
    std::ofstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Image::savePPM: cannot open " + path);
    f << "P6\n" << width << " " << height << "\n255\n";
    std::vector<unsigned char> raw(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        double v = data[i] * 255.0;
        raw[i] = (unsigned char)clampi((int)std::lround(v), 0, 255);
    }
    f.write(reinterpret_cast<const char*>(raw.data()), raw.size());
}

#ifdef GMCORE_USE_LIBPNG
Image Image::loadPNG(const std::string& path) {
    FILE* fp = fopen(path.c_str(), "rb");
    if (!fp) throw std::runtime_error("Image::loadPNG: cannot open " + path);
    png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    png_infop info = png_create_info_struct(png);
    if (setjmp(png_jmpbuf(png))) { fclose(fp); throw std::runtime_error("Image::loadPNG: decode error"); }
    png_init_io(png, fp);
    png_read_info(png, info);

    png_uint_32 w = png_get_image_width(png, info);
    png_uint_32 h = png_get_image_height(png, info);
    png_byte colorType = png_get_color_type(png, info);
    png_byte bitDepth = png_get_bit_depth(png, info);

    if (bitDepth == 16) png_set_strip_16(png);
    if (colorType == PNG_COLOR_TYPE_PALETTE) png_set_palette_to_rgb(png);
    if (colorType == PNG_COLOR_TYPE_GRAY && bitDepth < 8) png_set_expand_gray_1_2_4_to_8(png);
    if (png_get_valid(png, info, PNG_INFO_tRNS)) png_set_tRNS_to_alpha(png);
    if (colorType == PNG_COLOR_TYPE_RGB || colorType == PNG_COLOR_TYPE_GRAY ||
        colorType == PNG_COLOR_TYPE_PALETTE)
        png_set_filler(png, 0xFF, PNG_FILLER_AFTER);
    if (colorType == PNG_COLOR_TYPE_GRAY || colorType == PNG_COLOR_TYPE_GRAY_ALPHA)
        png_set_gray_to_rgb(png);
    png_read_update_info(png, info);

    std::vector<png_bytep> rows(h);
    std::vector<unsigned char> buf(size_t(w) * h * 4);
    for (png_uint_32 y = 0; y < h; ++y) rows[y] = buf.data() + size_t(y) * w * 4;
    png_read_image(png, rows.data());

    Image img((int)w, (int)h);
    for (png_uint_32 y = 0; y < h; ++y)
        for (png_uint_32 x = 0; x < w; ++x) {
            unsigned char* p = &buf[(size_t(y) * w + x) * 4];
            img.set((int)x, (int)y, Color(p[0] / 255.0, p[1] / 255.0, p[2] / 255.0));
        }
    png_destroy_read_struct(&png, &info, nullptr);
    fclose(fp);
    return img;
}

void Image::savePNG(const std::string& path) const {
    FILE* fp = fopen(path.c_str(), "wb");
    if (!fp) throw std::runtime_error("Image::savePNG: cannot open " + path);
    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    png_infop info = png_create_info_struct(png);
    if (setjmp(png_jmpbuf(png))) { fclose(fp); throw std::runtime_error("Image::savePNG: encode error"); }
    png_init_io(png, fp);
    png_set_IHDR(png, info, width, height, 8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);

    std::vector<unsigned char> row(size_t(width) * 3);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            Color c = at(x, y).clamped01();
            row[x * 3 + 0] = (unsigned char)std::lround(c.r * 255.0);
            row[x * 3 + 1] = (unsigned char)std::lround(c.g * 255.0);
            row[x * 3 + 2] = (unsigned char)std::lround(c.b * 255.0);
        }
        png_write_row(png, row.data());
    }
    png_write_end(png, nullptr);
    png_destroy_write_struct(&png, &info);
    fclose(fp);
}
#endif

static bool endsWith(const std::string& s, const std::string& suf) {
    return s.size() >= suf.size() && s.compare(s.size() - suf.size(), suf.size(), suf) == 0;
}

Image Image::load(const std::string& path) {
#ifdef GMCORE_USE_LIBPNG
    if (endsWith(path, ".png") || endsWith(path, ".PNG")) return loadPNG(path);
#endif
    return loadPPM(path);
}

void Image::save(const std::string& path) const {
#ifdef GMCORE_USE_LIBPNG
    if (endsWith(path, ".png") || endsWith(path, ".PNG")) { savePNG(path); return; }
#endif
    savePPM(path);
}

} // namespace gmcore
