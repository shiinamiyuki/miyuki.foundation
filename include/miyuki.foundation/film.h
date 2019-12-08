// MIT License
//
// Copyright (c) 2019 椎名深雪
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef MIYUKIRENDERER_FILM_H
#define MIYUKIRENDERER_FILM_H

#include <miyuki.foundation/math.hpp>
#include <algorithm>

namespace miyuki::core {

    struct Film {
        struct Pixels {
            std::vector<vec4> color;
            std::vector<vec4> weight;

            explicit Pixels(int s) : color(s), weight(s) {}
        };

        struct Pixel {
            vec4 &color;
            float &weight;

            Pixel(vec4 &color, float &weight) : color(color), weight(weight) {}
        };

        Pixels pixels;
        const size_t width, height;

        explicit Film(const ivec2 &dim) : Film(dim.x, dim.y) {}

        Film(size_t w, size_t h) : width(w), height(h), pixels(w * h) {}

        static float gamma(float x, float k = 1.0f / 2.2f) { return std::pow(clamp(x, 0.0f, 1.0f), k); }

        static int toInt(float x) {
            return std::max<uint32_t>(0, std::min<uint32_t>(255, std::lroundf(gamma(x) * 255)));
        }

        void writePPM(const std::string &filename) {
            auto f = fopen(filename.c_str(), "w");
            fprintf(f, "P3\n%d %d\n%d\n", width, height, 255);
            for (int i = 0; i < width * height; i++) {
                auto invWeight = pixels.weight[i].r == 0 ? 0.0f : 1.0f / pixels.weight[i].r;
                fprintf(f, "%d %d %d ", toInt(pixels.color[i][0] * invWeight), toInt(pixels.color[i][1] * invWeight),
                        toInt(pixels.color[i][2] * invWeight));
            }
        }

        void writeImage(const std::string &filename);

        Pixel operator()(const vec2 &p) { return (*this)(p.x, p.y); }

        Pixel operator()(float x, float y) {
            int px = clamp<int>(std::lround(x * width), 0, width - 1);
            int py = clamp<int>(std::lround(y * height), 0, height - 1);
            return Pixel(pixels.color.at(px + py * width), pixels.weight.at(px + py * width).r);
        }

        Pixel operator()(int px, int py) { return Pixel(pixels.color.at(px + py * width), pixels.weight.at(px + py * width).r); }

        void addSample(const vec2 &p, const vec3 &color, Float weight) {
            auto pixel = (*this)(p);
            pixel.color += vec4(color * weight, 0);
            pixel.weight += weight;
        }
    };
} // namespace miyuki::core
#endif // MIYUKIRENDERER_FILM_H
