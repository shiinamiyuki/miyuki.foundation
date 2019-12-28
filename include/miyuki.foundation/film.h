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
#include <miyuki.foundation/image.hpp>

namespace miyuki::core {

    struct Film {
        struct Sample {
            vec3 color, normal, albedo;
        };

        struct Pixel {
            vec4 &color;
            vec4 &normal;
            vec4 &albedo;
            float &weight;

            Pixel(vec4 &color, vec4 &normal, vec4 &albedo, float &weight) : color(color), normal(normal),
                                                                            albedo(albedo), weight(weight) {}
        };

        RGBAImage color, normal, weight, albedo;
        const size_t width, height;

        explicit Film(const ivec2 &dim) : color(dim), weight(dim), normal(dim), albedo(dim), width(dim[0]),
                                          height(dim[1]) {}

        Film(size_t w, size_t h) : Film(ivec2(w, h)) {}

        static float gamma(float x, float k = 1.0f / 2.2f) { return std::pow(clamp(x, 0.0f, 1.0f), k); }

        static int toInt(float x) {
            return std::max<uint32_t>(0, std::min<uint32_t>(255, std::lroundf(gamma(x) * 255)));
        }

        void writePPM(const std::string &filename) {
            auto f = fopen(filename.c_str(), "w");
            fprintf(f, "P3\n%zu %zu\n%d\n", width, height, 255);
            for (int i = 0; i < width * height; i++) {
                auto invWeight = weight.data()[i].r == 0 ? 0.0f : 1.0f / weight.data()[i].r;
                fprintf(f, "%d %d %d ", toInt(color.data()[i][0] * invWeight), toInt(color.data()[i][1] * invWeight),
                        toInt(color.data()[i][2] * invWeight));
            }
        }

        void writeImage(const std::string &filename);

        Pixel operator()(const vec2 &p) { return (*this)(p.x, p.y); }

        Pixel operator()(float x, float y) {
            return Pixel(color(x, y), normal(x, y), albedo(x, y), weight(x, y).r);
        }

        Pixel operator()(int px, int py) {
            return Pixel(color(px, py), normal(px, py), albedo(px, py), weight(px, py).r);
        }

        void addSample(const ivec2 &p, const vec3 &color, Float weight) {
            auto pixel = (*this)(p.x,p.y);
            pixel.color += vec4(color * weight, 0);
            pixel.weight += weight;
        }

        void addSample(const vec2 &p, const Sample &sample, Float weight) {
            auto pixel = (*this)(p);
            pixel.color += vec4(sample.color * weight, 0);
            pixel.normal += vec4(sample.normal * weight, 0);
            pixel.albedo += vec4(sample.albedo * weight, 0);
            pixel.weight += weight;
        }
    };
} // namespace miyuki::core
#endif // MIYUKIRENDERER_FILM_H
