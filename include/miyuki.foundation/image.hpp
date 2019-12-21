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
#pragma once

#include <miyuki.foundation/math.hpp>

namespace miyuki {
    template<class T>
    class TImage {
        ivec2 dimension;
        std::vector<T> texels;

    public:
        TImage(const ivec2 &dim) : dimension(dim), texels(dim[0] * dim[1]) {}

        const T &operator()(int x, int y) const {
            x = std::clamp(x, 0, dimension[0] - 1);
            y = std::clamp(y, 0, dimension[1] - 1);
            return texels[x + y * dimension[0]];
        }

        T &operator()(int x, int y) {
            x = std::clamp(x, 0, dimension[0] - 1);
            y = std::clamp(y, 0, dimension[1] - 1);
            return texels[x + y * dimension[0]];
        }

        const T &operator()(float x, float y) const { return (*this)(vec2(x, y)); }

        T &operator()(float x, float y) { return (*this)(vec2(x, y)); }

        const T &operator()(const ivec2 &p) const { return (*this)(p.x, p.y); }

        T &operator()(const ivec2 &p) { return (*this)(p.x, p.y); }

        const T &operator()(const vec2 &p) const { return (*this)(ivec2(p * vec2(dimension))); }

        T &operator()(const vec2 &p) { return (*this)(ivec2(p * vec2(dimension))); }

        T *data() { return texels.data(); }

        const T *data() const { return texels.data(); }
    };

    class RGBImage : public TImage<vec3> {
    public:
        using TImage<vec3>::TImage;
    };

    class RGBAImage : public TImage<vec4> {
        using TImage<vec4>::TImage;
    };

} // namespace miyuki