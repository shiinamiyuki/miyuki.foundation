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

#ifndef MIYUKIRENDERER_MATH_HPP
#define MIYUKIRENDERER_MATH_HPP

#include <cmath>
#include <nlohmann/json_fwd.hpp>
#include <xmmintrin.h>
#include <glm/glm.hpp>

namespace miyuki {
    using Float = float;
    using namespace glm;

    class Transform {
        mat4 T, invT;
    public:
        explicit Transform(const mat4 &T) : T(T), invT(inverse(T)) {}

        [[nodiscard]] const mat4 &matrix() const { return T; }
    };
} // namespace miyuki

namespace cereal {
    template<class Archive, size_t N, typename T, glm::qualifier Q>
    void save(Archive &ar, const glm::vec<N, T, Q> &v) {
        for (size_t i = 0; i < N; i++)
            ar(v[i]);
    }

    template<class Archive, size_t N, typename T, glm::qualifier Q>
    void load(const Archive &ar, glm::vec<N, T, Q> &v) {
        for (size_t i = 0; i < N; i++)
            ar(v[i]);
    }


    template<class Archive, class T, glm::qualifier Q>
    void save(Archive &ar, const glm::mat4 &m) { ar(m[0], m[1], m[2], m[3]); }

    template<class Archive>
    void load(Archive &ar, glm::mat4 &m) { ar(m[0], m[1], m[2], m[3]); }

    template<class Archive>
    void save(Archive &ar, const miyuki::Transform &m) { ar(m.matrix()); }

    template<class Archive>
    void load(Archive &ar, miyuki::Transform &m) {
        glm::mat4 matrix4;
        ar(matrix4);
        m = miyuki::Transform(matrix4);
    }
} // namespace cereal

#endif // MIYUKIRENDERER_MATH_HPP
