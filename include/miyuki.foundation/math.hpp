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
#include <miyuki.serialize/serialize.hpp>
#include <miyuki.foundation/defs.h>
#include <xmmintrin.h>
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <cmath>


namespace miyuki {

    using json = serialize::json::json;
    using Float = float;
    using namespace glm;
    using Point3f = vec3;
    using Vec3f = vec3;
    using Normal3f = vec3;
    using Point2f = vec2;
    using Point2i = ivec2;
    using Point3i = ivec3;
}
namespace glm {
    using json = nlohmann::json;

    template<int N, typename T, qualifier Q>
    inline void to_json(json &j, const vec<N, T, Q> &v) {
        j = json::array();
        for (int i = 0; i < N; i++) {
            j[i] = v[i];
        }
    }

    template<int N, typename T, qualifier Q>
    inline void from_json(const json &j, vec<N, T, Q> &v) {
        for (int i = 0; i < N; i++) {
            v[i] = j[i];
        }
    }

    inline void to_json(json &j, const mat4 &m) {
        j = json::array();
        for (int i = 0; i < 4; i++) {
            j[i] = m[i];
        }
    }

    inline void from_json(const json &j, mat4 &m) {
        for (int i = 0; i < 4; i++) {
            m[i] = j[i].get<vec4>();
        }
    }
}
namespace miyuki {

    class Transform {
        mat4 T, invT;
        mat3 T3, invT3, invT3T;
    public:
        Transform() = default;

        explicit Transform(const mat4 &T) : T(T), invT(glm::inverse(T)) {
            T3 = mat3(T);
            invT3 = mat3(invT);
            invT3T = transpose(invT3);
        }

        Transform(const mat4 &T, const mat4 &invT) : T(T), invT(invT) {
            T3 = mat3(T);
            invT3 = mat3(invT);
            invT3T = transpose(invT3);
        }

        [[nodiscard]] const mat4 &matrix() const { return T; }

        [[nodiscard]] Transform inverse() const {
            return Transform{invT, T};
        }

        [[nodiscard]] vec3 transformVec3(const vec3 &v) const {
            return T3 * v;
        }

        [[nodiscard]] vec3 transformPoint3(const vec3 &v) const {
            auto x = T * vec4(v, 1);
            if (x.w == 1) {
                return x;
            }
            return vec3(x) / x.w;
        }

        [[nodiscard]] vec3 transformNormal3(const vec3 &v) const {
            return invT3T * v;
        }
    };

    class TransformManipulator {
    public:
        vec3 rotation;
        vec3 translation;

        [[nodiscard]] Transform toTransform() const {
            mat4 m = identity<mat4>();
            m = rotate(rotation.z, Vec3f(0, 0, 1)) * m;
            m = rotate(rotation.y, Vec3f(1, 0, 0)) * m;
            m = rotate(rotation.x, Vec3f(0, 1, 0)) * m;
            m = glm::translate(translation) * m;
            return Transform(m);
        }
    };

    inline void ComputeLocalFrame(const Vec3f &v1, Vec3f *v2, Vec3f *v3) {
        if (std::abs(v1.x) > std::abs(v1.y))
            *v2 = Vec3f(-v1.z, 0, v1.x) / std::sqrt(v1.x * v1.x + v1.z * v1.z);
        else
            *v2 = Vec3f(0, v1.z, -v1.y) / std::sqrt(v1.y * v1.y + v1.z * v1.z);
        *v3 = normalize(cross(v1, *v2));
    }

    struct CoordinateSystem {
        CoordinateSystem() = default;

        explicit CoordinateSystem(const Vec3f &v) : normal(v) { ComputeLocalFrame(v, &localX, &localZ); }

        Vec3f worldToLocal(const Vec3f &v) const { return Vec3f(dot(localX, v), dot(normal, v), dot(localZ, v)); }

        Vec3f localToWorld(const Vec3f &v) const { return Vec3f(v.x * localX + v.y * normal + v.z * localZ); }

    private:
        Vec3f normal;
        Vec3f localX, localZ;
    };

    template<int N, class T, qualifier Q>
    class BoundBox {
    public:
        vec<N, T, Q> pMin, pMax;

        BoundBox unionOf(const BoundBox &box) const { return BoundBox{min(pMin, box.pMin), max(pMax, box.pMax)}; }

        BoundBox unionOf(const vec<N, T, Q> &rhs) const { return BoundBox{min(pMin, rhs), max(pMax, rhs)}; }

        vec<N, T, Q> centroid() const { return (pMin + pMax) * 0.5f; }

        vec<N, T, Q> size() const { return pMax - pMin; }

        T surfaceArea() const {
            auto a = (size()[0] * size()[1] + size()[0] * size()[2] + size()[1] * size()[2]);
            return a + a;
        }

        bool intersects(const BoundBox &rhs) const {
            for (size_t i = 0; i < N; i++) {
                if (pMin[i] > rhs.pMax[i] || pMax[i] < rhs.pMin[i]);
                else {
                    return true;
                }
            }
            return false;
        }

        vec<N, T, Q> offset(const vec<N, T, Q> &p) const {
            auto o = p - pMin;
            return o / size();
        }
    };

    using Bounds3f = BoundBox<3, float, qualifier::defaultp>;

    template<class T>
    T lerp3(const T &v1, const T &v2, const T &v3, float u, float v) {
        return (1 - u - v) * v1 + u * v1 + v * v2;
    }

    template<int N, typename T, qualifier Q>
    T minComp(const vec<N, T, Q> &v) {
        auto x = v[0];
        for (int i = 1; i < N; i++) {
            x = min(x, v[i]);
        }
        return x;
    }

    template<int N, typename T, qualifier Q>
    T maxComp(const vec<N, T, Q> &v) {
        auto x = v[0];
        for (int i = 1; i < N; i++) {
            x = max(x, v[i]);
        }
        return x;
    }

    template<class T>
    T lerp(const T &x, const T &y, const T &a) {
        return x * T(1.0f - a) + y * a;
    }

    template<class T>
    struct Angle {
        Angle() = default;

        Angle(const T &v) : data(v) {}

        operator T() const { return data; }

        auto &get() const { return data; }

    private:
        T data;
    };

    inline vec3 FaceForward(const vec3 &v, const vec3 &n) {
        return dot(v, n) < 0 ? -v : v;
    }

    inline void to_json(json &j, const Transform &transform) { j = transform.matrix(); }

    inline void from_json(const json &j, Transform &transform) { transform = Transform(j.get<mat4>()); }

    template<class T>
    void to_json(json &j, const Angle<T> &v) {
        j = v.get();
    }


    template<class T>
    void from_json(const json &j, Angle<T> &v) {
        if (j.contains("deg")) {
            v = DegreesToRadians(j.get<T>());
        } else if (j.contains("rad")) {
            v = j.get<T>();
        } else {
            v = j.get<T>();
        }
    }


    inline void to_json(json &j, const TransformManipulator &transform) {
        j = json::object();
        j["rotation"] = transform.rotation;
        j["translation"] = transform.translation;
    }

    inline void from_json(const json &j, TransformManipulator &transform) {
        transform.rotation = j.at("rotation").get<Angle<vec3 >>();
        transform.translation = j.at("translation").get<vec3>();
    }
} // namespace miyuki




#endif // MIYUKIRENDERER_MATH_HPP
