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
#include <glm/gtx/transform.hpp>
#include <cmath>
#include <miyuki.foundation/defs.h>

namespace miyuki {
    using Float = float;
    using namespace glm;
    using Point3f = vec3;
    using Vec3f = vec3;
    using Normal3f = vec3;
    using Point2f = vec2;
    using Point2i = ivec2;
    using Point3i = ivec3;

    template<class T>
    struct Radians;

    template<class T>
    struct Degrees {
        Degrees() = default;
        explicit Degrees(T v) : value(v) {}

        explicit operator T() const {
            return value;
        }

        inline explicit Degrees(const Radians<T> &);

        T &get() { return value; }

        const T &get() const { return value; }

    private:
        T value = T();
    };

    template<class T>
    struct Radians {
        Radians()= default;
        explicit Radians(T v) : value(v) {}

        explicit Radians(const Degrees<T> &degrees) {
            value = DegreesToRadians(T(degrees));
        }

        explicit operator T() const {
            return value;
        }

        T &get() { return value; }

        [[nodiscard]] const T &get() const { return value; }

    private:
        T value = T();
    };

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
        Radians<vec3> rotation;
        vec3 translation;

        [[nodiscard]] Transform toTransform() const {
            mat4 m = identity<mat4>();
            m = rotate(rotation.get().z, Vec3f(0, 0, 1)) * m;
            m = rotate(rotation.get().y, Vec3f(1, 0, 0)) * m;
            m = rotate(rotation.get().x, Vec3f(0, 1, 0)) * m;
            m = glm::translate(translation) * m;
            return Transform(m);
        }
    };



    template<class T>
    inline Degrees<T>::Degrees(const miyuki::Radians<T> &r) {
        value = RadiansToDegrees(T(r));
    }

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

        [[nodiscard]] Vec3f worldToLocal(const Vec3f &v) const {
            return Vec3f(dot(localX, v), dot(normal, v), dot(localZ, v));
        }

        [[nodiscard]] Vec3f localToWorld(const Vec3f &v) const {
            return Vec3f(v.x * localX + v.y * normal + v.z * localZ);
        }

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
    using Bounds2i = BoundBox<2, int, qualifier ::defaultp>;
    using Bounds2f = BoundBox<2, float, qualifier ::defaultp>;

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

    // finds the smallest i such that pred(i) is false
    template <typename Predicate>
    int FindInterval(int begin, int end, const Predicate &pred) {
        int lo = begin;
        int hi = end - 1;
        while(lo <= hi){
            int m = (lo + hi) / 2;
            if(m - 1 < 0 || pred(m - 1 ) && !pred(m)){
                return m;
            }
            if(pred(m)){
                lo = m + 1;
            }else{
                hi = m - 1;
            }
        }
        return lo;
    }

    inline vec3 FaceForward(const vec3& v, const vec3 & n){
        return dot(v, n) < 0 ? -v : v;
    }
} // namespace miyuki

namespace cereal {
    template<typename Archive, int N, typename T, glm::qualifier Q>
    void serialize(Archive &ar, glm::vec<N, T, Q> &v) {
        for (int i = 0; i < N; i++) {
            ar(v[i]);
        }
    }

    template<typename Archive, int C, int R, typename T, glm::qualifier Q>
    void serialize(Archive &ar, glm::mat<C, R, T, Q> &v) {
        for (int i = 0; i < C; i++) {
            ar(v[i]);
        }
    }

    template<class Archive, class T>
    void serialize(Archive &ar, miyuki::Degrees<T> &v) {
        ar(v.get());
    }

    template<class Archive, class T>
    void serialize(Archive &ar, miyuki::Radians<T> &v) {
        ar(v.get());
    }

}
namespace cereal {
    template<class Archive>
    void save(Archive &ar, const miyuki::Transform &m) { ar(m.matrix()); }

    template<class Archive>
    void load(Archive &ar, miyuki::Transform &m) {
        glm::mat4 matrix4;
        ar(matrix4);
        m = miyuki::Transform(matrix4);
    }

    template<class Archive>
    void save(Archive &ar, const miyuki::TransformManipulator &m) { ar(m.translation, m.rotation); }

    template<class Archive>
    void load(Archive &ar, miyuki::TransformManipulator &m) {
        ar(m.translation, m.rotation);
    }
} // namespace cereal

#endif // MIYUKIRENDERER_MATH_HPP
