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
#include <miyuki.math/math.hpp>
#include <cmath>


namespace miyuki {
    using namespace math;
    using json = serialize::json::json;
    using Float = float;
    using Point3f = math::Array<float, 3>;
    using Vec3f = math::Array<float, 3>;
    using Vec2i = math::Array<int, 2>;
    using Vec2f = math::Array<float, 2>;
    using Normal3f =  math::Array<float, 3>;
    using Point2f = math::Array<float, 2>;
    using Point2i = math::Array<int, 2>;
    using Point3i =  math::Array<int, 3>;
    using Matrix4f = math::Matrix4<float>;
    using Matrix3f = math::Matrix3<float>;

    using json = nlohmann::json;
    namespace math {
        template<class T, int N>
        inline void to_json(json &j, const math::Array<T, N> &v) {
            j = json::array();
            for (int i = 0; i < N; i++) {
                j[i] = v[i];
            }
        }

        template<class T, int N>
        inline void from_json(const json &j, math::Array<T, N> &v) {
            for (int i = 0; i < N; i++) {
                v[i] = j[i];
            }
        }

        template<class T>
        inline void to_json(json &j, const math::Matrix4<T> &m) {
            j = json::array();
            for (int i = 0; i < 4; i++) {
                j[i] = m[i];
            }
        }

        template<class T>
        inline void from_json(const json &j, math::Matrix4<T> &m) {
            for (int i = 0; i < 4; i++) {
                m[i] = j[i].get<Array<T, 4>>();
            }
        }
    }
}
namespace miyuki {
    template<class T>
    struct Angle {
        Angle() = default;

        Angle(const T &v) : data(v) {}

        operator T() const { return data; }

        auto &get() const { return data; }

    private:
        T data;
    };

    template<class Value>
    class Transform {
        using mat4 = Matrix4<Value>;
        using mat3 = Matrix3<Value>;
        mat4 T, invT;
        mat3 T3, invT3, invT3T;
    public:
        Transform() = default;

        explicit Transform(const mat4 &T) : T(T), invT(math::inverse(T)) {
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

        [[nodiscard]] Vec3f transformVec3(const Vec3f &v) const {
            return T3 * v;
        }

        [[nodiscard]] Point3f transformPoint3(const Point3f &v) const {
            auto x = T * Array<float, 4>{v.x(), v.y(), v.z(), 1.0f};
            if (x.w == 1) {
                return x;
            }
            return Vec3f(x) / x.w;
        }

        [[nodiscard]] Normal3f transformNormal3(const Normal3f &v) const {
            return invT3T * v;
        }
    };

    class TransformManipulator {
    public:
        Angle<Vec3f> rotation;
        Vec3f translation;

        [[nodiscard]] Transform<float> toTransform() const {
            Matrix4f m = Matrix4f::identity();
            m = Matrix4f::rotate(rotation.get().z(), Vec3f(0, 0, 1)) * m;
            m = Matrix4f::rotate(rotation.get().y(), Vec3f(1, 0, 0)) * m;
            m = Matrix4f::rotate(rotation.get().x(), Vec3f(0, 1, 0)) * m;
            m = Matrix4f::translate(translation) * m;
            return Transform<float>(m);
        }
    };

    inline void ComputeLocalFrame(const Vec3f &v1, Vec3f *v2, Vec3f *v3) {
        if (std::abs(v1.x()) > std::abs(v1.y()))
            *v2 = Vec3f(-v1.z(), 0, v1.x()) / Vec3f(std::sqrt(v1.x() * v1.x() + v1.z() * v1.z()));
        else
            *v2 = Vec3f(0, v1.z(), -v1.y()) / Vec3f(std::sqrt(v1.y() * v1.y() + v1.z() * v1.z()));
        *v3 = normalize(cross(v1, *v2));
    }

    struct CoordinateSystem {
        CoordinateSystem() = default;

        explicit CoordinateSystem(const Vec3f &v) : normal(v) { ComputeLocalFrame(v, &localX, &localZ); }

        [[nodiscard]] Vec3f worldToLocal(const Vec3f &v) const {
            return Vec3f(dot(localX, v), dot(normal, v), dot(localZ, v));
        }

        [[nodiscard]] Vec3f localToWorld(const Vec3f &v) const {
            return Vec3f(v.x() * localX + v.y() * normal + v.z() * localZ);
        }

    private:
        Vec3f normal;
        Vec3f localX, localZ;
    };

    template<class T, int N>
    class BoundBox {
    public:
        Array<T, N> pMin, pMax;

        BoundBox unionOf(const BoundBox &box) const { return BoundBox{min(pMin, box.pMin), max(pMax, box.pMax)}; }

        BoundBox unionOf(const Array<T, N> &rhs) const { return BoundBox{min(pMin, rhs), max(pMax, rhs)}; }

        Array<T, N> centroid() const { return (pMin + pMax) * 0.5f; }

        Array<T, N> size() const { return pMax - pMin; }

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

        Array<T, N> offset(const Array<T, N> &p) const {
            auto o = p - pMin;
            return o / size();
        }
    };

    using Bounds3f = BoundBox<float, 3>;

    template<class T>
    T lerp3(const T &v1, const T &v2, const T &v3, float u, float v) {
        return (1 - u - v) * v1 + u * v2 + v * v3;
    }

    template<class T, int N>
    T minComp(const Array<T, N> &v) {
        auto x = v[0];
        for (int i = 1; i < N; i++) {
            x = min(x, v[i]);
        }
        return x;
    }

    template<class T, int N>
    T maxComp(const Array<T, N> &v) {
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


    inline Vec3f FaceForward(const Vec3f &v, const Vec3f &n) {
        return dot(v, n) < 0.0f ? Vec3f(-1.0f * v) : v;
    }

    template<class T>
    inline void to_json(json &j, const Transform<T> &transform) { j = transform.matrix(); }

    template<class T>
    inline void from_json(const json &j, Transform<T> &transform) { transform = Transform(j.get<Matrix4<T>>()); }

    template<class T>
    void to_json(json &j, const Angle<T> &v) {
        j = {
                {"rad", v.get()}
        };
    }


    template<class T>
    void from_json(const json &j, Angle<T> &v) {
        if (j.contains("deg")) {
            v = DegreesToRadians(j.at("deg").get<T>());
        } else if (j.contains("rad")) {
            v = j.at("rad").get<T>();
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
        transform.rotation = j.at("rotation").get<Angle<Vec3f>>();
        transform.translation = j.at("translation").get<Vec3f>();
    }
} // namespace miyuki




#endif // MIYUKIRENDERER_MATH_HPP
