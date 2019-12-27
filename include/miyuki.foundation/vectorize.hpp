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

#ifndef MIYUKI_VECTORIZE_VECTORIZE_HPP
#define MIYUKI_VECTORIZE_VECTORIZE_HPP

#include <array>
#include <type_traits>
#include <functional>
#include <xmmintrin.h>

namespace miyuki::vectorize {
    template<class T, int N>
    class Array : public std::array<T, N> {
    public:
    };


    constexpr int align4(int N) {
        return (3 + N) & (-4);
    }


    namespace detail {
        template<class T, int N>
        struct to_array {
            using type = Array<T, N>;

        };
    }


    template<class F, int N, class... Args>
    auto
    apply(F &&f,
          const Array<Args, N> &... args) -> typename detail::to_array<std::invoke_result_t<F, Args...>, N>::type {
        typename detail::to_array<std::invoke_result_t<F, Args...>, N>::type ret;
        for (auto i = 0; i < N; i++) {
            ret[i] = f(args[i]...);
        }
        return ret;
    }

#define MYK_VEC_ARR_GEN_OP_(op) friend self_type operator op (const self_type & lhs,const self_type & rhs ) {\
                                        return apply([](const value_type &a, const value_type &b)->value_type{\
                                            return value_type(a op b);\
                                        }, lhs, rhs); \
                                }
#define MYK_VEC_ARR_GEN_OP(op) MYK_VEC_ARR_GEN_OP_(op)
#define MYK_VEC_ARR_GEN_ASSIGN_OP(op) self_type& operator op##= (const self_type & rhs){*this = *this op rhs;return *this;}

#define MYK_VEC_GEN_BASIC_ASSIGN_OPS()  \
    MYK_VEC_ARR_GEN_ASSIGN_OP(+) \
    MYK_VEC_ARR_GEN_ASSIGN_OP(-)\
    MYK_VEC_ARR_GEN_ASSIGN_OP(*)\
    MYK_VEC_ARR_GEN_ASSIGN_OP(/)

#define MYK_VEC_GEN_BASIC_OPS() \
    MYK_VEC_ARR_GEN_OP_(+) \
    MYK_VEC_ARR_GEN_OP_(-)\
    MYK_VEC_ARR_GEN_OP_(*)\
    MYK_VEC_ARR_GEN_OP_(/)\
    MYK_VEC_ARR_GEN_OP_(==)\
    MYK_VEC_ARR_GEN_OP_(!=)\
    MYK_VEC_ARR_GEN_OP_(<)\
    MYK_VEC_ARR_GEN_OP_(<=)\
    MYK_VEC_ARR_GEN_OP_(>)\
    MYK_VEC_ARR_GEN_OP_(>=)

    template<int N>
    class Array<float, N> : public std::array<float, N> {
        using value_type = float;
        using self_type = Array;
    public:
        MYK_VEC_GEN_BASIC_OPS()

        MYK_VEC_GEN_BASIC_ASSIGN_OPS()
    };

    class Float4Base {
        using self_type = Float4Base;
        using value_type = float;
    public:
        union {
            struct {
                float x, y, z, w;
            };
            struct {
                float r, g, b, a;
            };
            struct {
                float s0, s1, s2, s3;
            };
            __m128 m;
        };

        float &operator[](int i) {
            return ((float *) &m)[i];
        }

        const float &operator[](int i) const {
            return ((const float *) &m)[i];
        }

        Float4Base() = default;

        Float4Base(std::initializer_list<float> list) {
            auto it = list.begin();
            for (int i = 0; i < 4; i++) {
                (*this)[i] = *it;
                it++;
            }
        }

        explicit Float4Base(__m128 m) {
            auto n = m;
            memcpy(this, &n, sizeof(__m128));
        }

        explicit operator __m128() const {
            __m128 m;
            memcpy(&m, this, sizeof(__m128));
            return m;
        };

        friend Float4Base operator+(const Float4Base &lhs, const Float4Base &rhs) {
            return Float4Base(_mm_add_ps((__m128) lhs, (__m128) rhs));
        }

        friend Float4Base operator-(const Float4Base &lhs, const Float4Base &rhs) {
            return Float4Base(_mm_sub_ps((__m128) lhs, (__m128) rhs));
        }

        friend Float4Base operator*(const Float4Base &lhs, const Float4Base &rhs) {
            return Float4Base(_mm_mul_ps((__m128) lhs, (__m128) rhs));
        }

        friend Float4Base operator/(const Float4Base &lhs, const Float4Base &rhs) {
            return Float4Base(_mm_div_ps((__m128) lhs, (__m128) rhs));
        }

        friend Float4Base operator<(const Float4Base &lhs, const Float4Base &rhs) {
            return Float4Base(_mm_cmplt_ps((__m128) lhs, (__m128) rhs));
        }

        friend Float4Base operator<=(const Float4Base &lhs, const Float4Base &rhs) {
            return Float4Base(_mm_cmple_ps((__m128) lhs, (__m128) rhs));
        }

        friend Float4Base operator>(const Float4Base &lhs, const Float4Base &rhs) {
            return Float4Base(_mm_cmpgt_ps((__m128) lhs, (__m128) rhs));
        }

        friend Float4Base operator>=(const Float4Base &lhs, const Float4Base &rhs) {
            return Float4Base(_mm_cmpge_ps((__m128) lhs, (__m128) rhs));
        }


        MYK_VEC_GEN_BASIC_ASSIGN_OPS()
    };

    static_assert(sizeof(__m128) == sizeof(Float4Base));

    template<>
    class Array<float, 4> : public Float4Base {
    public:
        using Float4Base::Float4Base;
    };

    template<>
    class Array<float, 3> : public Float4Base {
    public:
        using Float4Base::Float4Base;
    };

    template<class T, size_t N>
    T dot(const Array<T, N> &a, const Array<T, N> &b) {
        auto x = a[0] * b[0];
        for (auto i = 1; i < N; i++) {
            x += a[i] * b[i];
        }
        return x;
    }

    template<class T>
    T cross(const Array<T, 3> &v1, const Array<T, 3> &v2) {
        return {v1[1] * v2[2] - v1[2] * v2[1], v1[2] * v2[0] - v1[0] * v2[2], v1[0] * v2[1] - v1[1] * v2[0]};
    }

}

namespace miyuki {
#define MYK_VEC_DECL_PRIMITIVE(TY, N) using TY##N = vectorize::Array<TY, N>;
    MYK_VEC_DECL_PRIMITIVE(bool, 2)
    MYK_VEC_DECL_PRIMITIVE(bool, 3)
    MYK_VEC_DECL_PRIMITIVE(bool, 4)
    MYK_VEC_DECL_PRIMITIVE(bool, 8)
    MYK_VEC_DECL_PRIMITIVE(bool, 16)

    MYK_VEC_DECL_PRIMITIVE(float, 2)
    MYK_VEC_DECL_PRIMITIVE(float, 3)
    MYK_VEC_DECL_PRIMITIVE(float, 4)
    MYK_VEC_DECL_PRIMITIVE(float, 8)
    MYK_VEC_DECL_PRIMITIVE(float, 16)

    MYK_VEC_DECL_PRIMITIVE(int, 2)
    MYK_VEC_DECL_PRIMITIVE(int, 3)
    MYK_VEC_DECL_PRIMITIVE(int, 4)
    MYK_VEC_DECL_PRIMITIVE(int, 8)
    MYK_VEC_DECL_PRIMITIVE(int, 16)

}


#define MYK_VEC_STRUCT_BEGIN(Struct) template<int NElem> struct T##Struct { using Base = Struct;
#define MYK_VEC_MEMBER(Name) typename miyuki::vectorize::detail::to_array<decltype(Base::Name),NElem>::type Name;
#define MYK_VEC_METHOD(Ret, Name, ...) Ret Name()
#define MYK_VEC_STRUCT_END };

#endif //MIYUKI_VECTORIZE_VECTORIZE_HPP
