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

#ifndef MIYUKIRENDERER_ARRAY_HPP
#define MIYUKIRENDERER_ARRAY_HPP

#include <cstdint>

namespace miyuki {
    template<class T>
    class Array {
        T *data = nullptr;
    public:
        const size_t length;

        Array(T *data, size_t N) : data(data), length(N) {}

        T &operator[](size_t i) {
            return data[i];
        }

        const T &operator[](size_t i) const {
            return data[i];
        }

        size_t size() const { return length; }

        T *begin() { return data; }

        const T *begin() const { return data; }

        T *end() { return data + size; }

        const T *end() const { return data + size; }

    };

}

#endif //MIYUKIRENDERER_ARRAY_HPP
