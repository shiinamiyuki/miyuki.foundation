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

#ifndef MIYUKIRENDERER_SPECTRUM_H
#define MIYUKIRENDERER_SPECTRUM_H

#include <miyuki.foundation/defs.h>
#include <miyuki.foundation/math.hpp>

namespace miyuki::core {

    class RGBSpectrum : public vec3 {
    public:
        using vec3::vec3;

        RGBSpectrum(const vec3 &v) : vec3(v) {}
    };

    using Spectrum = RGBSpectrum;

    inline bool IsBlack(const Spectrum &s) {
        return s.x <= 0 || s.y <= 0 || s.z <= 0;
    }

    inline Spectrum RemoveNaN(const Spectrum & s){
        auto removeNan = [=](float x){return std::isnan(x) ? 0 : x;};
        return Spectrum(removeNan(s[0]),
                        removeNan(s[1]),
                        removeNan(s[2]));
    }

}
#endif //MIYUKIRENDERER_SPECTRUM_H
