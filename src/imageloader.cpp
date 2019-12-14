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
#include <miyuki.foundation/imageloader.h>
#include <unordered_map>
#include <miyuki.foundation/log.hpp>
// #define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <miyuki.foundation/parallel.h>


namespace miyuki {
    class ImageLoader::Impl {
        struct ImageRecord {
            std::shared_ptr<RGBAImage> image;
            fs::file_time_type lastModifiedTime;
        };

    public:
        std::unordered_map<std::string, ImageRecord> cached;

        std::shared_ptr<RGBAImage> loadRGBAImage(const fs::path &path) {
            auto iter = cached.find(fs::absolute(path).string());
            auto last = fs::last_write_time(path);

            if (iter == cached.end() || last > iter->second.lastModifiedTime) {
                log::log("loading {}\n", path.string());
                auto extension = path.extension().string();
                if (extension == ".ppm") {
                    return nullptr;
                }
                int w, h;
                int comp;
                auto data = stbi_load(path.string().c_str(), &w, &h, &comp, 3);
                if (!data) {
                    log::log("failed to load {}\n", path.extension().string());
                    return nullptr;
                }
                auto invGamma = [](const vec3 &v, float gamma) {
                    return pow(v, vec3(1.0f / gamma));
                };
                auto image = std::make_shared<RGBAImage>(ivec2(w, h));
                if (comp == 4) {
                    ParallelFor(
                            0, w * h,
                            [=](int i, int threadIdx) {
                                image->data()[i] =
                                        vec4(invGamma(vec3(data[4 * i + 0], data[4 * i + 1], data[4 * i + 2]) / 255.0f,
                                                      1.0f / 2.2f), data[4 * i + 3] / 255.0f);
                            },
                            4096);
                } else if (comp == 3) {
                    ParallelFor(
                            0, w * h,
                            [=](int i, int threadIdx) {
                                image->data()[i] = vec4(
                                        invGamma(vec3(data[3 * i + 0], data[3 * i + 1], data[3 * i + 2])
                                                 / 255.0f, 1.0f / 2.2f), 1);
                            }, 4096);
                } else {
                    MIYUKI_NOT_IMPLEMENTED();
                }
                cached[fs::absolute(path).string()] = ImageRecord{image, last};
                log::log("loaded {}\n", path.string());
                stbi_image_free(data);
                return image;
            } else {
                return iter->second.image;
            }
        }
    };

    ImageLoader::ImageLoader() : impl(new Impl()) {}

    std::shared_ptr<RGBAImage> ImageLoader::loadRGBAImage(const fs::path &path) { return impl->loadRGBAImage(path); }

    static ImageLoader instance;

    ImageLoader *ImageLoader::getInstance() {
        return &instance;
    }

} // namespace miyuki