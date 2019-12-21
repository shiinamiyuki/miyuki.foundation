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

#ifndef MIYUKIRENDERER_ARENA_HPP
#define MIYUKIRENDERER_ARENA_HPP

#include <algorithm>
#include <list>
#include <cstdint>

namespace miyuki {
    template<class T>
    class Arena {
        static constexpr size_t align16(size_t x) {
            return (x + 15ULL) & (~15ULL);
        }

        struct Block {
            size_t size;
            size_t curPos;
            uint8_t *data;

            Block(size_t size) : data(new uint8_t[size]), size(size), curPos(0) {}

            [[nodiscard]] size_t avaliable() const {
                return size - curPos;
            }

            ~Block() = default;
        };

        const size_t blockSize = align16(sizeof(T)) * 8;
        std::list<Block> availableBlocks, usedBlocks;


    public:
        Arena() {
            availableBlocks.emplace_back(Block(blockSize));
        }

        template<class... Args>
        T *allocN(size_t count, Args &&... args) {
            typename std::list<Block>::iterator iter;
            for (iter = availableBlocks.begin(); iter != availableBlocks.end(); iter++) {
                if (iter->avaliable() >= sizeof(T) * count) {
                    break;
                }
            }
            if (iter == availableBlocks.end()) {
                availableBlocks.emplace_front(Block(std::max(sizeof(T) * count, blockSize)));
                iter = availableBlocks.begin();
            }
            uint8_t *p = iter->data + iter->curPos;
            iter->curPos += sizeof(T) * count;
            if (iter->avaliable() == 0) {
                usedBlocks.splice(usedBlocks.begin(), availableBlocks, iter);
            }
            for (int i = 0; i < count; i++) {
                new(p + i * sizeof(T)) T(std::forward<Args>(args)...);
            }
            return reinterpret_cast<T*>(p);
        }

        template<class... Args>
        T *alloc(Args &&...args) {
            return allocN(1, std::forward<Args>(args)...);
        }

        void reset() {
            for (auto &i:usedBlocks) {
                i.curPos = 0;
            }
            availableBlocks.splice(availableBlocks.begin(), usedBlocks);
        }

        ~Arena() {
            for (auto i: availableBlocks) {
                delete[]i.data;
            }
            for (auto i:usedBlocks) {
                delete[] i.data;
            }
        }
    };
}
#endif //MIYUKIRENDERER_ARENA_HPP
