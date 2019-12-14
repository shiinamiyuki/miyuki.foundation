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

#ifndef MIYUKIRENDERER_ENTITY_FUNCS_H
#define MIYUKIRENDERER_ENTITY_FUNCS_H


#include <memory>
#include <miyuki.foundation/defs.h>
#include <nlohmann/json_fwd.hpp>
#include <functional>
#include <miyuki.foundation/object.hpp>

namespace miyuki {
    class Object;

    class Type;

    MYK_PUBLIC_API std::shared_ptr<Object> CreateObject(const std::string &type);

    MYK_PUBLIC_API void RegisterObject(const std::string &type, Type *);

    MYK_PUBLIC_API void BindInterfaceImplementation(const std::string &interface, const std::string &alias);

    MYK_PUBLIC_API std::shared_ptr<Object> CreateObjectParams(const nlohmann::json &);

    MYK_PUBLIC_API void BindObject(const std::shared_ptr<Object> &entity, const std::string &name);

    MYK_PUBLIC_API std::shared_ptr<Object> GetObject(const std::string &name);

    Type * GetType(const std::string &);

    void ForeachImplementation(const std::string& interface,const std::function<void(const std::string & impl)> & f);

    void DumpJsonSchema(nlohmann::json& schema);

    template<class T>
    void Register() {
        T::_register();
    }
}

#endif //MIYUKIRENDERER_ENTITY_FUNCS_H
