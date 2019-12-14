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

#pragma once

#include <miyuki.foundation/defs.h>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <variant>

#include <miyuki.foundation/detail/object-funcs.h>
#include <nlohmann/json_fwd.hpp>

namespace miyuki {
    using json = nlohmann::json;
    namespace serialize {
        class InputArchive;

        class OutputArchive;
    } // namespace serialize
    class Type;

    class PropertyVisitor;

    // Base class for all entities in rendering
    class Object {
    public:
        [[nodiscard]] virtual Type *getType() const = 0;

        virtual void save(serialize::OutputArchive &) const {}

        virtual void load(serialize::InputArchive &) {}

        virtual void initialize(const json &) {}

        virtual bool isSerializable() const { return false; }

        virtual void preprocess() {}

        virtual void accept(PropertyVisitor *) {}

        virtual std::string toString() const;

        virtual ~Object() = default;

        virtual void _generateSchema(json &schema)const;

        // get a ObjectProperty from name
        //virtual std::shared_ptr<Object> getProperty(const char *name);

        //virtual void setProperty(const char * name, )
    };

    namespace serialize {
        void WriteObject(serialize::OutputArchive &ar, const std::shared_ptr<Object> &);

        std::shared_ptr<Object> ReadObject(serialize::InputArchive &ar);
    } // namespace serialize

    class Type {
    public:
        virtual std::shared_ptr<Object> create() const = 0;

        virtual const char *name() const = 0;

        virtual bool isInterface()const = 0;
    };

    template<class T>
    Type *GetStaticType(const char *_name) {
        struct TypeImpl : Type {
            const char *_name;

            explicit TypeImpl(const char *name) : _name(name) {}

            [[nodiscard]] std::shared_ptr<Object> create() const override { return std::make_shared<T>(); }

            [[nodiscard]] const char *name() const override { return _name; }

            [[nodiscard]] bool isInterface()const override {return false;}
        };
        static std::once_flag flag;
        static TypeImpl *type = nullptr;
        std::call_once(flag, [&]() { type = new TypeImpl(_name); });
        return type;
    }

    template<class T>
    Type *GetAbstraceStaticType(const char *_name) {
        struct TypeImpl : Type {
            const char *_name;

            explicit TypeImpl(const char *name) : _name(name) {}

            [[noreturn]] std::shared_ptr<Object> create() const override {
                MIYUKI_THROW(std::runtime_error, "Cannot create abstract type");
            }

            [[nodiscard]] const char *name() const override { return _name; }

            [[nodiscard]] bool isInterface()const override {return true;}
        };
        static std::once_flag flag;
        static TypeImpl *type = nullptr;
        std::call_once(flag, [&]() { type = new TypeImpl(_name); });
        return type;
    }

} // namespace miyuki

#define MYK_INTERFACE(Interface, Alias)                                                                                    \
        using Self = Interface;                                                                                            \
        static miyuki::Type *staticType() { return miyuki::GetAbstraceStaticType<Self>(Alias); }                           \

#define MYK_DECL_CLASS(Classname, Alias, ...)                                                                          \
    using Self = Classname;                                                                                            \
    static miyuki::Type *staticType() { return miyuki::GetStaticType<Self>(Alias); }                                   \
    miyuki::Type *getType() const override { return staticType(); }                                                    \
    static void _register() {                                                                                          \
        static_assert(std::is_final_v<Self>, Alias " must be final");                                                  \
        miyuki::RegisterObject(Alias, staticType());                                                                   \
        std::string interface;                                                                                         \
        miyuki::serialize::_assign(__VA_ARGS__);                                                                       \
        if (!interface.empty()) {                                                                                      \
            miyuki::BindInterfaceImplementation(interface, Alias);                                                     \
        }                                                                                                              \
    }


