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

#include <memory>
#include <type_traits>
#include <vector>

#include <miyuki.foundation/defs.h>
#include <miyuki.foundation/math.hpp>
#include <miyuki.foundation/spectrum.h>
#include <miyuki.foundation/reflection-visitor.hpp>

namespace miyuki {
    class Object;

    class PropertyIterator {
    public:
        virtual void next(PropertyVisitor *visitor) = 0;

        virtual bool hasNext() const = 0;

        virtual ~PropertyIterator() = default;
    };

    class PropertyVisitor;

    class Property {
    public:
        virtual void accept(PropertyVisitor *visitor) = 0;
    };

    namespace detail {
        template<class T>
        class BasicProperty : public Property {
            const char *_name;
            std::reference_wrapper<T> ref;

        public:
            BasicProperty(const char *name, T &ref) : _name(name), ref(ref) {}

            const T &getConstRef() const { return ref; }

            T &getRef() { return ref; }

            void set(const T &v) { ref = v; }

            virtual void accept(PropertyVisitor *visitor);

            //void accept(PropertyVisitor *visitor) { visitor->visit(this); }
            const char *name() const { return _name; }
        };
    } // namespace detail
    using BoolProperty = detail::BasicProperty<bool>;
    using IntProperty = detail::BasicProperty<int>;
    using FloatProperty = detail::BasicProperty<float>;
    using Float3Property = detail::BasicProperty<Vec3f>;
    using RGBProperty = detail::BasicProperty<core::RGBSpectrum>;
    using Float2Property = detail::BasicProperty<Point2f>;
    using Int2Property = detail::BasicProperty<Point2i>;

    using FileProperty = detail::BasicProperty<fs::path>;

    class ObjectProperty : public detail::BasicProperty<std::shared_ptr<Object>> {
    public:
        using detail::BasicProperty<std::shared_ptr<Object>>::BasicProperty;
        Type *type = nullptr;

        inline void accept(PropertyVisitor *visitor) override;
    };

    class PropertyVisitor {
    public:
        virtual void visit(Property *prop) { prop->accept(this); }

        virtual void visit(BoolProperty *) = 0;

        virtual void visit(IntProperty *) = 0;

        virtual void visit(FloatProperty *) = 0;

        virtual void visit(Float3Property *) = 0;

        virtual void visit(RGBProperty *) = 0;

        virtual void visit(ObjectProperty *) = 0;

        virtual void visit(FileProperty *) = 0;

        virtual void visit(Int2Property *) = 0;

        virtual void visit(Float2Property *) = 0;

        virtual void visit(PropertyIterator *iter) {
            iterPrologue(iter);
            while (iter->hasNext()) {
                iter->next(this);
            }
            iterEpilogue(iter);
        }

        virtual void iterPrologue(PropertyIterator *iter) {}

        virtual void iterEpilogue(PropertyIterator *iter) {}
    };
    namespace detail {
        template<class T>
        void BasicProperty<T>::accept(miyuki::PropertyVisitor *visitor) {
            visitor->visit(this);
        }
    }

    inline void ObjectProperty::accept(miyuki::PropertyVisitor *visitor) {
        visitor->visit(this);
    }

    struct ReflPropertyVisitor {
        PropertyVisitor *visitor;

        explicit ReflPropertyVisitor(PropertyVisitor *visitor) : visitor(visitor) {}


        template<class T>
        void visit(Degrees<T> &degrees, const char *name) {
            visit(degrees.get(), name);
        }

        template<class T>
        void visit(Radians<T> &radians, const char *name) {
            Degrees<T> degrees(radians);
            visit(degrees, name);
            radians = Radians<T>(degrees);
        }

        void visit(int &v, const char *name) {
            IntProperty prop(name, v);
            prop.accept(visitor);
        }

        void visit(bool &v, const char *name) {
            BoolProperty prop(name, v);
            prop.accept(visitor);
        }

        void visit(float &v, const char *name) {
            FloatProperty prop(name, v);
            prop.accept(visitor);
        }

        void visit(Vec3f &v, const char *name) {
            Float3Property prop(name, v);
            prop.accept(visitor);
        }

        void visit(core::RGBSpectrum &v, const char *name) {
            RGBProperty prop(name, v);
            prop.accept(visitor);
        }

        void visit(TransformManipulator &transform, const char *name) {
            visit(transform.translation, "translation");
            visit(transform.rotation, "rotation");
        }

        void visit(Point2f &v, const char *name) {
            Float2Property prop(name, v);
            prop.accept(visitor);
        }

        void visit(Point2i &v, const char *name) {
            Int2Property prop(name, v);
            prop.accept(visitor);
        }

        template<class T>
        std::enable_if_t<std::is_base_of_v<Object, T>, void> visit(std::vector<std::shared_ptr<T>> &v,
                                                                   const char *name) {
            class Iter : public PropertyIterator {
                std::vector<std::shared_ptr<T>> &vec;
                typename std::vector<std::shared_ptr<T>>::iterator iter;
                const char *name;
            public:
                Iter(const char *name, std::vector<std::shared_ptr<T>> &vec) : name(name),
                                                                               vec(vec) { iter = vec.begin(); }

                void next(PropertyVisitor *visitor) {
                    std::shared_ptr<Object> p = *iter;
                    ObjectProperty prop(name, p);
                    prop.accept(visitor);
                    *iter = std::dynamic_pointer_cast<T>(p);
                    iter++;
                }

                bool hasNext() const { return iter != vec.end(); }
            };
            Iter iter(name, v);
            visitor->visit(&iter);
        }

        template<class T>
        std::enable_if_t<std::is_base_of_v<Object, T>, void> visit(std::shared_ptr<T> &v, const char *name) {
            std::shared_ptr<Object> p = v;
            ObjectProperty prop(name, p);
            prop.type = T::staticType();
            prop.accept(visitor);
            v = std::dynamic_pointer_cast<T>(p);
        }
    };

#define MYK_PROP(...)                                                                                                  \
    void accept(miyuki::PropertyVisitor *visitor) override {                                                           \
        miyuki::ReflPropertyVisitor refl(visitor);                                                                     \
        MYK_REFL(refl, __VA_ARGS__);                                                                                   \
    }
} // namespace miyuki