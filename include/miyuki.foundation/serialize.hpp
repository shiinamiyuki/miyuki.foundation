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

#ifndef MIYUKIRENDERER_SERIALIZE_HPP
#define MIYUKIRENDERER_SERIALIZE_HPP

#include <miyuki.foundation/defs.h>
#include <miyuki.foundation/detail/object-funcs.h>
#include <miyuki.foundation/reflection-visitor.hpp>
#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>
#include <cereal/details/static_object.hpp>
#include <cereal/types//vector.hpp>
#include <nlohmann/json.hpp>
#include <unordered_set>

namespace miyuki {
    class Object;
}

#include <miyuki.foundation/math.hpp>

namespace glm {
    using json = nlohmann::json;

    template<int N, typename T, qualifier Q>
    inline void to_json(json &j, const vec <N, T, Q> &v) {
        j = json::array();
        for (int i = 0; i < N; i++) {
            j[i] = v[i];
        }
    }

    template<int N, typename T, qualifier Q>
    inline void from_json(const json &j, vec <N, T, Q> &v) {
        for (int i = 0; i < N; i++) {
            v[i] = j[i];
        }
    }

    inline void to_json(json &j, const mat4 &m) {
        j = json::array();
        for (int i = 0; i < 4; i++) {
            j[i] = m[i];
        }
    }

    inline void from_json(const json &j, mat4 &m) {
        for (int i = 0; i < 4; i++) {
            m[i] = j[i].get<vec4>();
        }
    }
}
namespace miyuki {
    inline void to_json(json &j, const Transform &transform) { j = transform.matrix(); }

    inline void from_json(const json &j, Transform &transform) { transform = Transform(j.get<mat4>()); }


    template<class T>
    void to_json(json &j, const Degrees<T> &v) {
        j = v.get();
    }

    template<class T>
    void to_json(json &j, const Radians<T> &v) {
        j = Degrees<T>(v).get();
    }

    template<class T>
    void from_json(const json &j, Degrees<T> &v) {
        v.get() = j.get<T>();
    }

    template<class T>
    void from_json(const json &j, Radians<T> &v) {
        v = Radians<T>(j.get<Degrees<T>>());
    }

    inline void to_json(json &j, const TransformManipulator &transform) {
        j = json::object();
        j["rotation"] = transform.rotation;
        j["translation"] = transform.translation;
    }

    inline void from_json(const json &j, TransformManipulator &transform) {
        transform.rotation = j.at("rotation").get<Radians<vec3>>();
        transform.translation = j.at("translation").get<vec3>();
    }

} // namespace miyuki
namespace nlohmann {
    template<typename T>
    struct adl_serializer<std::shared_ptr<T>> {
        static void to_json(json &j, const std::shared_ptr<T> &opt) { MIYUKI_NOT_IMPLEMENTED(); }

        static void from_json(const json &j, std::shared_ptr<T> &p) {
            p = std::dynamic_pointer_cast<T>(miyuki::CreateObjectParams(j));
        }
    };
} // namespace nlohmann
namespace miyuki::serialize {
    using nlohmann::json;

    class InputArchive;

    class OutputArchive;

    class InputArchive : public cereal::JSONInputArchive {
        std::unordered_map<size_t, std::shared_ptr<Object>> _refs;
        std::unordered_set<size_t> _serialized;

    public:
        using JSONInputArchive::JSONInputArchive;

        template<class T>
        void addRef(size_t addr, const std::shared_ptr<T> &p) { _refs[addr] = p; }

        template<class T>
        void addRef(size_t addr, const std::weak_ptr<T> &p) { _refs[addr] = p.lock(); }

        void addSerializedRef(size_t addr) { _serialized.insert(addr); }

        bool hasRegistered(size_t addr) const { return _refs.find(addr) != _refs.end(); }

        bool hasSerialized(size_t addr) const { return _serialized.find(addr) != _serialized.end(); }

        template<class T>
        std::shared_ptr<T> getRef(size_t addr) const {
            return std::dynamic_pointer_cast<T>(_refs.at(addr));
        }
    };

    class OutputArchive : public cereal::JSONOutputArchive {
        std::unordered_set<Object *> _serialized;

    public:
        using JSONOutputArchive::JSONOutputArchive;

        template<class T>
        void addSerialized(std::shared_ptr<T> p) { _serialized.insert(p.get()); }

        template<class T>
        bool hasSerialized(std::shared_ptr<T> p) const {
            return _serialized.find(p.get()) != _serialized.end();
        }

        template<class T>
        void addSerialized(std::weak_ptr<T> p) { _serialized.insert(p.get()); }

        template<class T>
        bool hasSerialized(std::weak_ptr<T> p) const {
            return _serialized.find(p.get()) != _serialized.end();
        }
    };
} // namespace miyuki::serialize

namespace cereal {
    template<class Archive, class T>
    inline void save(Archive &ar, std::shared_ptr<T> const &p) {
        // using Archive = kaede::OutputArchive;
        if (!p) {
            ar(CEREAL_NVP_("valid", 0));
        } else {

            ar(CEREAL_NVP_("type", std::string(p->getType()->name())));
            ar(CEREAL_NVP_("addr", reinterpret_cast<size_t>(static_cast<miyuki::Object *>(p.get()))));
            auto _archive = dynamic_cast<miyuki::serialize::OutputArchive *>(&ar);
            if (!_archive) {
                MIYUKI_THROW(std::runtime_error, "Archive must be of OutputArchive");
            }
            auto &archive = *_archive;
            if (!archive.hasSerialized(p)) {
                archive.addSerialized(p);
                p->save(archive);
                ar(CEREAL_NVP_("valid", 2));
            } else {
                ar(CEREAL_NVP_("valid", 1));
            }
        }
    }

    template<class Archive, class T>
    inline void load(Archive &ar, std::shared_ptr<T> &p) {
        int32_t valid = 2;
        ar(CEREAL_NVP_("valid", valid));
        if (!valid) {
            p = nullptr;
        } else {
            size_t addr;
            ar(CEREAL_NVP_("addr", addr));
            std::string type;
            ar(CEREAL_NVP_("type", type));
            auto _archive = dynamic_cast<miyuki::serialize::InputArchive *>(&ar);
            if (!_archive) {
                MIYUKI_THROW(std::runtime_error, "Archive must be of InputArchive");
            }
            auto &archive = *_archive;
            if (!archive.hasRegistered(addr)) {
                p = std::dynamic_pointer_cast<T>(miyuki::CreateObject(type));
                archive.addRef(addr, p);
            } else {
                p = archive.getRef<T>(addr);
            }
            if (valid & 2) {
                if (archive.hasSerialized(addr)) {
                    MIYUKI_THROW(std::runtime_error, "Load the same object twice!");
                }
                p->load(archive);
                archive.addSerializedRef(addr);
            }
        }
    }

    template<class Archive, class T, typename = std::enable_if_t<std::is_base_of_v<miyuki::Object, T>>>
    inline void save(Archive &ar, std::weak_ptr<T> const &p) {
        // using Archive = kaede::OutputArchive;
        if (!p) {
            ar(CEREAL_NVP_("valid", 0));
        } else {

            ar(CEREAL_NVP_("type", std::string(p->getType()->name())));
            ar(CEREAL_NVP_("addr", reinterpret_cast<size_t>(static_cast<miyuki::Object *>(p.get()))));
            auto _archive = dynamic_cast<miyuki::serialize::OutputArchive *>(&ar);
            if (!_archive) {
                MIYUKI_THROW(std::runtime_error, "Archive must be of OutputArchive");
            }
            auto &archive = *_archive;
            if (!archive.hasSerialized(p)) {
                archive.addSerialized(p);
                p->save(archive);
                ar(CEREAL_NVP_("valid", 2));
            } else {
                ar(CEREAL_NVP_("valid", 1));
            }
        }
    }

    template<class Archive, class T>
    inline void load(Archive &ar, std::weak_ptr<T> &p) {
        int32_t valid = 2;
        ar(CEREAL_NVP_("valid", valid));
        if (!valid) {
            p = nullptr;
        } else {
            size_t addr;
            ar(CEREAL_NVP_("addr", addr));
            std::string type;
            ar(CEREAL_NVP_("type", type));
            auto _archive = dynamic_cast<miyuki::serialize::InputArchive *>(&ar);
            if (!_archive) {
                MIYUKI_THROW(std::runtime_error, "Archive must be of InputArchive");
            }
            auto &archive = *_archive;
            if (!archive.hasRegistered(addr)) {
                auto ref = std::dynamic_pointer_cast<T>(miyuki::CreateObject(type));
                archive.addRef(addr, ref);
                p = ref;
            } else {
                p = archive.getRef<T>(addr);
            }
            if (valid & 2) {
                if (archive.hasSerialized(addr)) {
                    MIYUKI_THROW(std::runtime_error, "Load the same object twice!");
                }
                p->load(archive);
                archive.addSerializedRef(addr);
            }
        }
    }
} // namespace cereal
namespace cereal {
    template<class Archive, class T>
    void safe_apply(Archive &ar, const char *name, T &val) {
        try {
            ar(CEREAL_NVP_(name, val));
        } catch (cereal::Exception &e) {
            (void) e; //
        }
    }
} // namespace cereal

namespace miyuki::serialize {
    template<class Archive>
    struct ArchivingVisitor {
        Archive &ar;

        ArchivingVisitor(Archive &ar) : ar(ar) {}

        template<class T>
        void visit(T &v, const char *name) { cereal::safe_apply(ar, name, v); }

        template<class T>
        void visit(const T &v, const char *name) { cereal::safe_apply(ar, name, v); }
    };

    namespace detail {
        template<class T, typename _=void>
        struct JsonSchemaGenerator {
        };

        template<>
        struct JsonSchemaGenerator<bool> {
            static void generate(json &schema) {
                schema["type"] = "boolean";
            }
        };

        template<>
        struct JsonSchemaGenerator<int> {
            static void generate(json &schema) {
                schema["type"] = "integer";
            }
        };

        template<>
        struct JsonSchemaGenerator<float> {
            static void generate(json &schema) {
                schema["type"] = "number";
            }
        };

        template<>
        struct JsonSchemaGenerator<std::string> {
            static void generate(json &schema) {
                schema["type"] = "string";
            }
        };

        template<class T>
        struct JsonSchemaGenerator<
                std::vector<T>
        > {
            static void generate(json &schema) {
                schema["type"] = "array";
                schema["items"] = json::object();
                JsonSchemaGenerator<T>::generate(schema["items"]);
            }
        };

        template<class T>
        struct JsonSchemaGenerator<
                std::unordered_map<std::string, T>
        > {
            static void generate(json &schema) {
                schema["type"] = "object";
                schema["additionalProperties"] = json::object();
                JsonSchemaGenerator<T>::generate(schema["additionalProperties"]);
            }
        };


        template<class T>
        struct JsonSchemaGenerator<Degrees<T>> {
            static void generate(json &schema) {
                schema["type"] = "object";
                schema["degrees"] = json::object();
                JsonSchemaGenerator<T>::generate(schema["degrees"]);
            }
        };

        template<class T>
        struct JsonSchemaGenerator<Radians<T>> {
            static void generate(json &schema) {
                schema["type"] = "object";
                schema["radians"] = json::object();
                JsonSchemaGenerator<T>::generate(schema["degrees"]);
            }
        };

        template<class T>
        struct JsonSchemaGenerator<std::shared_ptr<T>, std::enable_if_t<std::is_base_of_v<Object, T>, void>> {
            static void generate(json &schema) {
                schema["type"] = "object";
                schema["properties"] = {};
                schema["properties"]["type"] = {
                        {"type", "string"}
                };
                schema["properties"]["valid"] = {
                        {"type", "integer"}
                };
                schema["properties"]["addr"] = {
                        {"type", "integer"}
                };
                schema["properties"]["data"] ={};
                auto type = T::staticType();
                if (type->isInterface()) {
                    // any of its subtypes
                    schema["properties"]["data"] = {{"anyOf", json::array()}};
                    json &anyOf = schema["properties"]["data"]["anyOf"];
                    ForeachImplementation(type->name(), [&](const std::string &impl) {
                        anyOf.emplace_back(json::object({{"$ref", std::string("#/definitions/").append(impl)}}));
                    });
                } else {
                    // non interface
                    schema["properties"]["data"] = {
                            {"$ref", std::string("#/definitions/").append(type->name())},
                    };
                }
            }
        };

        template<int N, class T, glm::qualifier Q>
        struct JsonSchemaGenerator<
                glm::vec<N, T, Q>
        > {
            static void generate(json &schema) {
                schema["type"] = "array";
                schema["items"] = json::object();
                JsonSchemaGenerator<T>::generate(schema["items"]);
            }
        };

        template<>
        struct JsonSchemaGenerator<TransformManipulator> {
            static void generate(json &schema) {
                schema["type"] = "object";
                schema["properties"] = {};
                schema["properties"]["rotation"] = {};
                schema["properties"]["translation"] = {};
                JsonSchemaGenerator<decltype(TransformManipulator::translation)>::generate(
                        schema["properties"]["translation"]);
                JsonSchemaGenerator<decltype(TransformManipulator::rotation)>::generate(
                        schema["properties"]["rotation"]);
            }
        };

    }


    struct JsonSchemaGeneratorVisitor {
        json &schema;

        explicit JsonSchemaGeneratorVisitor(json &schema) : schema(schema) {
            schema["properties"] = {};
            schema["type"] = "object";
        }

        template<class T>
        void visit(const T &v, const char *name) {
            schema["properties"][name] = {};
            detail::JsonSchemaGenerator<T>::generate(schema["properties"][name]);
        }

    };

    struct InitializeVisitor {
        const json &params;

        explicit InitializeVisitor(const json &params) : params(params) {}

        template<class T>
        void visit(T &v, const char *name) {
            if (params.contains(name)) {
                v = params.at(name).get<T>();
            }
        }
    };

    template<class... Args>
    void _assign(Args...) noexcept {}

    template<class>
    struct GetMethodSelfType {
    };
    template<class T, class Ret, class... Args>
    struct GetMethodSelfType<Ret (T::*)(Args...) const> {
        using type = std::decay_t<T>;
    };
} // namespace miyuki::serialize

#define _MYK_POLY_SER                                                                                                  \
    void save(miyuki::serialize::OutputArchive &ar) const override {                                                   \
        using Archive = miyuki::serialize::OutputArchive;                                                              \
        ar(CEREAL_NVP_("data", *this));                                                                               \
    }                                                                                                                  \
    void load(miyuki::serialize::InputArchive &ar) override {                                                          \
        using Archive = miyuki::serialize::InputArchive;                                                               \
        ar(CEREAL_NVP_("data", *this));                                                                               \
    }                                                                                                                  \
    bool isSerializable() const override { return true; }

#define MYK_SER_IMPL(ar)                                                                                               \
    template <class Archive> void save(Archive &ar) const {                                                            \
        const_cast<std::decay_t<decltype(*this)> &>(*this)._save_load(ar);                                             \
    }                                                                                                                  \
    template <class Archive> void load(Archive &ar) { _save_load(ar); }                                                \
    _MYK_POLY_SER                                                                                                      \
    template <class Archive> void _save_load(Archive &ar)

#define MYK_AUTO_SER(...)                                                                                              \
    template <class Archive> void save(Archive &ar) const {                                                            \
        miyuki::serialize::ArchivingVisitor v(ar);                                                                     \
        MYK_REFL(v, __VA_ARGS__);                                                                                      \
    }                                                                                                                  \
    template <class Archive> void load(Archive &ar) {                                                                  \
        miyuki::serialize::ArchivingVisitor v(ar);                                                                     \
        MYK_REFL(v, __VA_ARGS__);                                                                                      \
    }                                                                                                                  \
    void _generateSchema(json &schema)const override {                                                                          \
        miyuki::serialize::JsonSchemaGeneratorVisitor v(schema);                                                 \
        MYK_REFL(v, __VA_ARGS__);                                                                                      \
    }                                                                                                                  \
    _MYK_POLY_SER

#define MYK_AUTO_INIT(...)                                                                                             \
    void initialize(const json &params) override {                                                                     \
        miyuki::serialize::InitializeVisitor visitor(params);                                                          \
        MYK_REFL(visitor, __VA_ARGS__);                                                                                \
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
    void save(Archive &ar, const miyuki::TransformManipulator &m) {
        ar(CEREAL_NVP_("translation", m.translation));
        ar(CEREAL_NVP_("rotation", m.rotation));
    }

    template<class Archive>
    void load(Archive &ar, miyuki::TransformManipulator &m) {
        ar(CEREAL_NVP_("translation", m.translation));
        ar(CEREAL_NVP_("rotation", m.rotation));
    }

    template<class Archive, class T>
    void serialize(Archive &ar, miyuki::Degrees<T> &v) {
        ar(CEREAL_NVP_("degrees", v.get()));
    }

    template<class Archive, class T>
    void serialize(Archive &ar, miyuki::Radians<T> &v) {
        ar(CEREAL_NVP_("radians", v.get()));
    }

} // namespace cereal
namespace cereal {

    template<typename Archive, int N, typename T, glm::qualifier Q>
    void save(Archive &ar, const glm::vec<N, T, Q> &v) {
        std::vector<T> _v;
        for (int i = 0; i < N; i++) {
            _v.emplace_back(v[i]);
        }
        ar(_v);
    }

    template<typename Archive, int N, typename T, glm::qualifier Q>
    void load(Archive &ar, glm::vec<N, T, Q> &v) {
        std::vector<T> _v;
        ar(_v);
        for (int i = 0; i < N; i++) {
            v[i] = _v.at(i);
        }
    }

    template<typename Archive, int C, int R, typename T, glm::qualifier Q>
    void serialize(Archive &ar, glm::mat<C, R, T, Q> &v) {
        for (int i = 0; i < C; i++) {
            ar(v[i]);
        }
    }
}

#endif // MIYUKIRENDERER_SERIALIZE_HPP
