#include "sim0mqpp/any.hpp"
#include "sim0mqpp/primitive.hpp"
#include "sim0mqpp/string.hpp"
#include "sim0mqpp/serialization.hpp"
#include <boost/variant/apply_visitor.hpp>
#include <boost/variant/static_visitor.hpp>
#include <cstdio>

namespace sim0mqpp
{

struct SerializationAny : public boost::static_visitor<>
{
    SerializationAny(SerializationOutput& out) : out_(out) {}

    template<typename T>
    void operator()(const T& t)
    {
        serialize(out_, t);
    }

    SerializationOutput& out_;
};

void serialize(SerializationOutput& out, const Any& any)
{
    SerializationAny visitor(out);
    boost::apply_visitor(visitor, any);
}

FieldType deserialize(SerializationInput& in, Any& any)
{
    const auto ft = static_cast<FieldType>(in.read_byte());

    switch (ft) {
        case FieldType::BYTE_8: {
            std::int8_t tmp;
            in.read(tmp);
            any = tmp; }
            break;

        case FieldType::SHORT_16: {
            std::int16_t tmp;
            in.read(tmp);
            any = tmp; }
            break;

        case FieldType::INT_32: {
            std::int32_t tmp;
            in.read(tmp);
            any = tmp; }
            break;

        case FieldType::LONG_64: {
            std::int64_t tmp;
            in.read(tmp);
            any = tmp; }
            break;

        case FieldType::FLOAT_32: {
            float tmp;
            in.read(tmp);
            any = tmp; }
            break;

        case FieldType::DOUBLE_64: {
            double tmp;
            in.read(tmp);
            any = tmp; }
            break;

        case FieldType::BOOLEAN_8: {
            bool tmp;
            in.read(tmp);
            any = tmp; }
            break;

        case FieldType::CHAR_8: {
            char tmp;
            in.read(tmp);
            any = tmp; }
            break;

        case FieldType::CHAR_16: {
            char16_t tmp;
            in.read(tmp);
            any = tmp; }
            break;

        case FieldType::STRING_8: {
            std::string tmp;
            deserialize(in, tmp);
            any = std::move(tmp); }
            break;

        case FieldType::STRING_16: {
            std::u16string tmp;
            deserialize(in, tmp);
            any = std::move(tmp); }
            break;

        case FieldType::FLOAT_32_UNIT: {
            ScalarQuantity<float> tmp;
            deserialize(in, tmp);
            any = std::move(tmp); }
            break;

        case FieldType::DOUBLE_64_UNIT: {
            ScalarQuantity<double> tmp;
            deserialize(in, tmp);
            any = std::move(tmp); }
            break;

        case FieldType::FLOAT_32_UNIT_ARRAY: {
            VectorQuantity<float> tmp;
            deserialize(in, tmp);
            any = std::move(tmp); }
            break;

        case FieldType::DOUBLE_64_UNIT_ARRAY: {
            VectorQuantity<double> tmp;
            deserialize(in, tmp);
            any = std::move(tmp); }
            break;

        default:
            in.error("unsupported field type");
            break;
    }

    return ft;
}

namespace
{

template<typename T>
void print(char* buf, std::size_t len, const VectorQuantity<T>& q)
{
    const auto& v = q.values();
    if (v.empty()) {
        std::snprintf(buf, len, "[empty] (%s)", to_cstr(q.unit()));
    } else if (v.size() == 1) {
        std::snprintf(buf, len, "[%g] (%s)", v[0], to_cstr(q.unit()));
    } else if (v.size() == 2) {
        std::snprintf(buf, len, "[%g, %g] (%s)", v[0], v[1], to_cstr(q.unit()));
    } else if (v.size() == 3) {
        std::snprintf(buf, len, "[%g, %g, %g] (%s)", v[0], v[1], v[2], to_cstr(q.unit()));
    } else {
        std::snprintf(buf, len, "[%d values] (%s)", v.size(), to_cstr(q.unit()));
    }
}

} // namespace

std::string to_string(const Any& any)
{
    struct Stringifier : public boost::static_visitor<std::string>
    {
        std::string operator()(std::int8_t byte)
        {
            std::snprintf(buf, sizeof(buf), "0x%hhX", byte);
            return buf;
        }

        std::string operator()(std::int16_t i) { return std::to_string(i); }
        std::string operator()(std::int32_t i) { return std::to_string(i); }
        std::string operator()(std::int64_t i) { return std::to_string(i); }

        std::string operator()(float f)
        {
            std::snprintf(buf, sizeof(buf), "%g", f);
            return buf;
        }

        std::string operator()(double d)
        {
            std::snprintf(buf, sizeof(buf), "%g", d);
            return buf;
        }

        std::string operator()(bool b) { return b ? "true" : "false"; }

        std::string operator()(char c)
        {
            std::snprintf(buf, sizeof(buf), "'%c'", c);
            return buf;
        }

        std::string operator()(char16_t c)
        {
            std::snprintf(buf, sizeof(buf), "'%lc'", c);
            return buf;
        }

        std::string operator()(const std::string& s)
        {
            std::string r;
            r.push_back('"');
            r.append(s);
            r.push_back('"');
            return r;
        }

        std::string operator()(const std::u16string& s)
        {
            std::string r;
            r.push_back('"');
            r.append(s.begin(), s.end());
            r.push_back('"');
            return r;
        }

        std::string operator()(const ScalarQuantity<float>& q)
        {
            std::snprintf(buf, sizeof(buf), "%g (%s)", q.value(), to_cstr(q.unit()));
            return buf;
        }

        std::string operator()(const ScalarQuantity<double>& q)
        {
            std::snprintf(buf, sizeof(buf), "%g (%s)", q.value(), to_cstr(q.unit()));
            return buf;
        }

        std::string operator()(const VectorQuantity<float>& q)
        {
            print(buf, sizeof(buf), q);
            return buf;
        }

        std::string operator()(const VectorQuantity<double>& q)
        {
            print(buf, sizeof(buf), q);
            return buf;
        }

        char buf[64];
    };

    Stringifier visitor;
    return boost::apply_visitor(visitor, any);
}


} // namespace sim0mqpp
