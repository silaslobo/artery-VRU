#include "sim0mqpp/identifier.hpp"
#include "sim0mqpp/primitive.hpp"
#include "sim0mqpp/string.hpp"
#include "sim0mqpp/serialization.hpp"
#include <cstdint>
#include <boost/variant/apply_visitor.hpp>
#include <boost/variant/static_visitor.hpp>

namespace sim0mqpp
{

struct SerializeIdentifier : public boost::static_visitor<>
{
    SerializeIdentifier(SerializationOutput& out) : out_(out) {}

    template<typename T>
    void operator()(const T& x)
    {
        serialize(out_, x);
    }

    SerializationOutput& out_;
};

void serialize(SerializationOutput& out, const Identifier& id)
{
    SerializeIdentifier visitor(out);
    boost::apply_visitor(visitor, id);
}

void deserialize(SerializationInput& in, Identifier& id)
{
    // read one byte (field type), check range, read "payload"
    auto ft = static_cast<FieldType>(in.read_byte());
    if (ft == FieldType::SHORT_16) {
        std::int16_t tmp;
        in.read(tmp);
        id = tmp;
    } else if (ft == FieldType::INT_32) {
        std::int32_t tmp;
        in.read(tmp);
        id = tmp;
    } else if (ft == FieldType::LONG_64) {
        std::int64_t tmp;
        in.read(tmp);
        id = tmp;
    } else if (ft == FieldType::STRING_8) {
        std::string tmp;
        deserialize(in, tmp);
        id = std::move(tmp);
    } else if (ft == FieldType::STRING_16) {
        std::u16string tmp;
        deserialize(in, tmp);
        id = std::move(tmp);
    } else {
        in.error("unsupported field type when reading identifier");
    }
}

std::string to_string(const Identifier& id)
{
    struct Stringifier : public boost::static_visitor<std::string>
    {
        std::string operator()(std::int16_t i) { return std::to_string(i); }
        std::string operator()(std::int32_t i) { return std::to_string(i); }
        std::string operator()(std::int64_t i) { return std::to_string(i); }
        std::string operator()(const std::string& s) { return s; }
        std::string operator()(const std::u16string& s)
        {
            return std::string { s.begin(), s.end() };
        }
    };

    Stringifier visitor;
    return boost::apply_visitor(visitor, id);
}

} // namespace sim0mqpp
