#include "sim0mqpp/string.hpp"
#include "sim0mqpp/serialization.hpp"

namespace sim0mqpp
{

void serialize(SerializationOutput& out, const std::string& s)
{
    out.write_byte(static_cast<std::uint8_t>(FieldType::STRING_8));
    out.write(static_cast<std::int32_t>(s.size()));
    for (char c : s) {
        out.write(c);
    }
}

void serialize(SerializationOutput& out, const std::u16string& s)
{
    out.write_byte(static_cast<std::uint8_t>(FieldType::STRING_16));
    out.write(static_cast<std::int32_t>(s.size()));
    for (char16_t c : s) {
        out.write(c);
    }
}

void deserialize(SerializationInput& in, std::string& s)
{
    std::int32_t length = 0;
    in.read(length);
    if (length > 0) {
        s.reserve(s.size() + length);
    }
    for (std::int32_t i = 0; i < length; ++i) {
        char c;
        in.read(c);
        s.push_back(c);
    }
}

void deserialize(SerializationInput& in, std::u16string& s)
{
    std::int32_t length = 0;
    in.read(length);
    if (length > 0) {
        s.reserve(s.size() + length);
    }
    for (std::int32_t i = 0; i < length; ++i) {
        char16_t c;
        in.read(c);
        s.push_back(c);
    }
}

} // namespace sim0mqpp
