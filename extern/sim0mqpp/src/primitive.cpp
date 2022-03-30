#include "sim0mqpp/primitive.hpp"
#include "sim0mqpp/serialization.hpp"

namespace sim0mqpp
{
namespace
{

bool check(SerializationInput& in, FieldType ft)
{
    return ft == static_cast<FieldType>(in.read_byte());
}

} // namespace

void serialize(SerializationOutput& out, std::int8_t i)
{
    out.write_byte(static_cast<std::uint8_t>(FieldType::BYTE_8));
    out.write(i);
}

void serialize(SerializationOutput& out, std::int16_t i)
{
    out.write_byte(static_cast<std::uint8_t>(FieldType::SHORT_16));
    out.write(i);
}

void serialize(SerializationOutput& out, std::int32_t i)
{
    out.write_byte(static_cast<std::uint8_t>(FieldType::INT_32));
    out.write(i);
}

void serialize(SerializationOutput& out, std::int64_t i)
{
    out.write_byte(static_cast<std::uint8_t>(FieldType::LONG_64));
    out.write(i);
}

void serialize(SerializationOutput& out, float f)
{
    out.write_byte(static_cast<std::uint8_t>(FieldType::FLOAT_32));
    out.write(f);
}

void serialize(SerializationOutput& out, double d)
{
    out.write_byte(static_cast<std::uint8_t>(FieldType::DOUBLE_64));
    out.write(d);
}

void serialize(SerializationOutput& out, bool b)
{
    out.write_byte(static_cast<std::uint8_t>(FieldType::BOOLEAN_8));
    out.write(b);
}

void serialize(SerializationOutput& out, char c)
{
    out.write_byte(static_cast<std::uint8_t>(FieldType::CHAR_8));
    out.write(c);
}

void serialize(SerializationOutput& out, char16_t c)
{
    out.write_byte(static_cast<std::uint8_t>(FieldType::CHAR_16));
    out.write(c);
}

void deserialize(SerializationInput& in, std::int8_t& i)
{
    if (check(in, FieldType::BYTE_8)) {
        in.read(i);
    } else {
        in.error("field type does not match BYTE_8");
    }
}

void deserialize(SerializationInput& in, std::int16_t& i)
{
    if (check(in, FieldType::SHORT_16)) {
        in.read(i);
    } else {
        in.error("field type does not match SHORT_16");
    }
}

void deserialize(SerializationInput& in, std::int32_t& i)
{
    if (check(in, FieldType::INT_32)) {
        in.read(i);
    } else {
        in.error("field type does not match INT_32");
    }
}

void deserialize(SerializationInput& in, std::int64_t& i)
{
    if (check(in, FieldType::LONG_64)) {
        in.read(i);
    } else {
        in.error("field type does not match LONG_64");
    }
}

void deserialize(SerializationInput& in, float& f)
{
    if (check(in, FieldType::FLOAT_32)) {
        in.read(f);
    } else {
        in.error("field type does not match FLOAT_32");
    }
}

void deserialize(SerializationInput& in, double& d)
{
    if (check(in, FieldType::DOUBLE_64)) {
        in.read(d);
    } else {
        in.error("field type does not match DOUBLE_64");
    }
}

void deserialize(SerializationInput& in, bool& b)
{
    if (check(in, FieldType::BOOLEAN_8)) {
        in.read(b);
    } else {
        in.error("field type does not match BOOLEAN_8");
    }
}

void deserialize(SerializationInput& in, char& c)
{
    if (check(in, FieldType::CHAR_8)) {
        in.read(c);
    } else {
        in.error("field type does not match CHAR_8");
    }
}

void deserialize(SerializationInput& in, char16_t& c)
{
    if (check(in, FieldType::CHAR_16)) {
        in.read(c);
    } else {
        in.error("field type does not match CHAR_16");
    }
}

} // namespace sim0mqpp
