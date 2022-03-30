#pragma once

#include <cstdint>

namespace sim0mqpp
{

enum class FieldType : std::uint8_t
{
    BYTE_8 = 0,
    SHORT_16 = 1,
    INT_32 = 2,
    LONG_64 = 3,
    FLOAT_32 = 4,
    DOUBLE_64 = 5,
    BOOLEAN_8 = 6,
    CHAR_8 = 7,
    CHAR_16 = 8,
    STRING_8 = 9,
    STRING_16 = 10,
    BYTE_8_ARRAY = 11,
    SHORT_16_ARRAY = 12,
    INT_32_ARRAY = 13,
    LONG_64_ARRAY = 14,
    FLOAT_32_ARRAY = 15,
    DOUBLE_64_ARRAY = 16,
    BOOLEAN_8_ARRAY = 17,
    BYTE_8_MATRIX = 18,
    SHORT_16_MATRIX = 19,
    INT_32_MATRIX = 20,
    LONG_64_MATRIX = 21,
    FLOAT_32_MATRIX = 22,
    DOUBLE_64_MATRIX = 23,
    BOOLEAN_8_MATRIX = 24,
    FLOAT_32_UNIT = 25,
    DOUBLE_64_UNIT = 26,
    FLOAT_32_UNIT_ARRAY = 27,
    DOUBLE_64_UNIT_ARRAY = 28,
    FLOAT_32_UNIT_MATRIX = 29,
    DOUBLE_64_UNIT_MATRIX = 30,
    FLOAT_32_UNIT2_MATRIX = 31,
    DOUBLE_64_UNIT2_MATRIX = 32,
};


template<typename T>
struct get_field_type;

template<>
struct get_field_type<std::int8_t>
{
    static constexpr FieldType value = FieldType::BYTE_8;
};

template<>
struct get_field_type<std::int16_t>
{
    static constexpr FieldType value = FieldType::SHORT_16;
};

template<>
struct get_field_type<std::int32_t>
{
    static constexpr FieldType value = FieldType::INT_32;
};

template<>
struct get_field_type<std::int64_t>
{
    static constexpr FieldType value = FieldType::LONG_64;
};

template<>
struct get_field_type<float>
{
    static constexpr FieldType value = FieldType::FLOAT_32;
};

template<>
struct get_field_type<double>
{
    static constexpr FieldType value = FieldType::DOUBLE_64;
};

template<>
struct get_field_type<bool>
{
    static constexpr FieldType value = FieldType::BOOLEAN_8;
};

template<>
struct get_field_type<char>
{
    static constexpr FieldType value = FieldType::CHAR_8;
};

template<>
struct get_field_type<char16_t>
{
    static constexpr FieldType value = FieldType::CHAR_16;
};

} // namespace sim0mqpp
