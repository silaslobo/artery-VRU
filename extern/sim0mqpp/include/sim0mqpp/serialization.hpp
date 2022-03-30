#pragma once

#include "field_type.hpp"
#include "unit.hpp"
#include <cstddef>
#include <cstdint>

namespace sim0mqpp
{

class SerializationInput
{
public:
    virtual std::uint8_t read_byte() = 0;
    virtual void big_endian(bool) = 0;

    virtual void error(const char*) = 0;
    virtual bool good() const = 0;

    virtual void read(std::int8_t&) = 0;
    virtual void read(std::int16_t&) = 0;
    virtual void read(std::int32_t&) = 0;
    virtual void read(std::int64_t&) = 0;
    virtual void read(float&) = 0;
    virtual void read(double&) = 0;
    virtual void read(bool&) = 0;
    virtual void read(char&) = 0;
    virtual void read(char16_t&) = 0;

    virtual ~SerializationInput() = default;
};

class SerializationOutput
{
public:
    virtual void write_byte(std::uint8_t) = 0;
    virtual bool big_endian() const = 0;

    virtual void write(std::int8_t) = 0;
    virtual void write(std::int16_t) = 0;
    virtual void write(std::int32_t) = 0;
    virtual void write(std::int64_t) = 0;
    virtual void write(float) = 0;
    virtual void write(double) = 0;
    virtual void write(bool) = 0;
    virtual void write(char) = 0;
    virtual void write(char16_t) = 0;

    void write(FieldType ft)
    {
        static_assert(sizeof(FieldType) == sizeof(std::uint8_t),
                "field type is not a single byte wide");
        this->write_byte(static_cast<std::uint8_t>(ft));
    }

    void write(Unit u)
    {
        static_assert(sizeof(Unit) == sizeof(std::uint8_t),
                "unit type is not a single byte wide");
        this->write_byte(static_cast<std::uint8_t>(u));
    }

    virtual ~SerializationOutput() = default;
};

} // namespace sim0mqpp
