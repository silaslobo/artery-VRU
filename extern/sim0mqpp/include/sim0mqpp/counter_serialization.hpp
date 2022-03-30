#pragma once

#include "serialization.hpp"

namespace sim0mqpp
{

/**
 * Special-purpose serializer only counting bytes used by serialization
 */
class CounterSerializer : public SerializationOutput
{
public:
    bool big_endian() const override;
    void write_byte(std::uint8_t) override;

    void write(std::int8_t) override;
    void write(std::int16_t) override;
    void write(std::int32_t) override;
    void write(std::int64_t) override;
    void write(float) override;
    void write(double) override;
    void write(bool) override;
    void write(char) override;
    void write(char16_t) override;

    /**
     * Get number of output octets "used" by serialization
     */
    std::size_t counter() const { return m_counter; }

    /**
     * Reset octet counter to zero
     */
    void reset();

private:
    std::size_t m_counter = 0; /*< number of serialized octets */
};

} // namespace sim0mqpp
