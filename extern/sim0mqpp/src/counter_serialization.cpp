#include "sim0mqpp/counter_serialization.hpp"

namespace sim0mqpp
{

bool CounterSerializer::big_endian() const
{
    return true;
}

void CounterSerializer::write_byte(std::uint8_t)
{
    ++m_counter;
}

void CounterSerializer::write(std::int8_t)
{
    ++m_counter;
}

void CounterSerializer::write(std::int16_t)
{
    m_counter += 2;
}

void CounterSerializer::write(std::int32_t)
{
    m_counter += 4;
}

void CounterSerializer::write(std::int64_t)
{
    m_counter += 8;
}

void CounterSerializer::write(float)
{
    m_counter += 4;
}

void CounterSerializer::write(double)
{
    m_counter += 8;
}

void CounterSerializer::write(bool)
{
    ++m_counter;
}

void CounterSerializer::write(char)
{
    ++m_counter;
}

void CounterSerializer::write(char16_t)
{
    m_counter += 2;
}

void CounterSerializer::reset()
{
    m_counter = 0;
}

} // namespace sim0mqpp
