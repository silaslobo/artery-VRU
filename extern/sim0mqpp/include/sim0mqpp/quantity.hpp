#pragma once

#include "unit.hpp"
#include <vector>

namespace sim0mqpp
{

// forward declarations
class SerializationInput;
class SerializationOutput;

template<typename T>
class ScalarQuantity
{
public:
    using value_type = T;

    ScalarQuantity() : m_value(0), m_unit(Unit::Dimensionless), m_display(0) {}
    ScalarQuantity(value_type v, Unit u) :
        m_value(v), m_unit(u), m_display(0) {}

    value_type value() const { return m_value; }
    void value(value_type v) { m_value = v; }

    Unit unit() const { return m_unit; }
    void unit(Unit u) { m_unit = u; }

    std::uint8_t display() const { return m_display; }
    void display(std::uint8_t d) { m_display = d; }

private:
    value_type m_value;
    Unit m_unit;
    std::uint8_t m_display;
};

void serialize(SerializationOutput& out, const ScalarQuantity<float>& q);
void serialize(SerializationOutput& out, const ScalarQuantity<double>& q);

void deserialize(SerializationInput& in, ScalarQuantity<float>& q);
void deserialize(SerializationInput& in, ScalarQuantity<double>& q);


template<typename T>
class VectorQuantity
{
public:
    using value_type = T;

    VectorQuantity() : m_unit(Unit::Dimensionless), m_display(0) {}
    VectorQuantity(std::vector<value_type> v, Unit u) :
        m_vector(std::move(v)), m_unit(u), m_display(0) {}

    const std::vector<value_type>& values() const { return m_vector; }
    void values(std::vector<value_type> v) { m_vector = std::move(v); }

    Unit unit() const { return m_unit; }
    void unit(Unit u) { m_unit = u; }

    std::uint8_t display() const { return m_display; }
    void display(std::uint8_t d) { m_display = d; }

private:
    std::vector<T> m_vector;
    Unit m_unit;
    std::uint8_t m_display;
};

void serialize(SerializationOutput& out, const VectorQuantity<float>& q);
void serialize(SerializationOutput& out, const VectorQuantity<double>& q);

void deserialize(SerializationInput& in, VectorQuantity<float>& q);
void deserialize(SerializationInput& in, VectorQuantity<double>& q);

} // namespace sim0mqpp
