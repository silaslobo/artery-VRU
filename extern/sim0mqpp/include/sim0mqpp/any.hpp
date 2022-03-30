#pragma once

#include "field_type.hpp"
#include "quantity.hpp"
#include <boost/variant/variant.hpp>
#include <string>

namespace sim0mqpp
{

// forward declarations
class SerializationInput;
class SerializationOutput;

using Any = boost::variant<
    std::int8_t,
    std::int16_t,
    std::int32_t,
    std::int64_t,
    float,
    double,
    bool,
    char,
    char16_t,
    std::string,
    std::u16string,
    ScalarQuantity<float>,
    ScalarQuantity<double>,
    VectorQuantity<float>,
    VectorQuantity<double>
    // array and matrix types missing yet
>;

void serialize(SerializationOutput&, const Any&);
FieldType deserialize(SerializationInput&, Any&);

std::string to_string(const Any&);

} // namespace sim0mqpp
