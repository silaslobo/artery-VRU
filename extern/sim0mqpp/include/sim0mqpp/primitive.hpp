#pragma once

#include <cstddef>
#include <cstdint>

namespace sim0mqpp
{

// forward declarations
class SerializationInput;
class SerializationOutput;

void serialize(SerializationOutput&, std::int8_t);
void serialize(SerializationOutput&, std::int16_t);
void serialize(SerializationOutput&, std::int32_t);
void serialize(SerializationOutput&, std::int64_t);
void serialize(SerializationOutput&, float);
void serialize(SerializationOutput&, double);
void serialize(SerializationOutput&, bool);
void serialize(SerializationOutput&, char);
void serialize(SerializationOutput&, char16_t);

void deserialize(SerializationInput&, std::int8_t&);
void deserialize(SerializationInput&, std::int16_t&);
void deserialize(SerializationInput&, std::int32_t&);
void deserialize(SerializationInput&, std::int64_t&);
void deserialize(SerializationInput&, float&);
void deserialize(SerializationInput&, double&);
void deserialize(SerializationInput&, bool&);
void deserialize(SerializationInput&, char&);
void deserialize(SerializationInput&, char16_t&);

} // namespace sim0mqpp
