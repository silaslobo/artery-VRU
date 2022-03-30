#pragma once

#include <string>

namespace sim0mqpp
{

// forward declarations
class SerializationInput;
class SerializationOutput;

void serialize(SerializationOutput&, const std::string&);
void serialize(SerializationOutput&, const std::u16string&);

void deserialize(SerializationInput&, std::string&);
void deserialize(SerializationInput&, std::u16string&);

} // namespace sim0mqpp
