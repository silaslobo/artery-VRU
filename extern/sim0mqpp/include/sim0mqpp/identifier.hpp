#pragma once

#include <cstdint>
#include <string>
#include <boost/variant/variant.hpp>

namespace sim0mqpp
{

class SerializationInput;
class SerializationOutput;

using Identifier = boost::variant<
    std::int16_t,
    std::int32_t,
    std::int64_t,
    std::string,
    std::u16string
>;

void serialize(SerializationOutput&, const Identifier&);
void deserialize(SerializationInput&, Identifier&);

std::string to_string(const Identifier&);

} // namespace sim0mqpp
