#pragma once

#include "identifier.hpp"
#include "any.hpp"
#include <boost/variant/get.hpp>
#include <vector>

namespace sim0mqpp
{

// forward declarations
class SerializationInput;
class SerializationOutput;


struct Message
{
    Identifier federation_id;
    Identifier sender_id;
    Identifier receiver_id;
    Identifier message_type_id;
    Identifier message_id;
    std::vector<Any> payload;

    template<typename T>
    const T* get_payload(std::size_t pos) const
    {
        const T* elem = nullptr;
        if (payload.size() > pos) {
            elem = boost::get<T>(&payload[pos]);
        }
        return elem;
    }

    template<Unit U, typename T = ScalarQuantity<double>>
    const T* get_payload(std::size_t pos) const
    {
        const T* elem = get_payload<T>(pos);
        if (elem && elem->unit() == U) {
            return elem;
        } else {
            return nullptr;
        }
    }

    template<typename T, Unit U>
    const T* get_payload(std::size_t pos) const
    {
        return get_payload<U, T>(pos);
    }
};

void serialize(SerializationOutput&, const Message&);
void deserialize(SerializationInput&, Message&);

/**
 * Number of octets occupied by serialized message
 * \param msg message
 * \return number of octets
 */
std::size_t size(const Message&);

} // namespace sim0mqpp
