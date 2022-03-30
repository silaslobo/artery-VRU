#include "sim0mqpp/counter_serialization.hpp"
#include "sim0mqpp/message.hpp"
#include "sim0mqpp/primitive.hpp"
#include "sim0mqpp/string.hpp"
#include "sim0mqpp/serialization.hpp"
#include <string>

namespace sim0mqpp
{

namespace
{

static const std::string magic_number { "SIM02" };

} // namespace

void serialize(SerializationOutput& out, const Message& msg)
{
    // Frame 0: Magic number
    serialize(out, magic_number);

    // Frame 1: Endianness
    serialize(out, out.big_endian());

    // Frame 2: Federation ID
    serialize(out, msg.federation_id);

    // Frame 3: Sender ID
    serialize(out, msg.sender_id);

    // Frame 4: Receiver ID
    serialize(out, msg.receiver_id);

    // Frame 5: Messsage type ID
    serialize(out, msg.message_type_id);

    // Frame 6: Message ID
    serialize(out, msg.message_id);

    // Frame 7: Number of fields
    serialize(out, static_cast<std::int32_t>(msg.payload.size()));

    // Frame 8-n: Payload fields
    for (const auto& payload : msg.payload) {
        serialize(out, payload);
    }
}

void deserialize(SerializationInput& in, Message& msg)
{
    FieldType magic_number_ft = static_cast<FieldType>(in.read_byte());
    if (magic_number_ft == FieldType::STRING_8) {
        std::string magic_number_in;
        deserialize(in, magic_number_in);
        if (magic_number_in != magic_number) {
            in.error("sim0mq message's magic number mismatch");
        }
    } else {
        in.error("sim0mq message's magic number is not a STRING_8");
    }

    bool big_endian_in;
    deserialize(in, big_endian_in);
    in.big_endian(big_endian_in);

    deserialize(in, msg.federation_id);
    deserialize(in, msg.sender_id);
    deserialize(in, msg.receiver_id);
    deserialize(in, msg.message_type_id);
    deserialize(in, msg.message_id);

    const auto ft = static_cast<FieldType>(in.read_byte());
    std::size_t fields = 0;
    if (ft == FieldType::BYTE_8) {
        std::int8_t tmp = 0;
        in.read(tmp);
        fields = tmp;
    } else if (ft == FieldType::SHORT_16) {
        std::int16_t tmp = 0;
        in.read(tmp);
        fields = tmp;
    } else if (ft == FieldType::INT_32) {
        std::int32_t tmp = 0;
        in.read(tmp);
        fields = tmp;
    } else if (ft == FieldType::LONG_64) {
        std::int64_t tmp = 0;
        in.read(tmp);
        fields = tmp;
    } else {
        in.error("number of fields is encoded with invalid field type");
    }

    for (std::size_t i = 0; i < fields; ++i) {
        Any tmp;
        deserialize(in, tmp);
        msg.payload.emplace_back(std::move(tmp));
    }
}

std::size_t size(const Message& msg)
{
    CounterSerializer output;
    serialize(output, msg);
    return output.counter();
}

} // namespace sim0mqpp
