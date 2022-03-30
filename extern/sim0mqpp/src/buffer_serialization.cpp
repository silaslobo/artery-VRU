#include "sim0mqpp/buffer_serialization.hpp"
#include <boost/endian/conversion.hpp>

namespace sim0mqpp
{

BufferSerializer::BufferSerializer(Buffer& buffer) :
    m_output(std::back_inserter(buffer))
{
}

void BufferSerializer::write_byte(std::uint8_t byte)
{
    *m_output++ = byte;
}

bool BufferSerializer::big_endian() const
{
    return true;
}

void BufferSerializer::write(std::int8_t num8)
{
    *m_output++ = num8;
}

void BufferSerializer::write(std::int16_t num16)
{
    *m_output++ = (num16 >> 8);
    *m_output++ = (num16);
}

void BufferSerializer::write(std::int32_t num32)
{
    *m_output++ = (num32 >> 24);
    *m_output++ = (num32 >> 16);
    *m_output++ = (num32 >> 8);
    *m_output++ = (num32);
}

void BufferSerializer::write(std::int64_t num64)
{
    *m_output++ = (num64 >> 56);
    *m_output++ = (num64 >> 48);
    *m_output++ = (num64 >> 40);
    *m_output++ = (num64 >> 32);
    *m_output++ = (num64 >> 24);
    *m_output++ = (num64 >> 16);
    *m_output++ = (num64 >> 8);
    *m_output++ = (num64);
}

void BufferSerializer::write(float f)
{
    write(*reinterpret_cast<std::int32_t*>(&f));
}

void BufferSerializer::write(double d)
{
    write(*reinterpret_cast<std::int64_t*>(&d));
}

void BufferSerializer::write(bool b)
{
    *m_output++ = b ? 1 : 0;
}

void BufferSerializer::write(char c)
{
    *m_output++ = c;
}

void BufferSerializer::write(char16_t wc)
{
    write(static_cast<std::int16_t>(wc));
}


BufferDeserializer::BufferDeserializer(const Buffer& buffer) :
    BufferDeserializer(buffer.cbegin(), buffer.cend())
{
}

BufferDeserializer::BufferDeserializer(Buffer::const_iterator begin, Buffer::const_iterator end) :
    m_cursor(begin), m_end(end)
{
}

void BufferDeserializer::error(const char* msg)
{
    if (m_error.empty()) {
        m_error = msg;
    }
}

std::uint8_t BufferDeserializer::read_byte()
{
    if (std::distance(m_cursor, m_end) >= 1) {
        return *m_cursor++;
    } else {
        error("buffer ended prematurely reading 1 byte");
    }

    return 0;
}

void BufferDeserializer::read(std::int8_t& i)
{
    i = read_byte();
}

void BufferDeserializer::read(std::int16_t& i)
{
    if (std::distance(m_cursor, m_end) >= 2) {
        std::memcpy(&i, &(*m_cursor), 2);
        if (m_big_endian) {
            boost::endian::big_to_native_inplace(i);
        } else {
            boost::endian::little_to_native_inplace(i);
        }
        std::advance(m_cursor, 2);
    } else {
        error("buffer ended prematurely reading 2 bytes");
        i = 0;
    }
}

void BufferDeserializer::read(std::int32_t& i)
{
    if (std::distance(m_cursor, m_end) >= 4) {
        std::memcpy(&i, &(*m_cursor), 4);
        if (m_big_endian) {
            boost::endian::big_to_native_inplace(i);
        } else {
            boost::endian::little_to_native_inplace(i);
        }
        std::advance(m_cursor, 4);
    } else {
        error("buffer ended prematurely reading 4 bytes");
        i = 0;
    }
}

void BufferDeserializer::read(std::int64_t& i)
{
    if (std::distance(m_cursor, m_end) >= 8) {
        std::memcpy(&i, &(*m_cursor), 8);
        if (m_big_endian) {
            boost::endian::big_to_native_inplace(i);
        } else {
            boost::endian::little_to_native_inplace(i);
        }
        std::advance(m_cursor, 8);
    } else {
        error("buffer ended prematurely reading 8 bytes");
        i = 0;
    }
}

void BufferDeserializer::read(float& f)
{
    static_assert(sizeof(float) == sizeof(std::int32_t), "float has not the same size as int32_t");
    read(reinterpret_cast<std::int32_t&>(f));
}

void BufferDeserializer::read(double& d)
{
    static_assert(sizeof(double) == sizeof(std::int64_t), "double has not the same size as int64_t");
    read(reinterpret_cast<std::int64_t&>(d));
}

void BufferDeserializer::read(bool& b)
{
    b = (read_byte() != 0);
}

void BufferDeserializer::read(char& c)
{
    c = static_cast<char>(read_byte());
}

void BufferDeserializer::read(char16_t& c)
{
    static_assert(sizeof(char16_t) == sizeof(std::int16_t), "char16_t has not the same size as int16_t");
    read(reinterpret_cast<std::int16_t&>(c));
}

} // namespace sim0mqpp
