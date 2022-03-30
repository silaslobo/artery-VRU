#pragma once

#include "serialization.hpp"
#include <iterator>
#include <string>
#include <vector>

namespace sim0mqpp
{

using Buffer = std::vector<std::uint8_t>;

class BufferDeserializer : public sim0mqpp::SerializationInput
{
public:
    BufferDeserializer(const Buffer&);
    BufferDeserializer(Buffer::const_iterator begin, Buffer::const_iterator end);

    const std::string& error_message() const { return m_error; }
    void error(const char* msg) override;
    bool good() const override { return m_error.empty(); }

    void big_endian(bool be) override { m_big_endian = be; }
    std::uint8_t read_byte() override;

    void read(std::int8_t&) override;
    void read(std::int16_t&) override;
    void read(std::int32_t&) override;
    void read(std::int64_t&) override;
    void read(float&) override;
    void read(double&) override;
    void read(bool&) override;
    void read(char&) override;
    void read(char16_t&) override;

private:
    bool m_big_endian = true;
    std::string m_error;
    Buffer::const_iterator m_cursor;
    Buffer::const_iterator m_end;
};

class BufferSerializer : public sim0mqpp::SerializationOutput
{
public:
    BufferSerializer(Buffer&);

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

private:
    std::back_insert_iterator<Buffer> m_output;
};

} // namespace sim0mqpp
