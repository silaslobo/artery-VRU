#pragma once

#include "field_type.hpp"
#include "serialization.hpp"
#include <iterator>
#include <type_traits>

namespace sim0mqpp
{

template<typename T>
void serialize(SerializationOutput& out, T begin, T end)
{
    out.write(get_field_type<typename std::iterator_traits<T>::value_type>::value);
    out.write(static_cast<std::int32_t>(std::distance(begin, end)));
    for (auto it = begin; it != end; ++it) {
        out.write(*it);
    }
}

template<typename T>
void deserialize(SerializationInput& in, T out)
{
    using iterator_traits = std::iterator_traits<T>;
    using value_type = typename iterator_traits::value_type;

    static_assert(std::is_same<std::output_iterator_tag, typename iterator_traits::iterator_category>::value,
            "output iterator is mandatory");
    std::int32_t len = 0;
    in.read(len);
    for (; len > 0; --len) {
        value_type v;
        in.read(v);
        *out++ = v;
    }
}

} // namespace sim0mqpp
