#ifndef CPM_HPP_WXYNEKFN
#define CPM_HPP_WXYNEKFN

#include <vanetza/asn1/asn1c_conversion.hpp>
#include <vanetza/asn1/asn1c_wrapper.hpp>
#include <artery/cpm/compiled/CPM.h>

namespace artery
{
namespace cpm
{
	
class Cpm : public vanetza::asn1::asn1c_wrapper<CPM_t>
{
public:
    Cpm() : vanetza::asn1::asn1c_wrapper<CPM_t>(asn_DEF_CPM) {}
};

} // namespace cpm
} // namespace artery

#endif /* CPM_HPP_WXYNEKFN */

