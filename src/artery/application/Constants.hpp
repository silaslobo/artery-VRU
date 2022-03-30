#ifndef ARTERY_CONSTANTS_H_
#define ARTERY_CONSTANTS_H_

#include <boost/units/cmath.hpp>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <vanetza/dcc/transmission.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/length.hpp>
#include <vanetza/units/time.hpp>
#include <vanetza/units/velocity.hpp>
#include <boost/units/systems/si/prefixes.hpp>

namespace artery
{
namespace Constants
{
	const auto microdegree = vanetza::units::degree * boost::units::si::micro;
	const auto decidegree = vanetza::units::degree * boost::units::si::deci;
	const auto degree_per_second = vanetza::units::degree / vanetza::units::si::second;
	const auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

	template<typename T, typename U>
	long round(const boost::units::quantity<T>& q, const U& u)
	{
		boost::units::quantity<U> v { q };
		return std::round(v.value());
	}

} //Constants namespace

} //artery namespace

#endif //ARTERY_CONSTANTS_H_