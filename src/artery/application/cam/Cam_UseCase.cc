#include "artery/application/cam/Cam_UseCase.h"
#include "artery/application/CaService.h"
#include "artery/application/StoryboardSignal.h"
#include "artery/application/VehicleDataProvider.h"
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/checkandcast.h>
#include <vanetza/facilities/cam_functions.hpp>
#include "artery/application/ItsG5BaseService.h"
#include "artery/application/Middleware.h"

namespace artery
{

namespace cam
{

static const auto microdegree = vanetza::units::degree * boost::units::si::micro;
static const auto decidegree = vanetza::units::degree * boost::units::si::deci;
static const auto centimeter_per_second = vanetza::units::si::meter_per_second * boost::units::si::centi;

template<typename T, typename U>
long round(const boost::units::quantity<T>& q, const U& u)
{
	boost::units::quantity<U> v { q };
	return std::round (v.value());
}

void Cam_UseCase::initialize(int stage)
{
	
	if (stage == 0)
	{
		//EV_INFO << "here" << std::endl;
		mService = omnetpp::check_and_cast<CaService*>(getParentModule());
		mVdp = &mService->getFacilities().get_const<VehicleDataProvider>();
	}

	
	

}

vanetza::asn1::Cam Cam_UseCase::createMessageSkeleton()
{
	vanetza::asn1::Cam message;
	message->header.protocolVersion = 1;
	message->header.messageID = ItsPduHeader__messageID_cam;
	message->header.stationID = mVdp->station_id();


	return message;
}


} // namespace cam
} // namespace artery




