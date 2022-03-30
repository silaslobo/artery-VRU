/*
* Artery V2X Simulation Framework
* Copyright 2014-2019 Raphael Riebl et al.
* Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
*/

#ifndef ARTERY_VASERVICE_H_
#define ARTERY_VASERVICE_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/utility/Channel.h"
#include "artery/utility/Geometry.h"
#include <vanetza/asn1/cam.hpp>
#include <vanetza/asn1/vam.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>


namespace traci { class PersonController; }

namespace artery
{
class NetworkInterfaceTable;
class Timer;
class PersonDataProvider;

class VaService : public ItsG5BaseService
{
	public:
		VaService();
		void initialize() override;
		void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
		void trigger() override;

	private:
		void checkTriggeringConditions(const omnetpp::SimTime&);
		bool checkHeadingDelta() const;
		bool checkPositionDelta() const;
		bool checkSpeedDelta() const;
		void sendVam(const omnetpp::SimTime&);
		omnetpp::SimTime genVamDcc();

		ChannelNumber mPrimaryChannel = channel::CCH;
		const NetworkInterfaceTable* mNetworkInterfaceTable = nullptr;
		const PersonDataProvider* mPersonDataProvider = nullptr;
		const Timer* mTimer = nullptr;
		//LocalDynamicMap* mLocalDynamicMap = nullptr;

		omnetpp::SimTime mGenVamMin;
		omnetpp::SimTime mGenVamMax;
		omnetpp::SimTime mGenVam;
		unsigned mGenVamLowDynamicsCounter;
		unsigned mGenVamLowDynamicsLimit;
		Position mLastVamPosition;
		vanetza::units::Velocity mLastVamSpeed;
		vanetza::units::Angle mLastVamHeading;
		omnetpp::SimTime mLastVamTimestamp;
		omnetpp::SimTime mLastLowVamTimestamp;
		vanetza::units::Angle mHeadingDelta;
		vanetza::units::Length mPositionDelta;
		vanetza::units::Velocity mSpeedDelta;
		bool mDccRestriction;
		bool mFixedRate;

		//access to control Person connection among Artery, Omnetpp, and SUMO
		void personSumoData(const omnetpp::SimTime&);
		traci::PersonController* mPersonController = nullptr;
};

vanetza::asn1::Vam createVRUAwarenessMessage(const PersonDataProvider&, uint16_t genDeltaTime);
void addLowFrequencyContainer(vanetza::asn1::Vam&, unsigned pathHistoryLength = 0);

} // namespace artery

#endif /* ARTERY_VASERVICE_H_ */