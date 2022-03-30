//
// Copyright (C) 2014 Raphael Riebl <raphael.riebl@thi.de>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#ifndef ARTERY_CPSERVICE_H_
#define ARTERY_CPSERVICE_H_

#include "artery/application/ItsG5BaseService.h"
#include "artery/application/FilterObjects.h"
#include "artery/utility/Geometry.h"
#include "artery/application/InfoObject.h"
#include <artery/cpm/cpm.hpp>
#include "artery/cpm/compiled/ListOfSensorInformationContainer.h"
#include <artery/envmod/LocalEnvironmentModel.h>
#include <artery/envmod/EnvironmentModelPerson.h>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>
#include "artery/utility/Channel.h"

namespace artery
{

class NetworkInterfaceTable;
class Timer;
class PersonDataProvider;
class VehicleDataProvider;

template <class T>
T computeDifference(T a, T b, T limit, T maxValue){
	if(a >= maxValue - limit && b <= limit)
		return a - b - maxValue;
	if(a <= limit && b >= maxValue - limit)
		return a - b + maxValue;

	assert(abs(a-b) <= maxValue);
	return a - b;
}

class CPService : public ItsG5BaseService
{
	public:
		CPService();
        ~CPService();
		void initialize() override;
		void indicate(const vanetza::btp::DataIndication&, std::unique_ptr<vanetza::UpPacket>) override;
		void trigger() override;

	private:
		ChannelNumber mPrimaryChannel = channel::SCH1;
		const NetworkInterfaceTable* mNetworkInterfaceTable = nullptr;
		const PersonDataProvider* mPersonDataProvider = nullptr;
		const VehicleDataProvider* mVehicleDataProvider = nullptr;
		const Timer* mTimer = nullptr;
		omnetpp::SimTime mGenCpmMin;
		omnetpp::SimTime mGenCpmMax;
		omnetpp::SimTime mGenCpm;
		unsigned mGenCpmLowDynamicsCounter;
		unsigned mGenCpmLowDynamicsLimit;
		Position mLastCpmPosition;
		vanetza::units::Velocity mLastCpmSpeed;
		vanetza::units::Angle mLastCpmHeading;
		omnetpp::SimTime mLastCpmTimestamp;
		omnetpp::SimTime mLastSensorInfoCont;
		vanetza::units::Angle mHeadingDelta;
		vanetza::units::Length mPositionDelta;
		vanetza::units::Velocity mSpeedDelta;
		bool mDccRestriction;
		bool mFixedRate;
		std::string mTriggeringCondition;

		InfoObject::ObjectsToSendMap mObjectsToSend;
		InfoObject::ObjectsToSendMap mObjectsPrevSent;
		InfoObject::ObjectsReceivedMap mObjectsReceived;
        Sensor* mCPSensor;
        Sensor* mCASensor;
		FilterObjects mFilterObj;
	    LocalEnvironmentModel* mLocalEnvironmentModel;
		std::vector<Sensor*> mSensors;
		std::map<const Sensor*, Identifier_t> mSensorsId;
		std::vector<bool> mFiltersEnabled;
        uint32_t mStationId;
        bool mInitied = false;
        std::string mDccAlg;
		bool mOtherDynamicThresh;
		bool mUseMatchingSize;
		bool mUseThreshMatchingSize;
		double mThreshMatchingSize;
		double mStartFilteringThresh;
		bool mGenerateForSensor;
		bool mPreFilterLowDynamics;

        void retrieveInformationFromCPM(const artery::cpm::Cpm*);
		void removeExpiredObject();

		void fulfillmSensorsId();
        void checkTriggeringConditions(const omnetpp::SimTime&);
        bool checkHeadingDelta() const;
        bool checkPositionDelta() const;
        bool checkSpeedDelta() const;
		void sendCpm(const omnetpp::SimTime&);
        void requestCPMTransmission(const omnetpp::SimTime& T_now, const artery::cpm::Cpm& cpm);

        bool sendLimericAdaptedCPM(const omnetpp::SimTime&);

		omnetpp::SimTime genCpmDcc();
		artery::cpm::Cpm createCollectivePerceptionMessage(uint16_t genDeltaTime);
		void addSensorInformation(LocalEnvironmentModel*& localEnvironmentModel, artery::cpm::Cpm& message,
								  std::map<const Sensor*, Identifier_t> sensorsId);

		void createSensorInformationContainer(ListOfSensorInformationContainer_t*& seqSensInfCont,
											  Sensor*& sensor, Identifier_t id, SensorType_t sensorType);

		PerceivedObjectContainer_t* createPerceivedObjectContainer(const std::weak_ptr<artery::EnvironmentModelPerson>& object,
																	InfoObject& infoObj, CollectivePerceptionMessage_t& cpm);

        void addPerceivedObjectContainer(LocalEnvironmentModel* localEnvironmentModel, artery::cpm::Cpm& message, const omnetpp::SimTime& T_now);
		void generateASN1Objects(artery::cpm::Cpm& message, const omnetpp::SimTime& T_now, InfoObject::ObjectsToSendMap objToSend);

		void receiveSignal(cComponent*, omnetpp::simsignal_t signal, cObject *obj, cObject*);
		bool isObjectToSend(const omnetpp::SimTime& T_now);

		void printCPM(const artery::cpm::Cpm& message);

		double getRatioTimeAllowedUsed(const artery::cpm::Cpm* message);
        vanetza::Clock::duration getTonpp(const artery::cpm::Cpm *message);


    	void getToff(artery::cpm::Cpm *msg, vanetza::UnitInterval &delta, bool computeBestDelta, double &ToffFinal,
            const omnetpp::SimTime& T_now);

		void completeMyPrevObjSent(const omnetpp::SimTime& T_now, InfoObject::ObjectsToSendMap objToSend);

        void collectObjectAge();
        bool matchSpleepingTime(artery::cpm::Cpm& message, InfoObject::ObjectsToSendMap& objToSendNoFiltering, const omnetpp::SimTime& T_now);
		vanetza::UnitInterval getLimericDelta();
		omnetpp::SimTime getLimericLastDeltaUpdatingTime();

		void checkCPMSize(const omnetpp::SimTime& T_now, InfoObject::ObjectsToSendMap& objToSendNoFiltering, artery::cpm::Cpm& cpm);
		double getDCCToff();

};




} // namespace artery

#endif /* ARTERY_CPSERVICE_H_ */
