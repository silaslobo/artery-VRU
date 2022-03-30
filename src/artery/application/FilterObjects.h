

#ifndef ARTERY_FILTEROBJECTS_H
#define ARTERY_FILTEROBJECTS_H

#include "artery/utility/Geometry.h"
#include <artery/envmod/LocalEnvironmentModel.h>
#include "artery/application/InfoObject.h"
#include <vanetza/asn1/cam.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>

namespace artery
{
    //class CPService;

    class FilterObjects
    {
    public:

        FilterObjects();

        FilterObjects(const PersonDataProvider*, LocalEnvironmentModel*, std::vector<bool>,
                      vanetza::units::Angle, vanetza::units::Length, vanetza::units::Velocity,
                      std::map<const Sensor*, Identifier_t>*, const omnetpp::SimTime&,
                      const omnetpp::SimTime&);

        void initialize(const PersonDataProvider*, LocalEnvironmentModel*, std::vector<bool>,
                          vanetza::units::Angle, vanetza::units::Length, vanetza::units::Velocity,
                          std::map<const Sensor*, Identifier_t>*, const omnetpp::SimTime&,
                          const omnetpp::SimTime&);

        std::size_t filterObjects(InfoObject::ObjectsToSendMap &, InfoObject::ObjectsToSendMap &,
                           omnetpp::SimTime, Sensor *, InfoObject::ObjectsReceivedMap&, const omnetpp::SimTime& T_now);

        void changeDeltas(vanetza::units::Angle hd, vanetza::units::Length pd, vanetza::units::Velocity sd);

        void getObjToSendNoFilter(InfoObject::ObjectsToSendMap &objToSend, bool removeLowDynamics,
                InfoObject::ObjectsToSendMap objectsPrevSent, const omnetpp::SimTime& T_now);


    private:

        const PersonDataProvider* mPersonDataProvider;
        const LocalEnvironmentModel* mLocalEnvironmentModel;
        std::vector<bool> mFiltersEnabled;
        vanetza::units::Angle mHeadingDelta;
        vanetza::units::Length mPositionDelta;
        vanetza::units::Velocity mSpeedDelta;
        omnetpp::SimTime mTimeDelta;
        std::map<const Sensor*, Identifier_t>* mSensorsId;
        omnetpp::SimTime mGenCpmMin;
        omnetpp::SimTime mGenCpmMax;


        bool checkHeadingDelta(vanetza::units::Angle, vanetza::units::Angle) const;
        bool checkPositionDelta(Position, Position) const;
        bool checkSpeedDelta(vanetza::units::Velocity,  vanetza::units::Velocity) const;
        bool checkTimeDelta(omnetpp::SimTime T_prev, omnetpp::SimTime T_now) const;

        bool v2xCapabilities(const LocalEnvironmentModel::TrackedPerson&,
                            const LocalEnvironmentModel::Tracking::TrackingMap&,
                            InfoObject::ObjectsReceivedMap&);


        bool objectDynamicsLocal(const LocalEnvironmentModel::TrackedPerson& ,
                                const LocalEnvironmentModel::Tracking::TrackingMap&,
                                InfoObject::ObjectsToSendMap&, omnetpp::SimTime T_now);

        bool objectDynamicsV2X(const LocalEnvironmentModel::TrackedPerson&,
                              const LocalEnvironmentModel::Tracking::TrackingMap&,
                              Sensor * cpSensor, omnetpp::SimTime,
                              InfoObject::ObjectsReceivedMap&);


        bool fovSensors(const LocalEnvironmentModel::TrackedPerson&,
                         const LocalEnvironmentModel::Tracking::TrackingMap&,
                         omnetpp::SimTime);


        bool perceptionQuality(const LocalEnvironmentModel::TrackedPerson&,
                                              const LocalEnvironmentModel::Tracking::TrackingMap&,
                                              omnetpp::SimTime);


        bool updatingTime(const LocalEnvironmentModel::TrackedPerson& ,
                         const LocalEnvironmentModel::Tracking::TrackingMap&,
                         Sensor *, omnetpp::SimTime,
                         InfoObject::ObjectsReceivedMap&, omnetpp::SimTime T_now);

        bool etsiFilter(const LocalEnvironmentModel::TrackedPerson& obj,
                       const LocalEnvironmentModel::Tracking::TrackingMap& sensorsDetection,
                       InfoObject::ObjectsToSendMap& prevObjSent,
                       InfoObject::ObjectsReceivedMap& objReceived,
                       const omnetpp::SimTime& T_now);

    };

} // namespace artery



#endif //ARTERY_FILTEROBJECTS_H
