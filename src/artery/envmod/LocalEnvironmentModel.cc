/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/application/Middleware.h"
#include "artery/envmod/EnvironmentModelObject.h"
#include "artery/envmod/EnvironmentModelPerson.h"
#include "artery/envmod/LocalEnvironmentModel.h"
#include "artery/envmod/GlobalEnvironmentModel.h"
#include "artery/envmod/sensor/Sensor.h"
#include "artery/utility/FilterRules.h"
#include <inet/common/ModuleAccess.h>
#include <omnetpp/cxmlelement.h>
#include <utility>
#include <artery/envmod/sensor/CamSensor.h>

using namespace omnetpp;

namespace artery
{

Define_Module(LocalEnvironmentModel)

static const simsignal_t EnvironmentModelRefreshSignal = cComponent::registerSignal("EnvironmentModel.refresh");

LocalEnvironmentModel::LocalEnvironmentModel() :
    mGlobalEnvironmentModel(nullptr)
{
}

int LocalEnvironmentModel::numInitStages() const
{
    return 2;
}

void LocalEnvironmentModel::initialize(int stage)
{
    if (stage == 0) {
        mGlobalEnvironmentModel = inet::getModuleFromPar<GlobalEnvironmentModel>(par("globalEnvironmentModule"), this);
        mGlobalEnvironmentModel->subscribe(EnvironmentModelRefreshSignal, this);

        auto vehicle = inet::findContainingNode(this);
        mMiddleware = inet::getModuleFromPar<Middleware>(par("middlewareModule"), vehicle);
        Facilities& fac = mMiddleware->getFacilities();
        fac.register_mutable(mGlobalEnvironmentModel);
        fac.register_mutable(this);
    } else if (stage == 1) {
        initializeSensors();
    }
}

void LocalEnvironmentModel::finish()
{
    mGlobalEnvironmentModel->unsubscribe(EnvironmentModelRefreshSignal, this);
    mObjects.clear();
    p_mObjects.clear();
}

void LocalEnvironmentModel::receiveSignal(cComponent*, simsignal_t signal, cObject* obj, cObject*)
{
    if (signal == EnvironmentModelRefreshSignal) {
        for (auto* sensor : mSensors) {
            sensor->measurement();
        }
        update();
    }
}

void LocalEnvironmentModel::complementObjects(const SensorDetection& detection, const Sensor& sensor)
{
    auto it = detection.numberOfCornersDetected.begin();

    for (auto& detectedObject : detection.objects) {
        auto foundObject = mObjects.find(detectedObject);
        if (foundObject != mObjects.end()) {
        Tracking& tracking = foundObject->second;
        tracking.tap(&sensor);
        tracking.updateQuality(&sensor, *it);
        } else {
            mObjects.emplace(detectedObject, Tracking { ++mTrackingCounter, &sensor });
        }
   }

    for (auto& detectedPerson : detection.persons) {
        auto foundPerson = p_mObjects.find(detectedPerson);
        if (foundPerson != p_mObjects.end()) {
        Tracking& trackingPerson = foundPerson->second;
        trackingPerson.tap(&sensor);
        trackingPerson.updateQuality(&sensor, *it);
        } else {
        p_mObjects.emplace(detectedPerson, Tracking { ++mTrackingCounter, &sensor });
        }
   }
}

void LocalEnvironmentModel::update()
{    
    for (auto it = p_mObjects.begin(); it != p_mObjects.end();) {
        const Person& person = it->first;
        Tracking& trackingPerson = it->second;
        trackingPerson.update();

        if (person.expired() || trackingPerson.expired()) {
            it = p_mObjects.erase(it);
        } else {
            ++it;
        }
    }    
    
    for (auto it = mObjects.begin(); it != mObjects.end();) {
        const Object& object = it->first;
        Tracking& tracking = it->second;
        tracking.update();

        if (object.expired() || tracking.expired()) {
            it = mObjects.erase(it);
        } else {
            ++it;
        }
    }
}

void LocalEnvironmentModel::initializeSensors()
{
    cXMLElement* config = par("sensors").xmlValue();
    for (cXMLElement* sensor_cfg : config->getChildrenByTagName("sensor"))
    {
        cXMLElement* sensor_filters = sensor_cfg->getFirstChildWithTag("filters");
        bool sensor_applicable = true;
        if (sensor_filters) {
            auto identity = mMiddleware->getIdentity();
            FilterRules rules(getRNG(0), identity);
            sensor_applicable = rules.applyFilterConfig(*sensor_filters);
        }

        if (sensor_applicable) {
            cModuleType* module_type = cModuleType::get(sensor_cfg->getAttribute("type"));
            const char* sensor_name = sensor_cfg->getAttribute("name");
            if (!sensor_name || !*sensor_name) {
                sensor_name = module_type->getName();
            }

            cModule* module = module_type->create(sensor_name, this);
            module->finalizeParameters();
            module->buildInside();
            auto sensor = dynamic_cast<artery::Sensor*>(module);

            if (sensor != nullptr) {
                // set sensor name at very early stage so it is available during sensor initialization
                sensor->setSensorName(sensor_name);
            } else {
                throw cRuntimeError("%s is not of type Sensor", module_type->getFullName());
            }

            module->scheduleStart(simTime());
            module->callInitialize();
            mSensors.push_back(sensor);
        }
    }
}


LocalEnvironmentModel::Tracking::Tracking(int id, const Sensor* sensor) : mId(id)
{
    mSensors.emplace(sensor, TrackingTime {});
}

LocalEnvironmentModel::Tracking::Tracking(const Sensor* sensor, const int nbVisiblePoints){
    mSensors.emplace(sensor, TrackingTime {});
    mQuality.emplace(sensor, nbVisiblePoints);
}


bool LocalEnvironmentModel::Tracking::expired() const
{
    return mSensors.empty();
}

void LocalEnvironmentModel::Tracking::update()
{
    for (auto it = mSensors.begin(); it != mSensors.end();) {
      const Sensor* sensor = it->first;
      const TrackingTime& tracking = it->second;

      const bool expired = tracking.last() + sensor->getValidityPeriod() < simTime();
      if (expired) {
          it = mSensors.erase(it);
      } else {
          ++it;
      }
    }
}

SimTime LocalEnvironmentModel::Tracking::getTimeSinceLastUpdate(){
    SimTime timeElapsedMin = 0;

    for(auto it = this->sensors().begin(); it != this->sensors().end(); it++) {
        const Sensor* sensor = it->first;
        const TrackingTime& tracking = it->second;
        if(sensor->getSensorCategory() == "CA" || sensor->getSensorCategory() == "CP"){
            SimTime timeElapsed = simTime() - tracking.last();
            if(timeElapsedMin.isZero() || timeElapsed < timeElapsedMin){
                timeElapsedMin = timeElapsed;
            }
        }
    }
    return timeElapsedMin;
}

void LocalEnvironmentModel::Tracking::tap(const Sensor* sensor)
{
    auto found = mSensors.find(sensor);
    if (found != mSensors.end()) {
         TrackingTime& tracking = found->second;
         tracking.tap();
    } else {
         mSensors.emplace(sensor, TrackingTime {});
    }
}

void LocalEnvironmentModel::Tracking::updateQuality(const Sensor* sensor, int nbCornersDetected){
    auto found = mQuality.find(sensor);
    if (found != mQuality.end()) {
        found->second = nbCornersDetected;
    } else {
        mQuality.emplace(sensor, nbCornersDetected);
    }
}

LocalEnvironmentModel::TrackingTime::TrackingTime() :
   mFirst(simTime()), mLast(simTime())
{
}

LocalEnvironmentModel::TrackingTime::TrackingTime(omnetpp::SimTime time) :
        mFirst(time), mLast(time)
{}

void LocalEnvironmentModel::TrackingTime::tap()
{
    mLast = simTime();
}

void LocalEnvironmentModel::TrackingTime::setLast(omnetpp::SimTime time)
{
    mLast = time;
}

TrackedObjectsFilterRange filterBySensorCategory(const LocalEnvironmentModel::TrackedObjects& all, const std::string& category)
{
    // capture `category` by value because lambda expression will be evaluated after this function's return
    TrackedObjectsFilterPredicate seenByCategory = [category](const LocalEnvironmentModel::TrackedObject& obj) {
        const auto& detections = obj.second.sensors();
        return std::any_of(detections.begin(), detections.end(),
                [&category](const LocalEnvironmentModel::Tracking::TrackingMap::value_type& tracking) {
                    const Sensor* sensor = tracking.first;
                    return sensor->getSensorCategory() == category;
                });
    };

    auto begin = boost::make_filter_iterator(seenByCategory, all.begin(), all.end());
    auto end = boost::make_filter_iterator(seenByCategory, all.end(), all.end());
    return boost::make_iterator_range(begin, end);
}

TrackedPersonsFilterRange pfilterBySensorCategory(const LocalEnvironmentModel::TrackedPersons& all, const std::string& category)
{
    // capture `category` by value because lambda expression will be evaluated after this function's return
    TrackedPersonsFilterPredicate seenByCategory = [category](const LocalEnvironmentModel::TrackedPerson& obj) {
        const auto& detections = obj.second.sensors();
        return std::any_of(detections.begin(), detections.end(),
                [&category](const LocalEnvironmentModel::Tracking::TrackingMap::value_type& tracking) {
                    const Sensor* sensor = tracking.first;
                    return sensor->getSensorCategory() == category;
                });
    };

    auto begin = boost::make_filter_iterator(seenByCategory, all.begin(), all.end());
    auto end = boost::make_filter_iterator(seenByCategory, all.end(), all.end());
    return boost::make_iterator_range(begin, end);
}

TrackedObjectsFilterRange filterBySensorName(const LocalEnvironmentModel::TrackedObjects& all, const std::string& name)
{
    // capture `category` by value because lambda expression will be evaluated after this function's return
    TrackedObjectsFilterPredicate seenByName = [name](const LocalEnvironmentModel::TrackedObject& obj) {
        const auto& detections = obj.second.sensors();
        return std::any_of(detections.begin(), detections.end(),
                [&name](const LocalEnvironmentModel::Tracking::TrackingMap::value_type& tracking) {
                    const Sensor* sensor = tracking.first;
                    return sensor->getSensorName() == name;
                });
    };

    auto begin = boost::make_filter_iterator(seenByName, all.begin(), all.end());
    auto end = boost::make_filter_iterator(seenByName, all.end(), all.end());
    return boost::make_iterator_range(begin, end);
}

TrackedPersonsFilterRange pfilterBySensorName(const LocalEnvironmentModel::TrackedPersons& all, const std::string& name)
{
    // capture `category` by value because lambda expression will be evaluated after this function's return
    TrackedPersonsFilterPredicate seenByName = [name](const LocalEnvironmentModel::TrackedPerson& obj) {
        const auto& detections = obj.second.sensors();
        return std::any_of(detections.begin(), detections.end(),
                [&name](const LocalEnvironmentModel::Tracking::TrackingMap::value_type& tracking) {
                    const Sensor* sensor = tracking.first;
                    return sensor->getSensorName() == name;
                });
    };

    auto begin = boost::make_filter_iterator(seenByName, all.begin(), all.end());
    auto end = boost::make_filter_iterator(seenByName, all.end(), all.end());
    return boost::make_iterator_range(begin, end);
}

} // namespace artery
