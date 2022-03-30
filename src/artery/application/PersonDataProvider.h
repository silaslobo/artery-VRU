#ifndef ARTERY_PERSONDATAPROVIDER_H
#define ARTERY_PERSONDATAPROVIDER_H

#include "artery/application/PersonKinematics.h"
#include "artery/utility/Geometry.h"
#include <omnetpp/simtime.h>
#include <boost/circular_buffer.hpp>
#include <boost/units/systems/si/angular_acceleration.hpp>
#include <vanetza/geonet/station_type.hpp>
#include <vanetza/units/acceleration.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <vanetza/units/angular_velocity.hpp>
#include <vanetza/units/curvature.hpp>
#include <cstdint>
#include <map>

namespace artery
{
class PersonDataProvider
{
    public:
        using StationType = vanetza::geonet::StationType;

        PersonDataProvider(uint32_t id);

        // prevent inadvertent VDP copies
        PersonDataProvider(const PersonDataProvider&) = delete;
		PersonDataProvider& operator=(const PersonDataProvider&) = delete;

        void update(const PersonKinematics&);
        omnetpp::SimTime updated() const { return mLastUpdate; }

        const Position& position() const { return mPersonKinematics.p_position; }
        vanetza::units::GeoAngle longitude() const { return mPersonKinematics.p_geo_position.longitude; } // positive for east
		vanetza::units::GeoAngle latitude() const { return mPersonKinematics.p_geo_position.latitude; } // positive for north
		vanetza::units::Velocity speed() const { return mPersonKinematics.p_speed; }
		vanetza::units::Acceleration acceleration() const { return mPersonKinematics.p_acceleration; }
		vanetza::units::Angle heading() const { return mPersonKinematics.p_heading; } // degree from north, clockwise
		vanetza::units::AngularVelocity yaw_rate() const { return mPersonKinematics.p_yaw_rate; } // left turn positive
		vanetza::units::Curvature curvature() const { return mCurvature; } // 1/m radius, left turn positive
		double curvature_confidence() const { return mConfidence; } // percentage value

        void setStationType(StationType);
        StationType getStationType() const;

        void setStationId(uint32_t id);
        uint32_t getStationId() const { return mStationId; }
		uint32_t station_id() const { return mStationId; } /*< deprecated, use getStationId */

    private:
        typedef boost::units::quantity<boost::units::si::angular_acceleration> AngularAcceleration;
		void calculateCurvature();
		void calculateCurvatureConfidence();
		double mapOntoConfidence(AngularAcceleration) const;

		uint32_t mStationId;
		StationType mStationType;
		PersonKinematics mPersonKinematics;
		vanetza::units::Curvature mCurvature;
		double mConfidence;
		omnetpp::SimTime mLastUpdate;
		boost::circular_buffer<vanetza::units::Curvature> mCurvatureOutput;
		boost::circular_buffer<AngularAcceleration> mCurvatureConfidenceOutput;
		vanetza::units::AngularVelocity mCurvatureConfidenceInput;
		static const std::map<AngularAcceleration, double> mConfidenceTable;
};

}// namespace artery

#endif ARTERY_PERSONDATAPROVIDER_H