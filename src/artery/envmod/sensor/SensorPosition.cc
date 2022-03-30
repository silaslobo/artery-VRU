/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Hendrik-Joern Guenther, Raphael Riebl, Oliver Trauer
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/envmod/sensor/SensorPosition.h"
#include <boost/algorithm/string/case_conv.hpp>
#include <omnetpp/cexception.h>
#include <limits>

namespace artery
{

Angle relativeAngle(SensorPosition pos)
{
    Angle angle;

    switch (pos) {
        case SensorPosition::FRONT:
            angle = Angle::from_degree(0.0);
            break;
        case SensorPosition::BACK:
            angle = Angle::from_degree(180.0);
            break;
        case SensorPosition::LEFT:
            angle = Angle::from_degree(270.0);
            break;
        case SensorPosition::RIGHT:
            angle = Angle::from_degree(90.0);
            break;
        default:
            angle = Angle::from_degree(std::numeric_limits<double>::quiet_NaN());
    }

    return angle;
}

boost::units::quantity<boost::units::degree::plane_angle> relativeAngle1(SensorPosition pos)
{
    using boost::units::degree::degree;
    using quantity = boost::units::quantity<boost::units::degree::plane_angle>;

    quantity angle;
    switch (pos) {
        case SensorPosition::FRONT:
            angle = 0.0 * degree;
            break;
        case SensorPosition::BACK:
            angle = 180.0 * degree;
            break;
        case SensorPosition::LEFT:
            angle = 90.0 * degree;
            break;
        case SensorPosition::RIGHT:
            angle = 270.0 * degree;
            break;
        default:
            angle = quantity::from_value(std::numeric_limits<double>::quiet_NaN());
    }

    return angle;
}

SensorPosition determineSensorPosition(const std::string& id)
{
    static const std::unordered_map<std::string, SensorPosition> sensorPositionStrings = {
        {"VIRTUAL", SensorPosition::VIRTUAL},
        {"FRONT", SensorPosition::FRONT},
        {"BACK", SensorPosition::BACK},
        {"LEFT", SensorPosition::LEFT},
        {"RIGHT", SensorPosition::RIGHT},
    };

    auto found = sensorPositionStrings.find(boost::algorithm::to_upper_copy(id));
    if (found == sensorPositionStrings.end()) {
        throw omnetpp::cRuntimeError("Cannot map %s to a valid sensor position", id.c_str());
    }
    return found->second;
}

std::pair<long, long> relativePosition(SensorPosition pos)
{

    std::pair<long, long> position;
    switch(pos)
    {
        case SensorPosition::FRONT:
            position = std::make_pair(FRONTMIDX, FRONTMIDY);
            break;

        case SensorPosition::BACK:
            position = std::make_pair(BACKMIDX,BACKMIDY);
            break;

        case SensorPosition::LEFT:
            position = std::make_pair(LETFTMIDX,LETFTMIDY);
            break;

        case SensorPosition::RIGHT:
            position = std::make_pair(RIGHTTMIDX,RIGHTMIDY);
            break;
    }
    return position;
}


} // namespace artery
