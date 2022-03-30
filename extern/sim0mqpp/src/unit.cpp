#include "sim0mqpp/unit.hpp"

namespace sim0mqpp
{

const char* to_cstr(Unit u)
{
    const char* s = "";
    switch (u) {
        case Unit::Dimensionless:
            s = "(dimensionless)";
            break;
        case Unit::Acceleration:
            s = "acceleration";
            break;
        case Unit::SolidAngle:
            s = "solid angle";
            break;
        case Unit::Angle:
            s = "angle";
            break;
        case Unit::Direction:
            s = "direction";
            break;
        case Unit::Area:
            s = "area";
            break;
        case Unit::Density:
            s = "density";
            break;
        case Unit::ElectricalCharge:
            s = "electrical charge";
            break;
        case Unit::ElectricalCurrent:
            s = "electrical current";
            break;
        case Unit::ElectricalPotential:
            s = "electrical potential";
            break;
        case Unit::ElectricalResistance:
            s = "electrical resistance";
            break;
        case Unit::Energy:
            s = "energy";
            break;
        case Unit::FlowMass:
            s = "flow mass";
            break;
        case Unit::FlowVolume:
            s = "flow volume";
            break;
        case Unit::Force:
            s = "force";
            break;
        case Unit::Frequency:
            s = "frequency";
            break;
        case Unit::Length:
            s = "length";
            break;
        case Unit::Position:
            s = "position";
            break;
        case Unit::LinearDensity:
            s = "linear density";
            break;
        case Unit::Mass:
            s = "mass";
            break;
        case Unit::Power:
            s = "power";
            break;
        case Unit::Pressure:
            s = "pressure";
            break;
        case Unit::Speed:
            s = "speed";
            break;
        case Unit::Temperature:
            s = "temperature";
            break;
        case Unit::AbsoluteTemperature:
            s = "absolute temperature";
            break;
        case Unit::Duration:
            s = "duration";
            break;
        case Unit::Time:
            s = "time";
            break;
        case Unit::Torque:
            s = "torque";
            break;
        case Unit::Volume:
            s = "volume";
            break;
        case Unit::AbsorbedDose:
            s = "absorbed dose";
            break;
        case Unit::AmountOfSubstance:
            s = "amount of substance";
            break;
        case Unit::CatalyticActivity:
            s = "catalytic activity";
            break;
        case Unit::ElectricalCapacitance:
            s = "electrical capacitance";
            break;
        case Unit::ElectricalConductance:
            s = "electrical conductance";
            break;
        case Unit::ElectricalInductance:
            s = "electrical inductance";
            break;
        case Unit::EquivalentDose:
            s = "equivalent dose";
            break;
        case Unit::Illuminance:
            s = "illuminance";
            break;
        case Unit::LuminousFlux:
            s = "luminous flux";
            break;
        case Unit::LuminousIntensity:
            s = "luminous intensity";
            break;
        case Unit::MagneticFluxDensity:
            s = "magnetic flux density";
            break;
        case Unit::MagneticFlux:
            s = "magnetic flux";
            break;
        case Unit::Radioactivity:
            s = "radioactivity";
            break;
        case Unit::AngularAcceleration:
            s = "angular acceleration";
            break;
        case Unit::AngularVelocity:
            s = "angular velocity";
            break;
        case Unit::Momentum:
            s = "momentum";
            break;
        default:
            s = "unknown";
            break;
    }
    return s;
}

std::string to_string(Unit u)
{
    return to_cstr(u);
}

} // namespace sim0mqpp
