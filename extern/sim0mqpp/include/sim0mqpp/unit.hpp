#pragma once

#include <cstdint>
#include <string>

namespace sim0mqpp
{

enum class Unit : std::uint8_t
{
    Dimensionless = 0,
    Acceleration = 1,
    SolidAngle = 2,
    Angle = 3,
    Direction = 4,
    Area = 5,
    Density = 6,
    ElectricalCharge = 7,
    ElectricalCurrent = 8,
    ElectricalPotential = 9,
    ElectricalResistance = 10,
    Energy = 11,
    FlowMass = 12,
    FlowVolume = 13,
    Force = 14,
    Frequency = 15,
    Length = 16,
    Position = 17,
    LinearDensity = 18,
    Mass = 19,
    Power = 20,
    Pressure = 21,
    Speed = 22,
    Temperature = 23,
    AbsoluteTemperature = 24,
    Duration = 25,
    Time = 26,
    Torque = 27,
    Volume = 28,
    AbsorbedDose = 29,
    AmountOfSubstance = 30,
    CatalyticActivity = 31,
    ElectricalCapacitance = 32,
    ElectricalConductance = 33,
    ElectricalInductance = 34,
    EquivalentDose = 35,
    Illuminance = 36,
    LuminousFlux = 37,
    LuminousIntensity = 38,
    MagneticFluxDensity = 39,
    MagneticFlux = 40,
    Radioactivity = 41,
    AngularAcceleration = 42,
    AngularVelocity = 43,
    Momentum = 44
};

/**
 * Get string representation of a unit
 * \param u unit
 * \return unit's string representation
 */
std::string to_string(Unit u);

/**
 * Get string representation of a unit
 * \param u unit
 * \return null-terminated string representation
 */
const char* to_cstr(Unit u);

} // namespace sim0mqpp
