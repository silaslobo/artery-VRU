#ifndef EVIRONMENTMODELPERSON_H_
#define EVIRONMENTMODELPERSON_H_

#include "artery/application/PersonDataProvider.h"
#include "artery/envmod//Geometry.h"
#include "artery/envmod/sensor/SensorPosition.h"
#include "artery/traci/PersonType.h"
#include "artery/utility/Geometry.h"
#include <boost/optional/optional.hpp>
#include <cstdint>
#include <memory>
#include <vector>
#include <omnetpp/simtime.h>

namespace traci {class PersonController; }

namespace artery
{

/**
 * EnvironmentModelPerson
 */

class EnvironmentModelPerson : private PersonDataProvider
{
public:
    using Length = traci::PersonType::Length;

    /**
     * @param ctrl associated VehicleController to this object
     * @param id station ID used by this object for application messages (e.g. CAM)
     */
    EnvironmentModelPerson(const traci::PersonController*, uint32_t id);

    /**
     * Updates the internal object data.
     */
    void update();

    /**
     * Returns the polygon describing the object's outline
     * @return polygon points
     */
    const std::vector<Position>& getOutline() const { return mOutline; }

    /**
     * Returns a sensor attachment point of the vehicle object
     * @param pos logical position of sensor
     * @return sensor attachment point
     */
    const Position& getAttachmentPoint(const SensorPosition& pos) const;

    const PersonDataProvider& getPersonData() const;

    std::string getPersonId() const;

    /**
     * Return the centre point coord of this vehicle object
     * @return centre point
     */
    const Position& getCentrePoint() const { return mCentrePoint; }

    Length getLength() const { return mLength; }

    Length getWidth() const { return mWidth; }

    /**
     * Return outer object radius
     *
     * Object is guaranteed to lie completely in the circle described by getCentrePoint and getRadius().
     * @return outer radius
     */
    Length getRadius() const { return mRadius; }

private:
    const traci::PersonController* mPersonController;
    traci::PersonType::Length mLength;
    traci::PersonType::Length mWidth;
    traci::PersonType::Length mRadius;
    std::vector<Position> mOutline;
    std::vector<Position> mAttachmentPoints;
    Position mCentrePoint;
};

} // namespace artery

#endif /* EVIRONMENTMODELPERSON_H_ */
