#ifndef ARTERY_CLUSTERMANAGEMENT_H_
#define ARTERY_CLUSTERMANAGEMENT_H_

#include "artery/application/VaObject.h"
#include "artery/application/VaService.h"
#include "artery/utility/Geometry.h"
#include <vanetza/asn1/vam.hpp>
#include <omnetpp/simtime.h>
#include <string>
#include <artery/envmod/LocalEnvironmentModel.h>
#include <artery/envmod/EnvironmentModelObject.h>

namespace artery{

class VaService;
class PersonDataProvider;
class LocalEnvironmentModel;

class ClusterManagement
{
    public:
        enum VruRoleType {VruIdle, VruActiveStandalone, VruActiveClusterLeader, VruPassive};
        ClusterManagement(VruRoleType initialVruRole = VruActiveStandalone);

        bool joinOperation();
        bool leaveOperation();
        bool breakupOperation();

        void setCurrentVruRole(VruRoleType newVruRole);
        VruRoleType getCurrentVruRole();

    private:
        VruRoleType currentVruRole;

        std::map<artery::LocalDynamicMap::StationID, artery::LocalDynamicMap::AwarenessEntry> updateLDM(artery::LocalDynamicMap);
        bool checkClusteringConditions();

};

} // namespace artery

#endif /* ARTERY_CLUSTERMANAGEMENT_H_ */