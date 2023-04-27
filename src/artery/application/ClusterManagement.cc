#include "artery/application/ClusterManagement.h"
#include "artery/application/VaObject.h"
#include "artery/application/VaService.h"
#include <vanetza/asn1/vam.hpp>
#include <omnetpp/simtime.h>
#include <string>
#include <artery/envmod/LocalEnvironmentModel.h>
#include <artery/envmod/EnvironmentModelObject.h>

namespace artery
{
namespace bg = boost::geometry;
using namespace omnetpp;

ClusterManagement::ClusterManagement(VruRoleType initialVruRole) : currentVruRole(initialVruRole) {}

void ClusterManagement::setCurrentVruRole(VruRoleType newVruRole){
    currentVruRole = newVruRole;
}

ClusterManagement::VruRoleType ClusterManagement::getCurrentVruRole(){
    return currentVruRole;
}

}