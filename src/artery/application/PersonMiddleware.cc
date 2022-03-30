/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2021 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/application/StationType.h"
#include "artery/application/PersonMiddleware.h"
#include "artery/traci/ControllablePerson.h"
#include "artery/traci/MobilityBase.h"
#include "artery/utility/InitStages.h"
#include "inet/common/ModuleAccess.h"

using namespace omnetpp;

namespace artery
{

Define_Module(PersonMiddleware)

PersonMiddleware::PersonMiddleware() :
    mPersonDataProvider(0) // OMNeT++ assigns RNG after construction: set final station ID later
{
}

void PersonMiddleware::initialize(int stage)
{   
    if (stage == InitStages::Self) {
        findHost()->subscribe(MobilityBase::stateChangedSignal, this);
        initializePersonController(par("mobilityModule"));
        initializeStationType(mPersonController->getVehicleClass());
        getFacilities().register_const(&mPersonDataProvider);
        mPersonDataProvider.update(p_getKinematics(*mPersonController));

        Identity identity;
        identity.traci = mPersonController->getPersonId();
        identity.application = Identity::randomStationId(getRNG(0));
        mPersonDataProvider.setStationId(identity.application);
        emit(Identity::changeSignal, Identity::ChangeTraCI | Identity::ChangeStationId, &identity);
    }                

    Middleware::initialize(stage);

    //Previous development from Riebl
    /*
    if (stage == InitStages::Self) {
        mMobility = inet::getModuleFromPar<PersonMobility>(par("mobilityModule"), findHost());
        setStationType(vanetza::geonet::StationType::Pedestrian);
        getFacilities().register_const(mMobility);

        Identity identity;
        identity.traci = mMobility->getPersonId();
        identity.application = Identity::randomStationId(getRNG(0));
        emit(Identity::changeSignal, Identity::ChangeTraCI | Identity::ChangeStationId, &identity);
    }

    Middleware::initialize(stage);
    */
}

void PersonMiddleware::finish()
{
    Middleware::finish();
    findHost()->unsubscribe(MobilityBase::stateChangedSignal, this);
}

void PersonMiddleware::initializeStationType(const std::string& vclass)
{
    auto gnStationType = deriveStationTypeFromVehicleClass(vclass);
    setStationType(gnStationType);
    mPersonDataProvider.setStationType(gnStationType);
}

void PersonMiddleware::initializePersonController(cPar& mobilityPar)
{
	auto mobility = inet::getModuleFromPar<ControllablePerson>(mobilityPar, findHost());
	mPersonController = mobility->getPersonController();
	ASSERT(mPersonController);
	getFacilities().register_mutable(mPersonController);
}

void PersonMiddleware::receiveSignal(cComponent* component, simsignal_t signal, cObject* obj, cObject* details)
{
	if (signal == MobilityBase::stateChangedSignal && mPersonController) {
		mPersonDataProvider.update(p_getKinematics(*mPersonController));
	}
}

} // namespace artery

