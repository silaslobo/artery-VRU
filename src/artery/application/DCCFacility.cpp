//
// Created by rosk on 29.07.20.
//

#include "DCCFacility.h"
#include "artery/networking/AccessInterface.h"
#include <vanetza/dcc/transmission.hpp>

namespace artery{

using namespace omnetpp;

Define_Module(DCCFacility);


static const simsignal_t PassedGatekeeperMessageSignal = cComponent::registerSignal("gatekeeper_message");


DCCFacility::DCCFacility():
    mMiddleware(nullptr)
{
}

DCCFacility::~DCCFacility(){}

void DCCFacility::initialize(){
    Middleware* middleware = dynamic_cast<Middleware*>(getParentModule());
    if (middleware == nullptr) {
        throw cRuntimeError("Middleware not found");
    }

    mMiddleware = middleware;
    Facilities& fac = mMiddleware->getFacilities();
    fac.register_mutable(this);

    this->subscribe(PassedGatekeeperMessageSignal);

}


void DCCFacility::receiveSignal(cComponent *, simsignal_t signal, omnetpp::cObject* obj, cObject *) {
    if(signal == PassedGatekeeperMessageSignal){
        auto* accessInfo = dynamic_cast<AccessInterface*>(obj);
        //std::cout << "Gatekeeper let message passes: "  << accessInfo->getTimeStamp();
        //std::cout << "  "  << accessInfo->getHeaderSize() << "   " <<  accessInfo->getBodySize() << "  " << accessInfo->getAllSize() << std::endl;
    }
}


void DCCFacility::subscribe(const omnetpp::simsignal_t& signal){
    assert(mMiddleware);
    mMiddleware->getParentModule()->subscribe(signal, this);
}


void DCCFacility::unsubscribe(const omnetpp::simsignal_t& signal){
    assert(mMiddleware);
    mMiddleware->getParentModule()->unsubscribe(signal, this);
}


void DCCFacility::finish(){
    cSimpleModule::finish();
}


} //namespace Artery