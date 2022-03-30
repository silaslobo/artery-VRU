/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2021 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#ifndef ARTERY_PERSONMIDDLEWARE_H_IH5GPCMP
#define ARTERY_PERSONMIDDLEWARE_H_IH5GPCMP

#include "artery/application/Middleware.h"
//#include "artery/traci/PersonMobility.h"
#include "artery/application/PersonDataProvider.h"
#include "artery/traci/PersonController.h"

namespace artery
{

class PersonMiddleware : public Middleware
{
    public:
        PersonMiddleware();
        void initialize(int stage) override;
        void finish() override;

    protected:
        void initializeStationType(const std::string&);
        void initializePersonController(omnetpp::cPar&);
        void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject*) override;

    private:
        traci::PersonController* mPersonController = nullptr;
        PersonDataProvider mPersonDataProvider;
        //PersonMobility* mMobility = nullptr;
};

} // namespace artery

#endif /* ARTERY_PERSONMIDDLEWARE_H_IH5GPCMP */

