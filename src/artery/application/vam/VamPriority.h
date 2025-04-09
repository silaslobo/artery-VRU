#ifndef ARTERY_VAMPRIORITY_H_
#define ARTERY_VAMPRIORITY_H_

#include "artery/inet/PowerLevelRx.h"
#include "artery/application/VaService.h"
#include "artery/application/PersonDataProvider.h"
#include "artery/application/LocalDynamicMap.h"
#include "artery/application/ClusterManagement.h"
#include "vanetza/dcc/profile.hpp"
#include "artery/utility/Channel.h"
#include "artery/inet/ChannelLoadRx.h"
#include "random"

namespace artery{
using namespace omnetpp;

class NetworkInterfaceTable;

class VamPriority
{
public:
    VamPriority();
    vanetza::dcc::Profile setDP(artery::LocalDynamicMap* ldm, double cbr);       

private:
    PowerLevelRx channelLoad;
    std::vector<int> poissonServiceDistribution;
    std::poisson_distribution<int> mTriggerProb;
    
    std::vector<double> PoissonDistributionPMF(LocalDynamicMap::StationID id, omnetpp::SimTime time);
    

    enum MarkovStates {       
            /**
             * S1: for CBR < 0.30
             */
            Flexible = 0,

            /**
             * S2: for 0.30 <= CBR < 0.5
             */
            Dynamic = 1,

            /**
             * S3: for CBR >= 0.5
             */
            Limited = 2,

            /**
             * for use cases with no target, e.g. even that all souround nodes broadcast at 10Hz the CBR will not reach any of the limits
             */
            NoTarget = 3,  

            /**
             * for use cases with no target, e.g. even that all souround nodes broadcast at 10Hz the CBR will not reach any of the limits
             */
            NoStateSet = 4,  

        };
    
    struct targetStates{
        MarkovStates tState1;
        MarkovStates tState2;
    };

    targetStates getTargetStates(double);

    //map with states limits
    std::map<MarkovStates, std::vector<double>> markovStates(std::map<LocalDynamicMap::StationID, std::vector<double>> poissonDist, double cbr);
    std::map<VamPriority::MarkovStates,double> probabilityAnalysis (std::map<LocalDynamicMap::StationID, std::vector<double>> poissonDist, double cbr, std::map<VamPriority::MarkovStates, std::vector<double>> targetStates);
    std::map<VamPriority::MarkovStates,double> markovChainDefinition (std::map<MarkovStates,double> markovTargets, double cbr);

    //MarkovStates initialMarkovState{NoStateSet};    
	//void setInitialMarkovState(MarkovStates currentState){initialMarkovState=currentState;}
    
    int bytesToBits (int bytes) {return bytes * 8;}

    


};

} // namespace artery

#endif /* ARTERY_VAMPRIORITY_H_ */