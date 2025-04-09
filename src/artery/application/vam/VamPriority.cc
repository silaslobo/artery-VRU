#include "artery/application/vam/VamPriority.h"
#include <iostream>
#include <vanetza/dcc/transmit_rate_control.hpp>
#include <boost/math/special_functions/factorials.hpp>
#include <algorithm>

namespace artery{


using namespace omnetpp;

VamPriority::VamPriority(){
    
}

vanetza::dcc::Profile VamPriority::setDP(artery::LocalDynamicMap *map, double cbr){
    vanetza::dcc::Profile DP;
    double localCBR = cbr;
    
    //identifying all sourroundings ITS-S
    auto lDM = map->localMap();
    std::map<LocalDynamicMap::StationID, std::vector<double>> poissonPMF;
    static const omnetpp::SimTime lifetime { 1100, omnetpp::SIMTIME_MS };
    
    //Creating the Poisson Matrix distribution for each node
    for (auto const& itr : *lDM){
        poissonPMF.insert(std::pair<LocalDynamicMap::StationID, std::vector<double>>(itr.first,VamPriority::PoissonDistributionPMF(itr.first,(itr.second.expiry - lifetime))));
    }        

    std::map<VamPriority::MarkovStates, std::vector<double>> targetStates;

    if (poissonPMF.size() != 0) {
        targetStates = markovStates(poissonPMF,cbr);
    }
    std::map<VamPriority::MarkovStates,double> markovChain;
    if (poissonPMF.size() != 0){
        markovChain = probabilityAnalysis(poissonPMF,cbr,targetStates);
    }


    std::map<VamPriority::MarkovStates, std::vector<double>> markovChainStatesPower;
    if (markovChain.size() != 0){
        for (const auto k : markovChain){            
            std::vector<double> matrixPower;
            for (int p=1; p<4; ++p){
                double prob = k.second;
                matrixPower.push_back(std::pow(k.second,p));
            }
            markovChainStatesPower.insert(std::pair<VamPriority::MarkovStates, std::vector<double>>(k.first, matrixPower));
        }        
    } 

    for (const auto itr : markovChainStatesPower){
        if (itr.first == MarkovStates::Limited){
            for (auto vecItr : itr.second){
                if (vecItr >= 0.50){
                    DP = vanetza::dcc::Profile::DP3;
                }else
                { 
                    DP = vanetza::dcc::Profile::DP2;
                }
            } 
        } else { DP = vanetza::dcc::Profile::DP2;}
    }   

    poissonPMF.clear();

    return DP; 
}

std::vector<double> VamPriority::PoissonDistributionPMF(LocalDynamicMap::StationID id, omnetpp::SimTime time)
{
    auto T_now = simTime();
    auto mean = 0.375;
    auto lambda = mean * (T_now - time) + mean;
    
    std::vector<double> poissonPDFvector;

    for (int k = 0; k <=10; ++k){
        poissonPDFvector.push_back(std::exp(-lambda.dbl()) * std::pow(lambda.dbl(), k) / std::tgamma(k + 1));        
    }    

    std::vector<double> poissonPDFvectorNorm;
    double sumNorm = 0.0;

    for (const double& value : poissonPDFvector) {
        sumNorm += value;
    }

    for (const double& itr : poissonPDFvector) {
        poissonPDFvectorNorm.push_back(itr / sumNorm);
    }
    
    return poissonPDFvectorNorm;
}

std::map<VamPriority::MarkovStates, std::vector<double>> VamPriority::markovStates(std::map<LocalDynamicMap::StationID, std::vector<double>> poissonDist, double cbr)
{
    
    VamPriority::targetStates targetStates = getTargetStates(cbr);    //Initial Markov State
      
    std::map<MarkovStates, std::vector<double>> target;    

    std::vector<double> dynamicLimitsVector = {0.3, 0.5};
    std::vector<double> flexibleLimitsVector = {0.0, 0.3}; 
    std::vector<double> limitedLimitsVector = {0.5, 1.0};

    //for Flexible State
    if (targetStates.tState1 == MarkovStates::Dynamic)
    {
        target.insert(std::pair<MarkovStates, std::vector<double>>(MarkovStates::Dynamic, dynamicLimitsVector));
        target.insert(std::pair<MarkovStates, std::vector<double>>(MarkovStates::Limited, limitedLimitsVector));        
    }

    //for Dynamic State#include <algorithm>
    if (targetStates.tState1 == MarkovStates::Limited)
    {
        target.insert(std::pair<MarkovStates, std::vector<double>>(MarkovStates::Limited, limitedLimitsVector));
        target.insert(std::pair<MarkovStates, std::vector<double>>(MarkovStates::Flexible, flexibleLimitsVector));        
    }

    //for Limited State
    if (targetStates.tState1 == MarkovStates::Flexible)
    {
        target.insert(std::pair<MarkovStates, std::vector<double>>(MarkovStates::Flexible, flexibleLimitsVector));
        target.insert(std::pair<MarkovStates, std::vector<double>>(MarkovStates::Dynamic, dynamicLimitsVector));        
    }    

    return target;
}

VamPriority::targetStates VamPriority::getTargetStates(double cbr) {

    VamPriority::targetStates target;
    //actualState = initialMarkovState;     
    
    //retornar um mapa com os limites dos dois targets
    if (cbr < 0.30 || cbr >= 0.00) { //Flexible
        target.tState1 = MarkovStates::Dynamic;
        target.tState2 = MarkovStates::Limited;                        
    } else if (cbr >= 0.3 && cbr < 0.5) { //Dynamic
        target.tState1 = MarkovStates::Limited;
        target.tState2 = MarkovStates::Flexible;        
    } else if (cbr >= 0.5) { //Limited
        target.tState1 = MarkovStates::Flexible;
        target.tState2 = MarkovStates::Dynamic;        
    }
    
    return target;
}

std::map<VamPriority::MarkovStates,double> VamPriority::probabilityAnalysis(std::map<LocalDynamicMap::StationID, std::vector<double>> poissonDist, double cbr, std::map<VamPriority::MarkovStates, std::vector<double>> targetStates) {
    
    //Numeric Variables
    double avgMsgSize = 600; //Bytes
    double targetCBR = 0.68;
    double channelCapacity = 6 * 1000000;  // 1 Mbit = 1000000 bits

    //Evaluating Available Channel Resource
    double localCBR = cbr;    

    std::map<VamPriority::MarkovStates, double> targetStateMapBorders;    

    double dynamicLimit = 0.3;
    double flexibleLimit = 0.29; 
    double limitedLimit = 0.5;

    std::vector<double> poissonDistTotal;
    std::vector<double> poissonDistNorm;

    std::vector<double> poissonProb_0;
    std::vector<double> poissonProb_1;
    std::vector<double> poissonProb_2;
    std::vector<double> poissonProb_3;
    std::vector<double> poissonProb_4;
    std::vector<double> poissonProb_5;
    std::vector<double> poissonProb_6;
    std::vector<double> poissonProb_7;
    std::vector<double> poissonProb_8;
    std::vector<double> poissonProb_9;
    std::vector<double> poissonProb_10;

    //calculate the normalized poisson distribution for this time
    for (const auto& it : poissonDist){        
        poissonProb_0.push_back(it.second[0]);
        poissonProb_1.push_back(it.second[1]);
        poissonProb_2.push_back(it.second[2]);
        poissonProb_3.push_back(it.second[3]);
        poissonProb_4.push_back(it.second[4]);
        poissonProb_5.push_back(it.second[5]);
        poissonProb_6.push_back(it.second[6]);
        poissonProb_7.push_back(it.second[7]);
        poissonProb_8.push_back(it.second[8]);
        poissonProb_9.push_back(it.second[9]);
        poissonProb_10.push_back(it.second[10]);
    }    

    poissonDistTotal.push_back(std::accumulate(poissonProb_0.begin(), poissonProb_0.end(), 0.0f));
    poissonDistTotal.push_back(std::accumulate(poissonProb_1.begin(), poissonProb_1.end(), 0.0f));
    poissonDistTotal.push_back(std::accumulate(poissonProb_2.begin(), poissonProb_2.end(), 0.0f));
    poissonDistTotal.push_back(std::accumulate(poissonProb_3.begin(), poissonProb_3.end(), 0.0f));
    poissonDistTotal.push_back(std::accumulate(poissonProb_4.begin(), poissonProb_4.end(), 0.0f));
    poissonDistTotal.push_back(std::accumulate(poissonProb_5.begin(), poissonProb_5.end(), 0.0f));
    poissonDistTotal.push_back(std::accumulate(poissonProb_6.begin(), poissonProb_6.end(), 0.0f));
    poissonDistTotal.push_back(std::accumulate(poissonProb_7.begin(), poissonProb_7.end(), 0.0f));
    poissonDistTotal.push_back(std::accumulate(poissonProb_8.begin(), poissonProb_8.end(), 0.0f));
    poissonDistTotal.push_back(std::accumulate(poissonProb_9.begin(), poissonProb_9.end(), 0.0f));
    poissonDistTotal.push_back(std::accumulate(poissonProb_10.begin(), poissonProb_10.end(), 0.0f));

    double normFactor = std::accumulate(poissonDistTotal.begin(), poissonDistTotal.end(), 0.0f);

    for (auto element : poissonDistTotal){
        poissonDistNorm.push_back(element/normFactor);
    }

    for (const auto& itr : targetStates) {        
        if (itr.first == MarkovStates::Flexible) {
            targetStateMapBorders.insert(std::pair<MarkovStates, double>(MarkovStates::Flexible, flexibleLimit));
        } else if (itr.first == MarkovStates::Dynamic) {
            targetStateMapBorders.insert(std::pair<MarkovStates, double>(MarkovStates::Dynamic, dynamicLimit));
        } else if (itr.first == MarkovStates::Limited) {
            targetStateMapBorders.insert(std::pair<MarkovStates, double>(MarkovStates::Limited, limitedLimit));
        }
    }
    
    std::map<VamPriority::MarkovStates, int> nbrOfMsgLimitState;
    std::map<VamPriority::MarkovStates,double> MarkovProbabilityMap;

    for (auto element : targetStateMapBorders) {
        double limitCBR = element.second;
        double Resource2BoundBoxRate = limitCBR - localCBR; //Rate
        double Resource2BoundBoxBits = Resource2BoundBoxRate*channelCapacity;//std::max((Resource2BoundBoxRate)*channelCapacity, 0.0);        

        std::map<int, double> poissonLimit; 

        std::vector<double> requireBits;

        for (int k=0; k <=10; ++k){
            requireBits.push_back(k*poissonDist.size()*bytesToBits(avgMsgSize));            
        }
        
        if (Resource2BoundBoxBits > 0) {
            double newConsumeBandwidthBits = 0.0;
            int k = 0;
                        
            for (auto it : requireBits) {                                           
                if (it >= Resource2BoundBoxBits) {
                    poissonLimit.insert(std::pair<int, double>(k, newConsumeBandwidthBits));
                } else {
                    k++;
                }                            
            }            
        } else if (Resource2BoundBoxBits < 0){
            double newConsumeBandwidthBits = 0.0;
            int k = 0;

            for (auto it : requireBits){
                if (std::abs(it) >= std::abs(Resource2BoundBoxBits)){
                    poissonLimit.insert(std::pair<int, double>(k, newConsumeBandwidthBits));
                } else {k++;}
            }
        }  
        
        //get first element from the map
        if (poissonLimit.begin()->first != 0){
            nbrOfMsgLimitState.insert(std::pair<VamPriority::MarkovStates,int>(element.first, poissonLimit.begin()->first));
        }
    }    
    
    std::vector<int> lowerStepVector;
    std::vector<int> higherStepVector;
    int lowerStep, higherStep;    

    std::map<VamPriority::MarkovStates, std::vector<int>> stagesStepsMap;

    auto minElementIterator = std::min_element(nbrOfMsgLimitState.begin(), nbrOfMsgLimitState.end(),[](const std::pair<const VamPriority::MarkovStates, int>& a,
                            const std::pair<const VamPriority::MarkovStates, int>& b) {return a.second < b.second;});

    auto maxElementIterator = std::max_element(nbrOfMsgLimitState.begin(), nbrOfMsgLimitState.end(),[](const std::pair<const VamPriority::MarkovStates, int>& a, 
                            const std::pair<const VamPriority::MarkovStates, int>& b) {return a.second < b.second;});

    int minElementValue, maxElementValue;
    
    //avoid Map with no elements
    if (nbrOfMsgLimitState.size() > 0) { 
        if (minElementIterator != nbrOfMsgLimitState.end()) {
            lowerStep = minElementIterator->second;
        }
        if (maxElementIterator != nbrOfMsgLimitState.end()) {
            higherStep = maxElementIterator->second;
        }

        int diffSteps = std::abs(higherStep - lowerStep);        

        if (diffSteps > 1) {       
            for (auto it = lowerStep; it < higherStep; ++it){
                lowerStepVector.push_back(it);
            }
            for (auto it = higherStep; it <= 10; ++it){
                higherStepVector.push_back(it);
            }
        } else if (diffSteps == 1){
            lowerStepVector.push_back(lowerStep);
            for (auto it = higherStep; it <= 10; ++it){
                higherStepVector.push_back(it);
            }
        } else if (diffSteps == 0) {
            for (auto it = higherStep; it <= 10; ++it){
                lowerStepVector.push_back(it);
                higherStepVector.push_back(it);
            }
        }


        //primeiro trabalhar com os valores cumulativos
        double lowStepProb, highStepProb;

        for (auto itr_lo : lowerStepVector){
            lowStepProb += poissonDistNorm[itr_lo];            
        }

        for (auto itr_hi : higherStepVector){
            highStepProb += poissonDistNorm[itr_hi];            
        }

        //estrutura para criar Markov Probability
        

        if (minElementIterator != nbrOfMsgLimitState.end()) {
            MarkovProbabilityMap.insert(std::pair<VamPriority::MarkovStates,double>(minElementIterator->first, lowStepProb));
        }

        if (maxElementIterator != nbrOfMsgLimitState.end()) {
            MarkovProbabilityMap.insert(std::pair<VamPriority::MarkovStates,double>(maxElementIterator->first, highStepProb));
        }
    }
    
    auto markovChain = markovChainDefinition (MarkovProbabilityMap, cbr);

    return markovChain;
    
}

std::map<VamPriority::MarkovStates,double> VamPriority::markovChainDefinition(std::map<VamPriority::MarkovStates,double> markovChainTargets, double cbr){

    auto targetStates = markovChainTargets;

    VamPriority::MarkovStates currentState;
    double actualCBR = cbr;

    if (actualCBR < 0.30 || actualCBR >= 0.00) {
        currentState = MarkovStates::Flexible;
    } else if (cbr >= 0.3 && cbr < 0.5) { 
        currentState = MarkovStates::Dynamic;                
    } else if (cbr >= 0.5) { 
        currentState = MarkovStates::Limited;
    }

    //absorbing State
    if (targetStates.size() == 0){        
        std::map<VamPriority::MarkovStates,double> actualState;
        actualState.insert(std::pair<VamPriority::MarkovStates,double>(currentState, 1.00));
        return actualState;
    } 

    else if (targetStates.size() == 1){
        std::map<VamPriority::MarkovStates,double> targetState;
        double currentStateEngorvicProb;
        
        for (const auto it : markovChainTargets){
            targetState.insert(std::pair<VamPriority::MarkovStates,double>(it.first, it.second)); 
            currentStateEngorvicProb = 1 - it.second;
        }
        targetState.insert(std::pair<VamPriority::MarkovStates,double>(currentState, currentStateEngorvicProb));
        return targetState;
    }
    
    else if (targetStates.size() == 2) {
        std::vector<VamPriority::MarkovStates> MarkovStates {MarkovStates::Flexible, MarkovStates::Dynamic, MarkovStates::Limited};
        std::vector<VamPriority::MarkovStates> targetStatesVector;
        std::vector<VamPriority::MarkovStates> actualState;
        std::map<VamPriority::MarkovStates,double> markovChain;
        double actualStateProbability, targetStateProbability;

        for (const auto itr : targetStates){
            targetStatesVector.push_back(itr.first);
            targetStateProbability += itr.second;
        }

        actualStateProbability = 1.00 - targetStateProbability;
        
        //find the actual state in correlation to others
        std::set_difference(MarkovStates.begin(), MarkovStates.end(), targetStatesVector.begin(), targetStatesVector.end(),std::back_inserter(actualState));        

        //std::cout << "target Prob: " << targetStateProbability << std::endl;
        //std::cout << "actual Prob: " << actualStateProbability << std::endl;
        //std::cout << "Summ: " << targetStateProbability + actualStateProbability << std::endl;        

        for (const auto itr : targetStates){
            markovChain.insert(std::pair<VamPriority::MarkovStates,double>(itr.first, itr.second));
        }
        
        for (auto itr : actualState){
            markovChain.insert(std::pair<VamPriority::MarkovStates,double>(itr, actualStateProbability));
        }
        
        for (const auto it : markovChain){
            ;//std::cout << "state: " << it.first << " - prob.: " << it.second << std::endl;
        }

        //std::cout << "------------" << std::endl;

        return markovChain;
    }    
}

}
