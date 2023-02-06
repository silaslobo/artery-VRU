#ifndef ARTERY_LOCALDYNAMICMAP_H_AL7SS9KT
#define ARTERY_LOCALDYNAMICMAP_H_AL7SS9KT

#include "artery/application/VaObject.h"
#include <omnetpp/simtime.h>
#include <vanetza/asn1/vam.hpp>
#include <cstdint>
#include <functional>
#include <map>

namespace artery
{

class Timer;

class LocalDynamicMap
{    
public:
    using StationID = uint32_t;
    using Vam = vanetza::asn1::Vam;
    using VamPredicate = std::function<bool(const Vam&)>;

    struct AwarenessEntry
        {
            AwarenessEntry(const VaObject&, omnetpp::SimTime);
            AwarenessEntry(AwarenessEntry&&) = default;
            AwarenessEntry& operator=(AwarenessEntry&&) = default;

            omnetpp::SimTime expiry;
            VaObject object;
        };
    

    LocalDynamicMap(const Timer&);
    void updateAwareness(const VaObject&);
    void dropExpired();
    unsigned count(const VamPredicate&) const;
    std::map<StationID, LocalDynamicMap::AwarenessEntry>* localMap() {return &mVaMessages;};

private:    
    
    std::map<StationID, AwarenessEntry> mVaMessages;    
    const Timer& mTimer;        
};

} // namespace artery

#endif /* ARTERY_LOCALDYNAMICMAP_H_AL7SS9KT */

