#include "artery/application/LocalDynamicMap.h"
#include "artery/application/Timer.h"
#include <omnetpp/csimulation.h>
#include <cassert>
#include <algorithm>

namespace artery
{

LocalDynamicMap::LocalDynamicMap(const Timer& timer) :
    mTimer(timer)
{
}

void LocalDynamicMap::updateAwareness(const VaObject& obj)
{
    const vanetza::asn1::Vam& msg = obj.asn1();

    static const omnetpp::SimTime lifetime { 1100, omnetpp::SIMTIME_MS };
    auto tai = mTimer.reconstructMilliseconds(msg->vam.generationDeltaTime);
    const omnetpp::SimTime expiry = mTimer.getTimeFor(tai) + lifetime;

    const auto now = omnetpp::simTime();
    if (expiry < now || expiry > now + 2 * lifetime) {
        EV_STATICCONTEXT
        EV_WARN << "Expiry of received VAM is out of bounds";
        return;
    }

    AwarenessEntry entry(obj, expiry);
    auto found = mVaMessages.find(msg->header.stationID);
    if (found != mVaMessages.end()) {
        found->second = std::move(entry);
    } else {
        mVaMessages.emplace(msg->header.stationID, std::move(entry));
    }
}

void LocalDynamicMap::dropExpired()
{
    const auto now = omnetpp::simTime();
    for (auto it = mVaMessages.begin(); it != mVaMessages.end();) {
        if (it->second.expiry < now) {
            it = mVaMessages.erase(it);
        } else {
            ++it;
        }
    }
}

unsigned LocalDynamicMap::count(const VamPredicate& predicate) const
{    
    return std::count_if(mVaMessages.begin(), mVaMessages.end(),
            [&predicate](const std::pair<const StationID, AwarenessEntry>& map_entry) {
                const Vam& vam = map_entry.second.object.asn1();
                return predicate(vam);
            });
}

LocalDynamicMap::AwarenessEntry::AwarenessEntry(const VaObject& obj, omnetpp::SimTime t) :
    expiry(t), object(obj)
{
}

} // namespace artery
