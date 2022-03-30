#ifndef REGIONOFINTERESTPERSONPOLICY_H_TNK4CWW6
#define REGIONOFINTERESTPERSONPOLICY_H_TNK4CWW6

#include "traci/RegionsOfInterest.h"
#include "traci/PersonPolicy.h"
#include <unordered_set>
#include <omnetpp/clistener.h>

namespace traci
{

class SubscriptionManager;

class RegionOfInterestPersonPolicy : public PersonPolicy, public omnetpp::cListener
{
public:
    void initialize(PersonLifecycle*) override;
    Decision addPerson(const std::string& id) override;
    Decision updatePerson(const std::string& id) override;
    Decision removePerson(const std::string& id) override;

protected:
    void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, unsigned long n, omnetpp::cObject*) override;

private:
    void checkRegionOfInterest();

    SubscriptionManager* m_subscriptions;
    PersonLifecycle* m_lifecycle;
    RegionsOfInterest m_regions;
    std::unordered_set<std::string> m_outside;
};

} // namespace traci

#endif /* REGIONOFINTERESTPERSONPOLICY_H_TNK4CWW6 */

