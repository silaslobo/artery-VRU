#include <artery/application/VaObject.h>
#include <omnetpp.h>
#include <cassert>

namespace artery
{

using namespace vanetza::asn1;

Register_Abstract_Class(VaObject)

VaObject::VaObject(Vam&& vam) :
    m_vam_wrapper(std::make_shared<Vam>(std::move(vam)))
{
}

VaObject& VaObject::operator=(Vam&& vam)
{
    m_vam_wrapper = std::make_shared<Vam>(std::move(vam));
    return *this;
}

VaObject::VaObject(const Vam& vam) :
    m_vam_wrapper(std::make_shared<Vam>(vam))
{
}

VaObject& VaObject::operator=(const Vam& vam)
{
    m_vam_wrapper = std::make_shared<Vam>(vam);
    return *this;
}

VaObject::VaObject(const std::shared_ptr<const Vam>& ptr) :
    m_vam_wrapper(ptr)
{
    assert(m_vam_wrapper);
}

VaObject& VaObject::operator=(const std::shared_ptr<const Vam>& ptr)
{
    m_vam_wrapper = ptr;
    assert(m_vam_wrapper);
    return *this;
}

std::shared_ptr<const Vam> VaObject::shared_ptr() const
{
    assert(m_vam_wrapper);
    return m_vam_wrapper;
}

const vanetza::asn1::Vam& VaObject::asn1() const
{
    return *m_vam_wrapper;
}

omnetpp::cObject* VaObject::dup() const
{
    return new VaObject { *this };
}

using namespace omnetpp;

class VamStationIdResultFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto vam = dynamic_cast<VaObject*>(object)) {
            const auto id = vam->asn1()->header.stationID;
            fire(this, t, id, details);
        }
    }
};

Register_ResultFilter("vamStationId", VamStationIdResultFilter)


class VamGenerationDeltaTimeResultFilter : public cObjectResultFilter
{
protected:
    void receiveSignal(cResultFilter* prev, simtime_t_cref t, cObject* object, cObject* details) override
    {
        if (auto vam = dynamic_cast<VaObject*>(object)) {
            const auto genDeltaTime = vam->asn1()->vam.generationDeltaTime;
            fire(this, t, genDeltaTime, details);
        }
    }
};

Register_ResultFilter("vamGenerationDeltaTime", VamGenerationDeltaTimeResultFilter)

} // namespace artery
