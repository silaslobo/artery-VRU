#ifndef ARTERY_VAOBJECT_H_
#define ARTERY_VAOBJECT_H_

#include <omnetpp/cobject.h>
#include <vanetza/asn1/vam.hpp>
#include <memory>

namespace artery
{

class VaObject : public omnetpp::cObject
{
public:
    VaObject(const VaObject&) = default;
    VaObject& operator=(const VaObject&) = default;

    VaObject(vanetza::asn1::Vam&&);
    VaObject& operator=(vanetza::asn1::Vam&&);

    VaObject(const vanetza::asn1::Vam&);
    VaObject& operator=(const vanetza::asn1::Vam&);

    VaObject(const std::shared_ptr<const vanetza::asn1::Vam>&);
    VaObject& operator=(const std::shared_ptr<const vanetza::asn1::Vam>&);

    const vanetza::asn1::Vam& asn1() const;

    std::shared_ptr<const vanetza::asn1::Vam> shared_ptr() const;

    omnetpp::cObject* dup() const override;

private:
    std::shared_ptr<const vanetza::asn1::Vam> m_vam_wrapper;
};

} // namespace artery

#endif /* ARTERY_VAOBJECT_H_ */