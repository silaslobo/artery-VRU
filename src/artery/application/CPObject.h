#ifndef ARTERY_CPOBJECT_H_
#define ARTERY_CPOBJECT_H_

#include <omnetpp/cobject.h>
#include <artery/cpm/cpm.hpp>
#include <memory>

namespace artery
{

class CPObject : public omnetpp::cObject
{
public:
	CPObject(const CPObject&) = default;
    CPObject& operator=(const CPObject&) = default;
    
	CPObject(artery::cpm::Cpm&&);
	CPObject& operator=(artery::cpm::Cpm&&);
	
    CPObject(const artery::cpm::Cpm&);
    CPObject& operator=(const artery::cpm::Cpm&);

	CPObject(const std::shared_ptr<const artery::cpm::Cpm>&);
    CPObject& operator=(const std::shared_ptr<const artery::cpm::Cpm>&);
	
    const artery::cpm::Cpm& asn1() const;

    std::shared_ptr<const artery::cpm::Cpm> shared_ptr() const;
	
private:
	std::shared_ptr<const artery::cpm::Cpm> m_cpm_wrapper;

};

} // namespace artery

#endif /* ARTERY_CPOBJECT_H_ */
