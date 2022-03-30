#ifndef CONTROLLABLEPERSON_H
#define CONTROLLABLEPERSOM_H

#include "artery/traci/PersonController.h"

class ControllablePerson
{
public:
    virtual traci::PersonController* getPersonController() = 0;
    virtual ~ControllablePerson() = default;
};

#endif /* CONTROLLABLEPERSON_H */