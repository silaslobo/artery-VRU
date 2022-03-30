//
// Created by rosk on 29.07.20.
//

#ifndef ARTERY_DCCFACILITY_H
#define ARTERY_DCCFACILITY_H

#include "artery/application/ItsG5BaseService.h"
#include "artery/utility/Channel.h"
#include "artery/application/Constants.hpp"
#include "artery/application/Middleware.h"

#include "artery/utility/Geometry.h"
#include <vanetza/asn1/cam.hpp>
#include <vanetza/btp/data_interface.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/common/byte_buffer_convertible.hpp>
#include <vanetza/units/velocity.hpp>
#include <omnetpp/simtime.h>
#include <omnetpp/cexception.h>


namespace artery {

class DCCFacility :
        public omnetpp::cSimpleModule,
        public omnetpp::cListener
{
    public:
        DCCFacility();
        ~DCCFacility();

        typedef struct infoFlow{
            u_int8_t id;
        } InfoFlow;

        /*class FlowParameters: public vanetza::ByteBufferConvertible{
        public:
            FlowParameters(InfoFlow flow): vanetza::ByteBufferConvertible(flow), mFlowId(3){  }
            virtual ~FlowParameters(){}
            size_t getId() { return this->mFlowId; }

        private:
            size_t mFlowId;

        }; */


    protected:
        void initialize() override;
        void finish() override;
        void subscribe(const omnetpp::simsignal_t&);
        void unsubscribe(const omnetpp::simsignal_t&);

        // cListener
        void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t, omnetpp::cObject*, omnetpp::cObject*) override;


    private:
        Middleware* mMiddleware;


};

} //End namespace Artery

#endif //ARTERY_DCCFACILITY_H
