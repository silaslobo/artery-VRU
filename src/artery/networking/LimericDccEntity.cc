#include "artery/networking/LimericDccEntity.h"
#include "artery/utility/PointerCheck.h"

namespace artery
{

using namespace omnetpp;
static const simsignal_t SignalPacketsTransmitted = cComponent::registerSignal("packetsTransmitted");
static const simsignal_t SignalPacketsTransmittedDP2 = cComponent::registerSignal("packetsTransmittedDP2");
static const simsignal_t SignalPacketsTransmittedDP3 = cComponent::registerSignal("packetsTransmittedDP3");
static const simsignal_t SignalPacketsDropped = cComponent::registerSignal("packetsDropped");
static const simsignal_t SignalPacketsDroppedDP2 = cComponent::registerSignal("packetsDroppedDP2");
static const simsignal_t SignalPacketsDroppedDP3 = cComponent::registerSignal("packetsDroppedDP3");

Define_Module(LimericDccEntity)

void LimericDccEntity::finish()
{
    // free those objects before runtime vanishes
    mTransmitRateControl.reset();
    mAlgorithm.reset();
    DccEntityBase::finish();
}

vanetza::dcc::TransmitRateThrottle* LimericDccEntity::getTransmitRateThrottle()
{
    unsigned long transmittedPackets = DccEntityBase::mFlowControl->mDccTransmittedPackets;
    unsigned long transmittedPacketsDP2 = DccEntityBase::mFlowControl->mDccDP2TransmittedPackets;
    unsigned long transmittedPacketsDP3 = DccEntityBase::mFlowControl->mDccDP3TransmittedPackets;
    emit(SignalPacketsTransmitted, transmittedPackets);
    emit(SignalPacketsTransmittedDP2, transmittedPacketsDP2);
    emit(SignalPacketsTransmittedDP3, transmittedPacketsDP3);

    unsigned long droppedPackets = DccEntityBase::mFlowControl->mDccDroppedPackets;
    unsigned long droppedPacketsDP2 = DccEntityBase::mFlowControl->mDccDP2DroppedPackets;
    unsigned long droppedPacketsDP3 = DccEntityBase::mFlowControl->mDccDP3DroppedPackets;
    emit(SignalPacketsDropped, droppedPackets);
    emit(SignalPacketsDroppedDP2, droppedPacketsDP2);
    emit(SignalPacketsDroppedDP3, droppedPacketsDP3);

    return notNullPtr(mTransmitRateControl);
}

vanetza::dcc::TransmitRateControl* LimericDccEntity::getTransmitRateControl()
{
    return notNullPtr(mTransmitRateControl);
}

void LimericDccEntity::initializeTransmitRateControl()
{
    ASSERT(mRuntime);
    using namespace vanetza::dcc;

    Limeric::Parameters params;
    params.cbr_target = mTargetCbr;

    mAlgorithm.reset(new Limeric(*mRuntime, params));
    if (par("enableDualAlpha")) {
        Limeric::DualAlphaParameters dual_params;
        mAlgorithm->configure_dual_alpha(dual_params);
    }
    mTransmitRateControl.reset(new LimericTransmitRateControl(*mRuntime, *mAlgorithm));

    mAlgorithm->on_duty_cycle_change = [this](const Limeric*, vanetza::Clock::time_point) {
        mTransmitRateControl->update();
    };
}

void LimericDccEntity::onGlobalCbr(vanetza::dcc::ChannelLoad cbr)
{
    ASSERT(mAlgorithm);
    mAlgorithm->update_cbr(cbr);
}

} // namespace arterd
