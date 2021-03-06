//
// Copyright (C) 2015 OpenSim Ltd.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//
// Author: Benjamin Seregi
//

#include "MpduGen.h"

#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/transportlayer/contract/udp/UDPControlInfo_m.h"

namespace inet {

Define_Module(MpduGen);

simsignal_t MpduGen::sentPkSignal = registerSignal("sentPk");
simsignal_t MpduGen::rcvdPkSignal = registerSignal("rcvdPk");

void MpduGen::initialize(int stage) {
    ApplicationBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        localPort = par("localPort");
        destPort = par("destPort");
    }
    else if (stage == INITSTAGE_LAST) {
        selfMsg = new cMessage("Self msg");
        scheduleAt(simTime() + par("startTime").doubleValue(), selfMsg);
    }
}

void MpduGen::processPacket(cPacket *pk) {
    emit(rcvdPkSignal, pk);
    EV_INFO << "Received packet: " << UDPSocket::getReceivedPacketInfo(pk) << endl;
    delete pk;
    numReceived++;
}

void MpduGen::sendPackets() {
    socket.setOutputGate(gate("udpOut"));
    const char *localAddress = par("localAddress");
    socket.bind(*localAddress ? L3AddressResolver().resolve(localAddress) : L3Address(), localPort);
    const char *destAddrStr = par("destAddress");
    const char *packets = par("packets");
    L3Address destAddr = L3AddressResolver().resolve(destAddrStr);
    int len = strlen(packets);
    const char *packetName = par("packetName");
    for (int i = 0; i < len; i++) {
        std::ostringstream str;
        str << packetName << "-" << i;
        cPacket *payload = new cPacket(str.str().c_str());
        if (packets[i] == 'L') {
            payload->setByteLength(par("longPacketSize"));
        }
        else if (packets[i] == 'S') {
            payload->setByteLength(par("shortPacketSize"));
        }
        else
            throw cRuntimeError("Unknown packet type = %c", packets[i]);
        emit(sentPkSignal, payload);
        socket.sendTo(payload, destAddr, destPort);
        numSent++;
    }
    socket.close();
}

void MpduGen::handleMessageWhenUp(cMessage *msg) {
    if (msg->isSelfMessage()) {
        ASSERT(msg == selfMsg);
        sendPackets();
    }
    else if (msg->getKind() == UDP_I_DATA) {
        processPacket(PK(msg));
    }
    else if (msg->getKind() == UDP_I_ERROR) {
        EV_WARN << "Ignoring UDP error report\n";
        delete msg;
    }
    else {
        throw cRuntimeError("Unrecognized message (%s)%s", msg->getClassName(), msg->getName());
    }
    if (hasGUI()) {
        char buf[40];
        sprintf(buf, "rcvd: %d pks\nsent: %d pks", numReceived, numSent);
        getDisplayString().setTagArg("t", 0, buf);
    }
}

}
