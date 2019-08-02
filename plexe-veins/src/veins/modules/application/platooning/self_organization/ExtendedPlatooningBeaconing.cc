//
// Copyright (c) 2012-2018 Michele Segata <segata@ccs-labs.org>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#include "../self_organization/ExtendedPlatooningBeaconing.h"

Define_Module(ExtendedPlatooningBeaconing)

void ExtendedPlatooningBeaconing::initialize(int stage)
{
    BaseProtocol::initialize(stage);

    if (stage == 0) {
        // random start time
        SimTime beginTime = SimTime(uniform(0.001, beaconingInterval));
        if (beaconingInterval > 0) scheduleAt(simTime() + beaconingInterval + beginTime, sendBeacon);
    }
}

void ExtendedPlatooningBeaconing::sendPlatooningMessage(int destinationAddress)
{
    // vehicle's data to be included in the message
    Plexe::VEHICLE_DATA data;
    // get information about the vehicle via traci
    traciVehicle->getVehicleData(&data);

    // create and send beacon
    UnicastMessage* unicast = new UnicastMessage("", BEACON_TYPE);
//    UnicastMessage* unicast = new UnicastMessage("", EXTENDED_BEACON);
    unicast->setDestination(-1);
    unicast->setPriority(priority);
    unicast->setChannel(Channels::CCH);

    // create platooning beacon with data about the car
    ExtendedPlatooningBeacon* pkt = new ExtendedPlatooningBeacon();
    pkt->setControllerAcceleration(data.u);
    pkt->setAcceleration(data.acceleration);
    pkt->setSpeed(data.speed);
    pkt->setVehicleId(myId);
    pkt->setPositionX(data.positionX);
    pkt->setPositionY(data.positionY);
    // set the time to now
    pkt->setTime(data.time);
    pkt->setLength(length);
    pkt->setSpeedX(data.speedX);
    pkt->setSpeedY(data.speedY);
    pkt->setAngle(data.angle);
    // i generated the message, i send it
    pkt->setRelayerId(myId);
    pkt->setKind(BEACON_TYPE);
//    pkt->setKind(EXTENDED_BEACON);
    pkt->setByteLength(packetSize);
    pkt->setSequenceNumber(seq_n++);

    // Set Lane Index
    pkt->setLaneIndex(traciVehicle->getLaneIndex());

    // put platooning beacon into the message for the UnicastProtocol
    unicast->encapsulate(pkt);
    sendDown(unicast);
}

void ExtendedPlatooningBeaconing::handleSelfMsg(cMessage* msg)
{

    BaseProtocol::handleSelfMsg(msg);

    if (msg == sendBeacon) {
        sendPlatooningMessage(-1);
        scheduleAt(simTime() + beaconingInterval, sendBeacon);
    }
}

void ExtendedPlatooningBeaconing::messageReceived(PlatooningBeacon* pkt, UnicastMessage* unicast)
{
    // nothing to do for static beaconing
}

//void ExtendedPlatooningBeaconing::startBeaconing()
//{
//    scheduleAt(simTime(), sendBeacon);
//}
//
//void ExtendedPlatooningBeaconing::stopBeaconing()
//{
//    cancelEvent(sendBeacon);
//}

