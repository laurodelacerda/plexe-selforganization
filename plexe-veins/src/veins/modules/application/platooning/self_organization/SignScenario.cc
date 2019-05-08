//
// Copyright (C) 2018 Julian Heinovski <julian.heinovski@ccs-labs.org>
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

#include "../self_organization/SignScenario.h"

using namespace Veins;

Define_Module(SignScenario);

const simsignalwrap_t SignScenario::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);

void SignScenario::initialize(int stage)
{

    BaseScenario::initialize(stage);

    lastSignId = "";

    if (stage == 1) {
        app = FindModule<SelfOrganizationApp*>::findSubModule(getParentModule());
//        prepareManeuverCars(0);

        findHost()->subscribe(mobilityStateChangedSignal, this);
    }
}

SignScenario::~SignScenario()
{
    findHost()->unsubscribe(mobilityStateChangedSignal, this);
//    startManeuver = nullptr;
}

void SignScenario::setupFormation()
{
    std::vector<int> formation;
    for (int i = 0; i < 4; i++) formation.push_back(i);
    positionHelper->setPlatoonFormation(formation);
}

void SignScenario::prepareManeuverCars(int platoonLane)
{

    switch (positionHelper->getId()) {

    case 0: {
        // this is the leader
        traciVehicle->setCruiseControlDesiredSpeed(100.0 / 3.6);
        traciVehicle->setActiveController(Plexe::ACC);
        traciVehicle->setFixedLane(platoonLane);

        positionHelper->setIsLeader(true);
        positionHelper->setPlatoonLane(platoonLane);
        positionHelper->setPlatoonSpeed(100 / 3.6);
        positionHelper->setPlatoonId(positionHelper->getId());
        setupFormation();

        break;
    }

    case 1:
    case 2:
    case 3: {
        // these are the followers which are already in the platoon
        traciVehicle->setCruiseControlDesiredSpeed(130.0 / 3.6);
        traciVehicle->setActiveController(Plexe::CACC);
        traciVehicle->setFixedLane(platoonLane);

        positionHelper->setIsLeader(false);
        positionHelper->setPlatoonLane(platoonLane);
        positionHelper->setPlatoonSpeed(100 / 3.6);
        positionHelper->setPlatoonId(positionHelper->getLeaderId());
        setupFormation();

        break;
    }

    case 4: {
        // this is the car which will join
        traciVehicle->setCruiseControlDesiredSpeed(100 / 3.6);
        traciVehicle->setFixedLane(2);
        traciVehicle->setActiveController(Plexe::ACC);

        positionHelper->setPlatoonId(-1);
        positionHelper->setIsLeader(false);
        positionHelper->setPlatoonLane(-1);

        // after 10 seconds of simulation, start the maneuver
//        startManeuver = new cMessage();
//        scheduleAt(simTime() + SimTime(10), startManeuver);

        break;
    }
    }
}

void SignScenario::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details)
{
    std::string signId;
    std::string shape;

    Enter_Method_Silent();

    /// TODO Maybe emit a signal when a road sign is perceived
    if (signalID == mobilityStateChangedSignal)
    {
//        traciVehicle->getCustomParameter("has.signDetector.device", device);
//        traciVehicle->getCustomParameter("device.signDetector.lastRoadSignLane", lane);
//        traciVehicle->getCustomParameter("device.signDetector.lastRoadSignShape", shape);
        traciVehicle->getCustomParameter("device.signDetector.lastRoadSignId", signId);


        if (lastSignId != signId)
        {
            lastSignId = signId;
            app->onRoadSignDetection(shape);
        }
    }
}

void SignScenario::handleSelfMsg(cMessage* msg)
{
    // this takes car of feeding data into CACC and reschedule the self message
    BaseScenario::handleSelfMsg(msg);
}

