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

#include "SelfOrganizationApp.h"

using namespace Veins;

Define_Module(SelfOrganizationApp);


SelfOrganizationApp::SelfOrganizationApp()
{
    telemetryMap = new TelemetryMap(this);
    inDanger = false;
}


void SelfOrganizationApp::initialize(int stage)
{
    GeneralPlatooningApp::initialize(stage);

    if (stage == 1)
    {
        SimTime rounded = SimTime(floor(simTime().dbl() * 1000 + 100), SIMTIME_MS);

        topologyCheck = new cMessage("topologyCheck");
        scheduleAt(rounded, topologyCheck);

        positionUpdateMsg = new cMessage("positionUpdateMsg");
        scheduleAt(rounded, positionUpdateMsg);

        safeJoinCheck = new cMessage("safeJoinCheck");
    }

}


SelfOrganizationApp::~SelfOrganizationApp()
{

    cancelAndDelete(topologyCheck);
    topologyCheck = nullptr;
    cancelAndDelete(positionUpdateMsg);
    positionUpdateMsg = nullptr;
    cancelAndDelete(safeJoinCheck);
    safeJoinCheck = nullptr;

}


void SelfOrganizationApp::handleSelfMsg(cMessage* msg)
{
    if (msg == positionUpdateMsg)
    {
        Plexe::VEHICLE_DATA data;
        traciVehicle->getVehicleData(&data);
        telemetryMap->updatePosition(myId, traciVehicle->getLaneIndex(), data);

        updateFlags();
        scheduleAt(simTime() + SimTime(100, SIMTIME_MS), positionUpdateMsg);
    }

    if (msg == topologyCheck)
    {
        printInfo();
        scheduleAt(simTime() + SimTime(100, SIMTIME_MS) + 1, topologyCheck);
    }


    if (msg == safeJoinCheck)
    {
        cancelEvent(safeJoinCheck);
        startManeuverFormation();
    }

}


void SelfOrganizationApp::onPlatoonBeacon(const PlatooningBeacon* pb)
{
    // TODO Check if message has Extended Beaconing Kind
    telemetryMap->updateRoadTopology(pb);

    // TODO Check if message is a maneuver to be developed

    // Send to lower layers, including maneuvers
    GeneralPlatooningApp::onPlatoonBeacon(pb);
}


void SelfOrganizationApp::onRoadSignDetection(std::string sign)
{

    std::string lane_id;
    traciVehicle->getCustomParameter("device.signDetector.lastRoadSignLane", lane_id);

    std::string lane_index_str;
    traciVehicle->getCustomParameter("device.signDetector.lastRoadSignLaneIndex", lane_index_str);
    int lane_index = std::atoi(lane_index_str.c_str());

    telemetryMap->addRoadSignDetected(sign, lane_index);

    updateFlags();
    startManeuverFormation();
}


void SelfOrganizationApp::updateFlags()
{

    bool lane_leader = telemetryMap->isLaneLeader(myId);
    bool lane_safe   = telemetryMap->isLaneSafe(myId);

    if ((lane_leader) && (lane_safe))
    {
        role = PlatoonRole::LEADER;
        inDanger = false;
        findHost()->getDisplayString().updateWith("r=8,green");
    }
    else if ((lane_leader) && (!lane_safe))
    {
        role = PlatoonRole::UNSAFE_LEADER;
        inDanger = true;
        findHost()->getDisplayString().updateWith("r=8,red");
    }
    else if ((!lane_leader) && (lane_safe))
    {
        role = PlatoonRole::FOLLOWER;
        inDanger = false;
        findHost()->getDisplayString().updateWith("r=8,blue");
    }
    else
    {
        role = PlatoonRole::UNSAFE_FOLLOWER;
        inDanger = true;
        findHost()->getDisplayString().updateWith("r=0,gray");
    }
}


void SelfOrganizationApp::setupFormation()
{
    std::vector<int> formation = telemetryMap->getFormation(traciVehicle->getLaneIndex());

    positionHelper->setPlatoonFormation(formation);
}


void SelfOrganizationApp::startManeuverFormation()
{
   // Enter_Method_Silent();

    switch(role)
    {
    case PlatoonRole::LEADER:
        startSafeLeaderFormation();
        break;

    case PlatoonRole::UNSAFE_LEADER:
        startUnsafeLeaderFormation();
        break;

    case PlatoonRole::FOLLOWER:
        startSafeFollowerFormation();
        break;

    case PlatoonRole::UNSAFE_FOLLOWER:
        startUnsafeFollowerFormation();
        break;

    default:
        break;
    }
}


void SelfOrganizationApp::startSafeLeaderFormation()
{
    traciVehicle->setCruiseControlDesiredSpeed(50.0 / 3.6);
    traciVehicle->setActiveController(Plexe::ACC);
    traciVehicle->setFixedLane(traciVehicle->getLaneIndex());

    positionHelper->setIsLeader(true);
    positionHelper->setPlatoonLane(traciVehicle->getLaneIndex());
    positionHelper->setPlatoonSpeed(50 / 3.6);
    positionHelper->setPlatoonId(positionHelper->getId());

    setupFormation();
}


void SelfOrganizationApp::startSafeFollowerFormation()
{
    traciVehicle->setCruiseControlDesiredSpeed(160.0 / 3.6);
    traciVehicle->setActiveController(Plexe::CACC);
    traciVehicle->setFixedLane(traciVehicle->getLaneIndex());
    traciVehicle->setCACCConstantSpacing(1);

    positionHelper->setIsLeader(false);
    positionHelper->setPlatoonLane(traciVehicle->getLaneIndex());
    positionHelper->setPlatoonSpeed(50 / 3.6);
    positionHelper->setPlatoonId(positionHelper->getLeaderId());

    setupFormation();
}


void SelfOrganizationApp::startUnsafeLeaderFormation()
{

//    role = PlatoonRole::UNSAFE_LEADER;
    setInManeuver(false);

    int safest_lane   = telemetryMap->getSafestLaneIndex();
    int safest_leader = telemetryMap->getSafestLaneLeader();
    bool lane_leader  = telemetryMap->isLaneLeader(myId);

    bool permission = telemetryMap->isSafeToMoveTo(PlatoonManeuver::JOIN_AT_BACK, safest_lane);
//    std::cout << "Permissão: " << permission << std::endl;

    if (permission && lane_leader && inDanger)
    {
        startJoinManeuver(safest_leader, safest_leader, -1);
    }
    else
    {
        Plexe::VEHICLE_DATA data;
        traciVehicle->getVehicleData(&data);
//        traciVehicle->slowDown((data.speed - 1) / 3.6, 0);
        traciVehicle->slowDown(30 / 3.6, 0);

        if (!safeJoinCheck->isScheduled())
            scheduleAt(simTime() + SimTime(500, SIMTIME_MS), safeJoinCheck);
    }
}


void SelfOrganizationApp::startUnsafeFollowerFormation()
{

//    setPlatoonRole(PlatoonRole::NONE);

//    traciVehicle->setCruiseControlDesiredSpeed(160.0 / 3.6);
//    traciVehicle->setActiveController(Plexe::CACC);
    traciVehicle->setFixedLane(traciVehicle->getLaneIndex());
//    traciVehicle->setCACCConstantSpacing(1);

    positionHelper->setIsLeader(false);
    positionHelper->setPlatoonLane(traciVehicle->getLaneIndex());
    positionHelper->setPlatoonSpeed(30 / 3.6);
    positionHelper->setPlatoonId(positionHelper->getLeaderId());

    bool lane_leader  = telemetryMap->isLaneLeader(myId);

    if (inDanger && !safeJoinCheck->isScheduled() && !lane_leader)
        scheduleAt(simTime() + SimTime(100, SIMTIME_MS), safeJoinCheck);

}


void SelfOrganizationApp::printInfo()
{
    std::string role_str ;

    if (getPlatoonRole() == PlatoonRole::LEADER)
        role_str = "LEADER";
    else if (getPlatoonRole() == PlatoonRole::FOLLOWER)
        role_str = "FOLLOWER";
    else if (getPlatoonRole() == PlatoonRole::JOINER)
        role_str = "JOINER";
    else if (getPlatoonRole() == PlatoonRole::UNSAFE_LEADER)
        role_str = "UNSAFE_LEADER";
    else if (getPlatoonRole() == PlatoonRole::UNSAFE_FOLLOWER)
        role_str = "UNSAFE_FOLLOWER";
    else
        role_str = "NONE";


    std::cout << "\n\n@@@ Vehicle " << myId << " in Platoon " << positionHelper->getPlatoonId() << " at " << simTime() << " @@@"
//                 << "\n Cruise Model: " << traciVehicle->getActiveController()
//                 << "\n P Helper id: "  << positionHelper->getId()
                 << "\n Platoon Pos: "  << positionHelper->getPosition()
                 << "\n Leader Pos: "   << positionHelper->getLeaderId()
                 << "\n Platoon Size: " << positionHelper->getPlatoonSize()
//                 << "\n P Leader: "     << positionHelper->isLeader()
//                 << "\n CurrentLane: "  << traciVehicle->getLaneIndex()
//                 << "\n Best Lane: "    << getSafestLane()
//                 << "\n Sign dist: "    << lastSeenRoadSign
//                 << "\n Sign time: "    << timeToRoadSign
//                 << "\n Headway (s): "  << traciVehicle->getACCHeadwayTime()
//                 << "\n Spacing (m): "  << traciVehicle->getCACCConstantSpacing()
                 << "\n Platoon Role: " << role_str
                 << "\n P Length: "     << telemetryMap->getPlatoonLength(traciVehicle->getLaneIndex())
                 << "\n In Danger: "    << inDanger
                 << "\n In Maneuver: "  << inManeuver;
//                 << "\n\n";


    for (int i = 0; i < 4; i++)
    {
        std::cout << "\nLane" << "[" << i << "]: " ;

        std::vector<int> formation = telemetryMap->getFormation(i);
        for (auto &j : formation)
            std::cout << j << " ";
    }

    std::vector<int> blocked_lanes = telemetryMap->getBlockedLanes();

    std::cout << "\nBlocked Lanes: "  ;
    for (int j = 0; j < blocked_lanes.size(); j++)
    {
        std::cout << blocked_lanes.at(j) << " ";
    }


}