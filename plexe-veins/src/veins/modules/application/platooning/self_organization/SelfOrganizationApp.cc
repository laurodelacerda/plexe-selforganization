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

void SelfOrganizationApp::initialize(int stage)
{
    BaseSelfOrganizationApp::initialize(stage);

    if (stage == 1)
    {
//        std::string joinManeuverName = par("joinManeuver").stdstringValue();
//        if (joinManeuverName == "JoinAtBack")
//            joinManeuver = new JoinAtBack(this);
//        else if (joinManeuverName == "JoinAtBackSign")
//            joinManeuver = new JoinAtBackSign(this);
//        else
//            throw new cRuntimeError("Invalid join maneuver implementation chosen");
        joinManeuver = new JoinAtBackSign(this);

    }
}


void SelfOrganizationApp::onManeuverMessage(ManeuverMessage* mm)
{
    if (UpdatePlatoonFormation* msg = dynamic_cast<UpdatePlatoonFormation*>(mm))
    {
        handleUpdatePlatoonFormation(msg);
        delete msg;
    }
    else
    {
        joinManeuver->onManeuverMessage(mm);
//        // Verify each kind of maneuver to invoke correct maneuver
//        if (maneuverInCourse == PlatoonManeuver::JOIN_AT_BACK)
//        {
////            joinAtBack->onManeuverMessage(mm);
//        }
//        else if (maneuverInCourse == PlatoonManeuver::JOIN_IN_THE_MIDDLE)
//        {
////            joinInTheMiddle->onManeuverMessage(mm);
//        }
//        else if (maneuverInCourse == PlatoonManeuver::JOIN_AT_FRONT)
//        {
////            joinAtFront->onManeuverMessage(mm);
//        }
//        else
//        {}
////      TODO Adicionar outras manobras
    }

    delete mm;
}

void SelfOrganizationApp::startJoinManeuver(int platoonId, int leaderId, int position)
{
    ASSERT(getPlatoonRole() == PlatoonRole::NONE || getPlatoonRole() == PlatoonRole::UNSAFE_FOLLOWER || getPlatoonRole() == PlatoonRole::UNSAFE_LEADER);
    ASSERT(!isInManeuver());

    JoinManeuverParameters params;
    params.platoonId = platoonId;
    params.leaderId = leaderId;
    params.position = position;
    joinManeuver->startManeuver(&params);
}


//void SelfOrganizationApp::handleSelfMsg(cMessage* msg){}


void SelfOrganizationApp::abortJoinManeuver()
{

}

void SelfOrganizationApp::startSplitManeuver()
{
}

void SelfOrganizationApp::abortSplitManeuver()
{}

void SelfOrganizationApp::startMultipleJoinManeuver()
{}

void SelfOrganizationApp::abortMultipleJoinManeuver()
{}

void SelfOrganizationApp::setupFormation()
{
    std::vector<int> formation = map->getFormation(traciVehicle->getLaneIndex());

    positionHelper->setPlatoonFormation(formation);
}

void SelfOrganizationApp::startManeuverFormation()
{
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

void SelfOrganizationApp::startUnsafeLeaderFormation()
{
    role = PlatoonRole::UNSAFE_LEADER;
    setInManeuver(false);

    int safest_lane   = map->getSafestLaneIndex();
    int safest_leader = map->getSafestLaneLeader();
    bool lane_leader  = map->isLaneLeader(myId);

    bool permission = map->isSafeToMoveTo(PlatoonManeuver::JOIN_AT_BACK, safest_lane);
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

    bool lane_leader  = map->isLaneLeader(myId);

    if (inDanger && !safeJoinCheck->isScheduled() && !lane_leader)
        scheduleAt(simTime() + SimTime(100, SIMTIME_MS), safeJoinCheck);
}

SelfOrganizationApp::~SelfOrganizationApp()
{
    delete joinManeuver;
}

void SelfOrganizationApp::sendUnicast(cPacket* msg, int destination)
{
    Enter_Method_Silent();
    take(msg);
    UnicastMessage* unicast = new UnicastMessage("UnicastMessage", msg->getKind());
    unicast->setDestination(destination);
    unicast->setChannel(Channels::CCH);
    unicast->encapsulate(msg);
    sendDown(unicast);
}
