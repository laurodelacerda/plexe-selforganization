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

#include "SignPlatooningApp.h"

using namespace Veins;

Define_Module(SignPlatooningApp);


SignPlatooningApp::SignPlatooningApp()
{}


void SignPlatooningApp::initialize(int stage)
{
    GeneralPlatooningApp::initialize(stage);

    safeToManeuver = false;

//    inSafeLane   = false ;
//    isLaneLeader = false ;

    roadSignMode = false;
    roadSignPos = -99;
    lastSeenRoadSign   = -99;
    timeToRoadSign     = -99;

    if (stage == 1)
    {
        SimTime rounded = SimTime(floor(simTime().dbl() * 1000 + 100), SIMTIME_MS);

        topologyCheck = new cMessage("topologyCheck");
        scheduleAt(rounded, topologyCheck);

        telemetryCheck = new cMessage("telemetryCheck");
        scheduleAt(rounded, telemetryCheck);

        safeJoinCheck = new cMessage("safeJoinCheck");
    }

}


SignPlatooningApp::~SignPlatooningApp()
{

    cancelAndDelete(topologyCheck);
    topologyCheck = nullptr;
    cancelAndDelete(telemetryCheck);
    telemetryCheck = nullptr;
    cancelAndDelete(safeJoinCheck);
    safeJoinCheck = nullptr;

}


void SignPlatooningApp::handleSelfMsg(cMessage* msg)
{
    if (msg == telemetryCheck)
    {
        updatePosition();
        updateFlags();
        scheduleAt(simTime() + SimTime(100, SIMTIME_MS), telemetryCheck);
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


void SignPlatooningApp::onPlatoonBeacon(const PlatooningBeacon* pb)
{
    // TODO Check if message has Extended Beaconing Kind
    updateRoadTopology(pb);

    // TODO Check if message is a maneuver to be developed

    // Send to lower layers, including maneuvers
    GeneralPlatooningApp::onPlatoonBeacon(pb);
}


void SignPlatooningApp::updateRoadTopology(const PlatooningBeacon* pb) {

    int     nb_id   = pb->getVehicleId();
    int     nb_lane = pb->getLaneIndex();
    Heading nb_head = getDirection(pb->getAngle());
    int     nb_len  = pb->getLength();

    // Nbors at some lane and heading to the same direction
    if ((nb_lane != -1) && (myHeading == nb_head))
    {
        if(lastSeenLane.count(nb_id) == 0) // New node
        {
            nborCoord[nb_lane].push_back(VehicleCoord(nb_id, nb_lane, Coord(pb->getPositionX(), pb->getPositionY()), nb_len));
        }
        else                               // Known node
        {
            int lastSeenAt = lastSeenLane[nb_id];
            removeVehicleFromLane(nb_id, lastSeenAt); // Remove node from old lane in topology
            nborCoord[nb_lane].push_back(VehicleCoord(nb_id, nb_lane, Coord(pb->getPositionX(), pb->getPositionY()), nb_len));
        }

        lastSeenLane[nb_id] = nb_lane;
    }
}


void SignPlatooningApp::updatePosition()
{
    Plexe::VEHICLE_DATA data;
    traciVehicle->getVehicleData(&data);

    int lane_index = traciVehicle->getLaneIndex();

    if(lastSeenLane.count(myId) == 0)
        nborCoord[lane_index].push_back(VehicleCoord(myId, lane_index, Coord(data.positionX, data.positionY)));
    else
    {
        int lastSeenAt = lastSeenLane[myId];
        removeVehicleFromLane(myId, lastSeenAt);
        nborCoord[lane_index].push_back(VehicleCoord(myId, lane_index, Coord(data.positionX, data.positionY)));
    }

    lastSeenLane[myId] = lane_index;

    myHeading = getDirection(mobility->getAngleRad());

}


void SignPlatooningApp::removeVehicleFromLane(int vehId, int lane)
{
    for (auto it = nborCoord[lane].begin(); it != nborCoord[lane].end(); )
    {
        if ((*it).getId() == vehId)
            it = nborCoord[lane].erase(it);
        else
            ++it;
    }
}


SignPlatooningApp::Heading SignPlatooningApp::getDirection(double angleRad)
{
    int angleDeg = convertAngleToDegrees(angleRad);

    Heading direction;

    switch(angleDeg)
    {
    case 0 ... 44:
        direction = Heading::EAST;
        break;

    case 45 ... 134:
        direction = Heading::NORTH;
        break;

    case 135 ... 224:
        direction = Heading::WEST;
        break;

    case 225 ... 314:
        direction = Heading::SOUTH;
        break;

    case 315 ... 359:
        direction = Heading::EAST;
        break;
    }

    return direction;
}


int SignPlatooningApp::convertAngleToDegrees(double angleRad)
{
    double angleDeg;

    angleDeg = (180 / 3.14) * angleRad;
    angleDeg = fmod(angleDeg, 360);

    if (angleDeg < 0)
        angleDeg += 360;

    //    EV_INFO << angleRad << " " << angleDeg << endl;
    return angleDeg;
}


void SignPlatooningApp::onRoadSignDetection(std::string sign)
{

    // List of all possible road constraints
    if (sign == "narrowing")
        sign_detected = Sign::NARROW;
    else if (sign == "triangle")
        // coordinate vehicle's movements to minimize accidents and improve flux
        sign_detected = Sign::TRIANGLE;
    else if (sign == "stop")
        // stop vehicles synchronously
        sign_detected = Sign::STOP;
    else if (sign == "reduce")
        // reduce vehicles synchronously
        sign_detected = Sign::REDUCE;
    else {} // do nothing

    updateFlags();
    startManeuverFormation();

}


bool SignPlatooningApp::isLaneSafe()
{
    std::string blockedLane;
    traciVehicle->getCustomParameter("device.signDetector.lastRoadSignLane", blockedLane);

    std::string lane;
    traciVehicle->getCustomParameter("device.signDetector.lastRoadSignLaneIndex", lane);
    int sign_index = std::atoi(lane.c_str());

    // Adds the constrained lane to container
    // TODO Optimize this: It is currently O(n²), it could be O(n) in std::unordered_set
    if (std::find(constrainedLanes.begin(), constrainedLanes.end(), blockedLane) == constrainedLanes.end())
    {
        constrainedLanes.push_back(blockedLane);
        constrainedLaneIndexes.push_back(sign_index);
    }

    int lane_index = traciVehicle->getLaneIndex();

    // Checks if vehicle is in safe lane
    if (std::find(constrainedLaneIndexes.begin(), constrainedLaneIndexes.end(), lane_index) != constrainedLaneIndexes.end())
        return false;
    else
        return true;
}


bool comparePosNorth (VehicleCoord a, VehicleCoord b)
{
    return (a.getCoord().y > b.getCoord().y);
}

bool comparePosSouth (VehicleCoord a, VehicleCoord b)
{
    return (a.getCoord().y < b.getCoord().y);
}

bool comparePosEast (VehicleCoord a, VehicleCoord b)
{
    return (a.getCoord().x > b.getCoord().x);
}

bool comparePosWest (VehicleCoord a, VehicleCoord b)
{
    return (a.getCoord().x < b.getCoord().x);
}

bool SignPlatooningApp::isLaneLeader()
{
    int lane_index = traciVehicle->getLaneIndex();

    for(int i = 0; i < (sizeof(nborCoord)/sizeof(nborCoord[0])); i++)
    {
        std::sort(nborCoord[i].begin(), nborCoord[i].end(), comparePosEast);

        if (nborCoord[i].size() > 0)
            laneLeaders[i] = nborCoord[i].at(0).getId();
        else
            laneLeaders[i] = -99;
    }

    if (laneLeaders[lane_index] == myId)
        return true;
    else
        return false;
}


int SignPlatooningApp::getSafestLane()
{
    int current_lane = traciVehicle->getLaneIndex();

    std::vector<int> safeLanes;

    for (auto const& it : laneLeaders)
    {
        /// BUG Vehicles are finding their own lanes
        /// TODO Get Index of Lanes in SUMO source file
        if (std::find(constrainedLaneIndexes.begin(), constrainedLaneIndexes.end(), it.first) == constrainedLaneIndexes.end())
            safeLanes.push_back(it.first);
    }

    // Find nearest and safest lane
    int safest_lane = 100;
    int least_distance;
    int aux = 100;

    for (int j = 0; j < safeLanes.size(); ++j)
    {
        least_distance = std::abs(current_lane - safeLanes.at(j));

        if (least_distance < aux)
        {
            aux = least_distance;
            safest_lane = safeLanes.at(j);
        }

    }

    return safest_lane;
}


void SignPlatooningApp::updateFlags()
{
    bool lane_safe   = isLaneSafe();
    bool lane_leader = isLaneLeader();

    if ((lane_leader) && (lane_safe))
    {
        findHost()->getDisplayString().updateWith("r=8,green");
        role = PlatoonRole::LEADER;
    }
    else if ((lane_leader) && (!lane_safe))
    {
        findHost()->getDisplayString().updateWith("r=8,red");
        role = PlatoonRole::UNSAFE_LEADER;
    }
    else if ((!lane_leader) && (lane_safe))
    {
        findHost()->getDisplayString().updateWith("r=8,blue");
        role = PlatoonRole::FOLLOWER;
    }
    else
    {
        findHost()->getDisplayString().updateWith("r=0,gray");
        role = PlatoonRole::UNSAFE_FOLLOWER;
    }

}


void SignPlatooningApp::setupFormation()
{
    std::vector<int> formation;

    int lane_index = traciVehicle->getLaneIndex();

    for (int i = 0; i < nborCoord[lane_index].size(); i++)
        formation.push_back(nborCoord[lane_index].at(i).getId());

    positionHelper->setPlatoonFormation(formation);

}


void SignPlatooningApp::startManeuverFormation()
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


void SignPlatooningApp::sendHelpRequest(int platoon_leader)
{
}


void SignPlatooningApp::startSafeLeaderFormation()
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


void SignPlatooningApp::startSafeFollowerFormation()
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


void SignPlatooningApp::startUnsafeLeaderFormation()
{

    role = PlatoonRole::UNSAFE_LEADER;
    inManeuver = false;
    int safest_lane   = getSafestLane();
    int safest_leader = nborCoord[safest_lane].at(0).getId();

    bool permission = isSafeToMove(PlatoonManeuver::JOIN_AT_BACK, safest_lane);
//    std::cout << "Permissão: " << permission << std::endl;

    if (permission)
        startJoinManeuver(safest_leader, safest_leader, -1);
    else
        traciVehicle->slowDown(40 / 3.6, 0);
        if (!safeJoinCheck->isScheduled())
            scheduleAt(simTime() + SimTime(500, SIMTIME_MS), safeJoinCheck);
}


void SignPlatooningApp::startUnsafeFollowerFormation()
{

//    setPlatoonRole(PlatoonRole::NONE);

//    traciVehicle->setCruiseControlDesiredSpeed(160.0 / 3.6);
//    traciVehicle->setActiveController(Plexe::CACC);
    traciVehicle->setFixedLane(traciVehicle->getLaneIndex());
//    traciVehicle->setCACCConstantSpacing(1);

    positionHelper->setIsLeader(false);
    positionHelper->setPlatoonLane(traciVehicle->getLaneIndex());
    positionHelper->setPlatoonSpeed(50 / 3.6);
    positionHelper->setPlatoonId(positionHelper->getLeaderId());

}


void SignPlatooningApp::sortTopology()
{
    for(int i = 0; i < (sizeof(nborCoord)/sizeof(nborCoord[0])); i++)
    {
        if (myHeading == Heading::NORTH)
            std::sort(nborCoord[i].begin(), nborCoord[i].end(), comparePosNorth);
        else if (myHeading == Heading::SOUTH)
            std::sort(nborCoord[i].begin(), nborCoord[i].end(), comparePosSouth);
        else if (myHeading == Heading::EAST)
            std::sort(nborCoord[i].begin(), nborCoord[i].end(), comparePosEast);
        else if (myHeading == Heading::WEST)
            std::sort(nborCoord[i].begin(), nborCoord[i].end(), comparePosWest);
    }
}


bool SignPlatooningApp::isSafeToMove(PlatoonManeuver m, int laneIndex)
{
    double platoon_dist;

    if (m == PlatoonManeuver::JOIN_AT_BACK)
    {
        platoon_dist = distanceFromPlatoonTail(laneIndex);
        /// Vehicle should be at a safe distance from platoon's tail to perform maneuver
//        std::cout << "\nVehicle " << myId << " distance from platoon " << nborCoord[getSafestLane()].at(0).getId() << "'s tail is " << platoon_dist << std::endl;
    }

    if (platoon_dist > 0) // Vehicle is behind platoon's tail
        return true;
    else                  // Vehicle is in front of platoon's tail
        return false;


}

double SignPlatooningApp::getPlatoonLength(int laneIndex)
{
    double leaderPos;
    double tailPos;

//    if (positionHelper->getPlatoonSize() == 0)
    if (nborCoord[laneIndex].size() == 0)
    {
        return -1;
    }
    else
    {
        VehicleCoord leader_data = getVehicleTopologyInfo(laneIndex, 0);

        int size = nborCoord[laneIndex].size();

        VehicleCoord tail_data   = getVehicleTopologyInfo(laneIndex, size - 1);

        double tail_length = tail_data.getLength();

        if ((myHeading == Heading::NORTH) || (myHeading == Heading::SOUTH))
        {
            leaderPos = leader_data.getCoord().y;
            tailPos   = tail_data.getCoord().y;
        }
        else if ((myHeading == Heading::EAST) || (myHeading == Heading::WEST))
        {
            leaderPos = leader_data.getCoord().x;
            tailPos   = tail_data.getCoord().x;
        }

        return std::fabs(leaderPos - tailPos) + tail_length;
    }
}

double SignPlatooningApp::distanceFromPlatoonTail(int laneIndex)
{
    int platoon_size = nborCoord[laneIndex].size();

    if (platoon_size == 0)
        return -1;
    else
    {
        Plexe::VEHICLE_DATA data;
        traciVehicle->getVehicleData(&data);

        VehicleCoord platoon_tail = nborCoord[laneIndex].at(platoon_size - 1);

        // NOTE Posição do líder menos posição da cauda menos distância de segurança
        if ((myHeading == Heading::NORTH) || (myHeading == Heading::SOUTH))
            return (platoon_tail.getCoord().y - data.positionY) - (platoon_tail.getLength()/2) - 2;
        else if ((myHeading == Heading::EAST) || (myHeading == Heading::WEST))
            return (platoon_tail.getCoord().x - data.positionX) - (platoon_tail.getLength()/2) - 2;
        else
            return -1;
    }
}

VehicleCoord SignPlatooningApp::getVehicleTopologyInfo(int lane, int position)
{
    int current_lane = traciVehicle->getLaneIndex();

    // WARNING Verificar se é uma lane trafegável
    return nborCoord[lane].at(position);
}

void SignPlatooningApp::printRoadSignInfo(std::string sign)
{
    std::string lane ;
    traciVehicle->getCustomParameter("device.signDetector.lastRoadSignLane", lane);

    std::string role = "none";
    if (getPlatoonRole() == PlatoonRole::LEADER)
        role = "leader";
    else if (getPlatoonRole() == PlatoonRole::FOLLOWER)
        role = "follower";

    std::cout << "@@@ Vehicle " << myId  << " in Platoon " << positionHelper->getPlatoonId() << " at " << simTime() << " @@@"
              << "\n\n@@@ Sign: "        << sign
              << "\n Sign Lane: "        << lane
              << "\n My Lane: "          << traciVehicle->getLaneId()
              << "\n Platoon Role: "     << role
              << "\n\n";

//    for (auto const& it : laneLeaders)
//    {
//        std::cout << it.first << " : " << it.second << std::endl;
//    }Fla
//
//    std::cout << "\n\n" << std::endl;
}


void SignPlatooningApp::printRoadTopology(bool sorted)
{
//    std::cout << ">>> Car " << myId << " on lane " << traciVehicle->getLaneIndex() << std::endl;

    for (int i = 0; i < 4; i++)
    {
        std::cout << "\nLane" << "[" << i << "]: " ;

        if (sorted)
            std::sort(nborCoord[i].begin(), nborCoord[i].end(), comparePosEast);

        for (auto &j : nborCoord[i])
        {
            std::cout << j.getId() << "[" << j.getCoord().x << ", " << j.getLastUpdate() << "] ";
        }
    }

    std::cout << "\nConstrained Lanes: ";
    for (auto const& it : constrainedLanes)
        std::cout << it << " " ;

//    int tags = findHost()->getDisplayString().getNumTags();
//    std::cout << "\nTags: " << tags ;
//    for(int i = 0; i < tags; i++) std::cout << "\nTag " << i << " : " << findHost()->getDisplayString().getTagName(i) ;

    std::cout << "\n\n";
}


void SignPlatooningApp::printInfo()
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

    std::cout << "@@@ Vehicle " << myId << " in Platoon " << positionHelper->getPlatoonId() << " at " << simTime() << " @@@"
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
                 << "\n P Length: "     << getPlatoonLength(traciVehicle->getLaneIndex());
//                 << "\n\n";
    printRoadTopology();

}
