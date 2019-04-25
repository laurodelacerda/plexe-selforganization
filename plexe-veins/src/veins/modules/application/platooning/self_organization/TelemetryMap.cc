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

#include "TopologyMap.h"


using namespace Veins;


TopologyMap::TopologyMap(SignPlatooningApp* app)
{
    this->app = app;
}

TopologyMap::~TopologyMap()
{

}

void TopologyMap::updatePosition(int vehicleId, int laneIndex, Plexe::VEHICLE_DATA& data)
{
    VehicleCoord vCoord = VehicleCoord();
    vCoord.fromPlexeData(vehicleId, laneIndex, data);

    int lastSeenAt;

    if(lastSeenLane.count(vehicleId) == 0)
    {
        nborCoord[laneIndex].push_back(vCoord);
    }
    else
    {
        lastSeenAt = lastSeenLane[vehicleId];
        removeVehicleFromLane(vehicleId, lastSeenAt);
        nborCoord[laneIndex].push_back(vCoord);
    }

    lastSeenLane[vehicleId] = laneIndex;

    myDirection = getDirection(vCoord.getAngle());
}

void TopologyMap::updateRoadTopology(const PlatooningBeacon* pb)
{
    int       nb_id   = pb->getVehicleId();
    int       nb_lane = pb->getLaneIndex();
    Direction nb_head = getDirection(pb->getAngle());

    int lastSeenAt;

    VehicleCoord vCoord = VehicleCoord();
    vCoord.fromPacket(pb);

    // n'bors at some lane Direction to the same direction
    if ((nb_lane != -1) && (myDirection == nb_head))
    {
        if(lastSeenLane.count(nb_id) == 0) // New node
        {
            nborCoord[nb_lane].push_back(vCoord);
        }
        else                               // Known node
        {
            lastSeenAt = lastSeenLane[nb_id];
            removeVehicleFromLane(nb_id, lastSeenAt); // Remove node from old lane in topology
            nborCoord[nb_lane].push_back(vCoord);
        }

        lastSeenLane[nb_id] = nb_lane;
    }
}


void TopologyMap::addRoadSignDetected(std::string signType, int lane_index)
{
    // Adds the constrained lane to container
    // TODO Optimize this: It is currently O(n²), it could be O(n) in std::unordered_set
    if (std::find(blockedLanes.begin(), blockedLanes.end(), lane_index) == blockedLanes.end())
        blockedLanes.push_back(lane_index);


    // List of all possible road constraints
//    if (sign == "narrowing")
//        sign_detected = Sign::NARROW;
//    else if (sign == "triangle")
//        // coordinate vehicle's movements to minimize accidents and improve flux
//        sign_detected = Sign::TRIANGLE;
//    else if (sign == "stop")
//        // stop vehicles synchronously
//        sign_detected = Sign::STOP;
//    else if (sign == "reduce")
//        // reduce vehicles synchronously
//        sign_detected = Sign::REDUCE;
//    else {} // do nothing
}


bool TopologyMap::isSafeToMoveTo(PlatoonManeuver maneuver, int laneIndex)
{
    double platoon_dist;

    if (maneuver == PlatoonManeuver::JOIN_AT_BACK)
    {
        platoon_dist = distanceFromPlatoonTail(laneIndex);
        /// Vehicle should be at a safe distance from platoon's tail to perform maneuver
//        std::cout << "\nVehicle " << myId << " distance from platoon " << nborCoord[getSafestLaneIndex()].at(0).getId() << "'s tail is " << platoon_dist << std::endl;
    }

    if (platoon_dist > 0) // Vehicle is behind platoon's tail
        return true;
    else                  // Vehicle is in front of platoon's tail
        return false;
}


int TopologyMap::convertAngleToDegrees(double angleRad)
{
    double angleDeg;

    angleDeg = (180 / 3.14) * angleRad;
    angleDeg = fmod(angleDeg, 360);

    if (angleDeg < 0)
        angleDeg += 360;

    return angleDeg;
}

Direction TopologyMap::getDirection(double angleRad)
{
    int angleDeg = convertAngleToDegrees(angleRad);

    Direction direction;

    switch(angleDeg)
    {
    case 0 ... 44:
        direction = Direction::EAST;
        break;

    case 45 ... 134:
        direction = Direction::NORTH;
        break;

    case 135 ... 224:
        direction = Direction::WEST;
        break;

    case 225 ... 314:
        direction = Direction::SOUTH;
        break;

    case 315 ... 359:
        direction = Direction::EAST;
        break;
    }

    return direction;
}

void TopologyMap::removeVehicleFromLane(int vehicleId, int laneIndex)
{
    for (auto it = nborCoord[laneIndex].begin(); it != nborCoord[laneIndex].end(); )
    {

        if ((*it).getId() == vehicleId)
            it = nborCoord[laneIndex].erase(it);
        else
            ++it;
    }
}

bool comparePosNorthA (VehicleCoord a, VehicleCoord b)
{
    return (a.getCoord().y > b.getCoord().y);
}

bool comparePosSouthA (VehicleCoord a, VehicleCoord b)
{
    return (a.getCoord().y < b.getCoord().y);
}

bool comparePosEastA (VehicleCoord a, VehicleCoord b)
{
    return (a.getCoord().x > b.getCoord().x);
}

bool comparePosWestA (VehicleCoord a, VehicleCoord b)
{
    return (a.getCoord().x < b.getCoord().x);
}

void TopologyMap::sortTopology()
{
    for(int i = 0; i < (sizeof(nborCoord)/sizeof(nborCoord[0])); i++)
    {
        if (myDirection == Direction::NORTH)
            std::sort(nborCoord[i].begin(), nborCoord[i].end(), comparePosNorthA);
        else if (myDirection == Direction::SOUTH)
            std::sort(nborCoord[i].begin(), nborCoord[i].end(), comparePosSouthA);
        else if (myDirection == Direction::EAST)
            std::sort(nborCoord[i].begin(), nborCoord[i].end(), comparePosEastA);
        else if (myDirection == Direction::WEST)
            std::sort(nborCoord[i].begin(), nborCoord[i].end(), comparePosWestA);
    }
}


double TopologyMap::getPlatoonLength(int laneIndex)
{
    double leaderPos;
    double tailPos;

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

        if ((myDirection == Direction::NORTH) || (myDirection == Direction::SOUTH))
        {
            leaderPos = leader_data.getCoord().y;
            tailPos   = tail_data.getCoord().y;
        }
        else if ((myDirection == Direction::EAST) || (myDirection == Direction::WEST))
        {
            leaderPos = leader_data.getCoord().x;
            tailPos   = tail_data.getCoord().x;
        }

        return std::fabs(leaderPos - tailPos) + tail_length;
    }

}

double TopologyMap::distanceFromPlatoonTail(int laneIndex)
{
    int platoon_size = nborCoord[laneIndex].size();

    if (platoon_size == 0)
        return -1;
    else
    {
        Plexe::VEHICLE_DATA data;
        app->getTraciVehicle()->getVehicleData(&data);

        VehicleCoord platoon_tail = nborCoord[laneIndex].at(platoon_size - 1);

        // NOTE Posição do líder menos posição da cauda menos distância de segurança
        if ((myDirection == Direction::NORTH) || (myDirection == Direction::SOUTH))
            return (platoon_tail.getCoord().y - data.positionY) - (platoon_tail.getLength()/2) - 2;
        else if ((myDirection == Direction::EAST) || (myDirection == Direction::WEST))
            return (platoon_tail.getCoord().x - data.positionX) - (platoon_tail.getLength()/2) - 2;
        else
            return -1;
    }
}

VehicleCoord TopologyMap::getVehicleTopologyInfo(int laneIndex, int position)
{
    // WARNING Verificar se é uma lane trafegável
    return nborCoord[laneIndex].at(position);
}

bool TopologyMap::isLaneSafe(int vehicleId)
{
    if (std::find(blockedLanes.begin(), blockedLanes.end(), lastSeenLane[vehicleId]) != blockedLanes.end())
        return false;
    else
        return true;
}

bool TopologyMap::isLaneLeader(int vehicleId)
{
    int lane_index = app->getCurrentLaneIndex();

    this->sortTopology();

    for(int i = 0; i < (sizeof(nborCoord)/sizeof(nborCoord[0])); i++)
    {
//        std::sort(nborCoord[i].begin(), nborCoord[i].end(), comparePosEastA);

        if (nborCoord[i].size() > 0)
            laneLeaders[i] = nborCoord[i].at(0).getId();
        else
            laneLeaders[i] = -99;
    }

    if (laneLeaders[lane_index] == vehicleId)
        return true;
    else
        return false;
}

int TopologyMap::getSafestLaneIndex()
{
    int current_lane = app->getCurrentLaneIndex();

    std::vector<int> safeLanes;

    for (auto const& it : laneLeaders)
    {
        /// BUG Vehicles are finding their own lanes
        /// TODO Get Index of Lanes in SUMO source file
        if (std::find(blockedLanes.begin(), blockedLanes.end(), it.first) == blockedLanes.end())
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


int TopologyMap::getSafestLaneLeader()
{
    nborCoord[this->getSafestLaneIndex()].at(0).getId();
}

std::vector<int> TopologyMap::getFormation(int laneIndex)
{
    std::vector<int> formation;

    this->sortTopology();

    for(int i = 0; i < nborCoord[laneIndex].size(); i++)
    {
        formation.push_back(nborCoord[laneIndex].at(i).getId());
    }

    return formation;
}

std::vector<int> TopologyMap::getLaneLeaders()
{
    std::vector<int> leader_list;

    for (auto &i : laneLeaders) leader_list.push_back(i.second);

    return leader_list;
}

std::vector<int> TopologyMap::getBlockedLanes()
{
    return blockedLanes;
}
