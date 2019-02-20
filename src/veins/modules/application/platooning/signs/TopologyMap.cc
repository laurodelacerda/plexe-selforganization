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
: app(app)
{

}

TopologyMap::~TopologyMap()
{

}

void TopologyMap::updatePosition(int vehicleId, int laneIndex, Plexe::VEHICLE_DATA& data)
{
    VehicleCoord vCoord = VehicleCoord();
    vCoord.fromPlexeData(vehicleId, laneIndex, data);

    if(lastSeenLane.count(vehicleId) == 0)
    {
        nborCoord[laneIndex].push_back(vCoord);
    }
    else
    {
        int lastSeenAt = lastSeenLane[laneIndex];
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
    int       nb_len  = pb->getLength();

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
            int lastSeenAt = lastSeenLane[nb_id];
            removeVehicleFromLane(nb_id, lastSeenAt); // Remove node from old lane in topology
            nborCoord[nb_lane].push_back(vCoord);
        }

        lastSeenLane[nb_id] = nb_lane;
    }
}

int TopologyMap::convertAngleToDegrees(double angleRad)
{
    double angleDeg;

    angleDeg = (180 / 3.14) * angleRad;
    angleDeg = fmod(angleDeg, 360);

    if (angleDeg < 0)
        angleDeg += 360;

    //    EV_INFO << angleRad << " " << angleDeg << endl;
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

void TopologyMap::removeVehicleFromLane(int vehId, int lane)
{
    for (auto it = nborCoord[lane].begin(); it != nborCoord[lane].end(); )
    {
        if ((*it).getId() == vehId)
            it = nborCoord[lane].erase(it);
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

bool TopologyMap::isSafeToMoveTo(PlatoonManeuver m, int laneIndex)
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

double TopologyMap::getPlatoonLength(int laneIndex)
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

VehicleCoord TopologyMap::getVehicleTopologyInfo(int lane, int position)
{
    int current_lane = app->getTraciVehicle()->getLaneIndex();

    // WARNING Verificar se é uma lane trafegável
    return nborCoord[lane].at(position);
}

bool TopologyMap::isLaneSafe(int vehicleId)
{
    std::string blockedLane;
    app->getTraciVehicle()->getCustomParameter("device.signDetector.lastRoadSignLane", blockedLane);

    std::string lane;
    app->getTraciVehicle()->getCustomParameter("device.signDetector.lastRoadSignLaneIndex", lane);
    int sign_index = std::atoi(lane.c_str());

    // Adds the constrained lane to container
    // TODO Optimize this: It is currently O(n²), it could be O(n) in std::unordered_set
    if (std::find(constrainedLanes.begin(), constrainedLanes.end(), blockedLane) == constrainedLanes.end())
    {
        constrainedLanes.push_back(blockedLane);
        constrainedLaneIndexes.push_back(sign_index);
    }

    // Checks if vehicle is in safe lane
    if (std::find(constrainedLaneIndexes.begin(), constrainedLaneIndexes.end(), vehicleId) != constrainedLaneIndexes.end())
        return false;
    else
        return true;
}

bool TopologyMap::isLaneLeader(int vehicleId)
{
    int lane_index = app->getTraciVehicle()->getLaneIndex();

    for(int i = 0; i < (sizeof(nborCoord)/sizeof(nborCoord[0])); i++)
    {
        std::sort(nborCoord[i].begin(), nborCoord[i].end(), comparePosEastA);

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

int TopologyMap::getSafestLane()
{
    int current_lane = app->getTraciVehicle()->getLaneIndex();

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

PlatoonRole TopologyMap::getPlatoonRole(int vehicleId)
{
    bool lane_safe   = isLaneSafe(vehicleId);
    bool lane_leader = isLaneLeader(vehicleId);

    if ((lane_leader) && (lane_safe))
    {
//        findHost()->getDisplayString().updateWith("r=8,green");
        return PlatoonRole::LEADER;
    }
    else if ((lane_leader) && (!lane_safe))
    {
//        findHost()->getDisplayString().updateWith("r=8,red");
        return PlatoonRole::UNSAFE_LEADER;
    }
    else if ((!lane_leader) && (lane_safe))
    {
//        findHost()->getDisplayString().updateWith("r=8,blue");
        return PlatoonRole::FOLLOWER;
    }
    else
    {
//        findHost()->getDisplayString().updateWith("r=0,gray");
        return PlatoonRole::UNSAFE_FOLLOWER;
    }
}
