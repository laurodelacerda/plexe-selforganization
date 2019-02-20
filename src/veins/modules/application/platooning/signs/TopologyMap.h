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

#ifndef TOPOLOGYMAP_H_
#define TOPOLOGYMAP_H_

#include "veins/modules/application/platooning/signs/VehicleCoord.h"
#include "veins/modules/application/platooning/messages/PlatooningBeacon_m.h"
#include "veins/modules/application/platooning/signs/SignPlatooningApp.h"

enum class Direction : size_t
{
    NONE,
    NORTH,
    SOUTH,
    WEST,
    EAST,
    NORTHWEST,
    SOUTHWEST,
    NORTHEAST,
    SOUTHEAST
};


class TopologyMap {

public:

    TopologyMap(SignPlatooningApp* app);

    virtual ~TopologyMap();

    // On new beacon, update topology
    void updateRoadTopology(const PlatooningBeacon* pb);

    // On new info on the vehicle
    void updatePosition(int vehicleId, int laneIndex, Plexe::VEHICLE_DATA& data);

protected:

    int convertAngleToDegrees(double angleRad);

    Direction getDirection(double angleRad);

    void removeVehicleFromLane(int vehId, int lane);

    void sortTopology();

    bool isSafeToMoveTo(PlatoonManeuver m, int laneIndex);

    double getPlatoonLength(int laneIndex);

    /// Shows the distance from platoon's tail
    double distanceFromPlatoonTail(int laneIndex);

    VehicleCoord getVehicleTopologyInfo(int lane, int position);

    // Checks if vehicle is in safe lane
    bool isLaneSafe(int vehicleId);

    // Checks if vehicle is leader of the current lane
    bool isLaneLeader(int vehicleId);

    // Gets the safest lane to request help
    int getSafestLane();

    // Update leadership and sefety flags
    // TODO Migrate to enum sign_status
    PlatoonRole getPlatoonRole(int vehicleId);

private:

    Direction myDirection;

    // stores n'bors' telemetry
    std::deque<VehicleCoord> nborCoord[5];

    // Stores all elected leaders
    std::map<int, int> laneLeaders;

    // stores n'bors' last 'seen' lane
    std::map<int, int> lastSeenLane;

    // Stores all constrained Lanes
    std::vector<std::string> constrainedLanes;

    // Stores all constrained Lanes Indexes
    std::vector<int> constrainedLaneIndexes;

    SignPlatooningApp* app;

};

#endif /* TOPOLOGYMAP_H_ */
