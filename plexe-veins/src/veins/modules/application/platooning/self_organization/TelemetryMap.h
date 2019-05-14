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

#ifndef TELEMETRYMAP_H_
#define TELEMETRYMAP_H_

class BaseSelfOrganizationApp;

#include <algorithm>
#include <vector>
#include "../self_organization/VehicleCoord.h"
#include "veins/modules/application/platooning/messages/PlatooningBeacon_m.h"

#include "veins/modules/application/platooning/apps/PlatoonDefs.h"


/**
 * Stores geographic position of vehicles in surroundings
 * Only safe-lane leaders have the permission to access this information.
 */
class TelemetryMap {

public:

    // @author lauro
    // TODO Remover necessidade de parâmetro app pois mapa de telemetria
    // deve existir independente de aplicação e se manter apenas com dados
    // de mensagens
    TelemetryMap(BaseSelfOrganizationApp* app);

    virtual ~TelemetryMap();

    // On new beacon, update telemetry map
    void updateTelemetryMap(const PlatooningBeacon* pb);

    // On new info of the current vehicle
    void updateMyPosition(int vehicleId, int laneIndex, Plexe::VEHICLE_DATA& data);

    // Adds a road sign detected
    void addRoadSignDetected(std::string signType, int laneIndex, double range /*, Coord pos*/);

    // Detects if a maneuver is possible to be executed in a given lane
    // TODO Alterar o método para o seguinte
    // bool isSafeToMoveTo(int platoonId, int position);
    bool isSafeToMoveTo(PlatoonManeuver maneuver, int laneIndex);

    // Detects the length of the platoon
    double getPlatoonLength(int laneIndex);

    // Shows the distance from platoon's tail
    double distanceFromPlatoonTail(int laneIndex);

    // Gets info from vehicle in given laneIndex and position
    VehicleCoord getVehicleTopologyInfo(int laneIndex, int position);

    // Detects if vehicle is in a safe lane
    bool isLaneSafe(int vehicleId);

    // Detects if vehicle is the leader of the current lane
    bool isLaneLeader(int vehicleId);

    // Detects which is the safest lane
    int getSafestLaneIndex();

    // Gets the id of the leader from the safest lane detected
    int getSafestLaneLeader();

    std::vector<int> getFormation(int laneIndex);

    std::vector<int> getLaneLeaders();

    std::vector<int> getBlockedLanes();

protected:

    // Converts angle from radians to degrees
    int convertAngleToDegrees(double angleRad);

    // Checks the direction a vehicle is heading
    Direction getDirection(double angleRad);

    // Remove old telemetry
    void removeVehicleFromLane(int vehicleId, int laneIndex);

    // Sorts topology
    void sortTopology();


private:

    void calculatePlatoonSpacing();

private:

    // stores direction of current vehicle
    Direction myDirection;

    // stores n'bors' coordinates
    std::vector<VehicleCoord> nborCoord[5];

    // Calculates available spacing of platoon
    // It will be always number of cars + 1 spaces (counting front and back)
    std::vector<double> platoon_gaps [5];

    // Stores all elected leaders
    std::map<int, int> laneLeaders;

    // stores n'bors' last 'seen' lane
    std::map<int, int> lastSeenLane;

    // Stores all indexes of blocked lanes
    std::vector<int> blockedLanes;

    // Application
    BaseSelfOrganizationApp* app;

};

#endif /* TELEMETRYMAP_H_ */
