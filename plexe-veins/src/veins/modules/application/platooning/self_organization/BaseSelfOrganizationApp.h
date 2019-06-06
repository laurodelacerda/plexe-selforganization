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

#ifndef BASESELFORGANIZATIONAPP_H_
#define BASESELFORGANIZATIONAPP_H_

#include <cfloat>
#include "veins/modules/application/platooning/apps/GeneralPlatooningApp.h"
#include "veins/modules/application/platooning/self_organization/VehicleCoord.h"
#include "veins/modules/application/platooning/messages/RoadSignWarning_m.h"

// Distâncias em metros
#define SAFETY_GAP           5
#define DISTANCE_SAFE      200
#define DISTANCE_CAUTIOUS  150
#define DISTANCE_DANGEROUS  50

// Cronômetros em ms
#define TIMER_ROAD_SIGN    100 // em ms

/**
 * General purpose application for Self-Organizing Platoons
 * This module is focused on interaction (communicating vehicles) and self-adaptation on road constraints.
 *
 * @see BaseApp
 * @see Maneuver
 */
class BaseSelfOrganizationApp : public GeneralPlatooningApp {

public:

    /** c'tor for BaseSelfOrganizationApp */
    BaseSelfOrganizationApp()
        : inDanger(false),
          blocking_distance(DistanceToBlockage::NONE)
    {
    }

    /** d'tor for BaseSelfOrganizationApp */
    virtual ~BaseSelfOrganizationApp();

    virtual void initialize(int stage);

    // Decides what to do when a road sign is detected
    void onRoadSignDetection(std::string sign_id, std::string sign_type, int lane_index, double range);

    virtual bool isSafeToManeuver(int lane_index, int position);

    virtual void adjustToManeuver(){}

    bool isInDanger() const
    {
        return inDanger;
    }

    void setInDanger(bool inDanger)
    {
        this->inDanger = inDanger;
    }


protected:

    virtual void fillRoadSignMessage(RoadSignWarning* msg, std::string sign_id, std::string sign_type, int lane_index, double posX, double posY);

    // Detect self events
    virtual void handleSelfMsg(cMessage* msg);

    virtual void handleLowerMsg(cMessage* msg) override;

    /** override from GeneralPlatooningApp */
    virtual void onPlatoonBeacon(const PlatooningBeacon* pb) override;

    // On new beacon, update telemetry map
    void updateTelemetryMap(const PlatooningBeacon* pb);

    // On new info of the current vehicle
    void updateMyPosition(int vehicleId, int laneIndex, Plexe::VEHICLE_DATA& data);

    // Update lane leadership and safety flags
    void updateFlags();

    void onRoadSignWarningMsg(RoadSignWarning* msg);

    // Monitors vehicle status in relation to road signs
    void onRoadSignTracking();

     void addNewRoadSign(std::string sign_id, std::string sign_type, int lane_index);

    // Update Platoon Formation
    virtual void setupFormation(){}

    // Set Platoon Formation after road sign detection
    virtual void startManeuverFormation(){}

    // Detects if a maneuver is possible to be executed in a given lane
    double calculateGap(int lane_index, int position);

    // Detects the length of the platoon
    double getPlatoonLength(int laneIndex);

    // Calculates the distance from a vehicle to any reachable platoon gap
    double calculateDistanceToPlatoonMember(int lane_index, int position);

    double calculateDistancePlatoonMembers(int lane_index, int position);

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

    virtual void calculatePlatoonGaps(std::vector<double> &gaps, int lane_index, double veh_lenght = 0);

    // Converts angle from radians to degrees
    int convertAngleToDegrees(double angleRad);

    // Checks the direction a vehicle is heading
    Orientation calculateDirection(double angleRad);

    // Checks the direction a platoon is heading
    Orientation calculatePlatoonDirection(int lane_index);

    // Remove old telemetry
    void removeVehicleFromLane(int vehicleId, int laneIndex);

    // Sorts topology
    void sortTopology();

    // prints info about current vehicle and topology
    void printInfo();


protected:

    // Container: Discovered Road Signs
    std::vector<std::string> road_signs;

    // Container: Information about oncoming road signs
    std::vector<RoadSignInfo> info_signs;

    // Container: Information on blocked lanes
    std::vector<int> blockedLanes;

    // Container: Information about neighborhood telemetry per lane
    std::array<std::vector<VehicleCoord>, 5> nborCoord;

    // Container: Last Info about neighborhood
    std::map<int, VehicleCoord> nbor_info;

    // Cointainer: Information about inter-vehicular spacing
    std::array<std::vector<double>, 5> platoon_gaps;

    // Container: Which vehicles are leaders of their lanes
    std::map<int, int> laneLeaders;

    // Container: Last seen lane of each vehicle
    std::map<int, int> lastSeenLane;


    // Event: Topology print
    cMessage* printCheck;

    // Event: Joining Lane is safe to enter
    cMessage* safeJoinCheck;

    // Event: Self telemetry update
    cMessage* positionUpdateMsg;

    // Event: Road Signs Distance Calculation
    cMessage* signCheck;


    // Distance a vehicle should consider to move before a road constraint
    double alert_distance;

    // Am I in danger?
    bool inDanger;

    DistanceToBlockage blocking_distance;

    // Orientation of the current vehicle
    Orientation myDirection;

    // Orientation of the platoon a vehicle is currently part of
    Orientation platoonDirection;

};

#endif /* BASESELFORGANIZATIONAPP_H_ */
