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

#ifndef SIGNPLATOONINGAPP_H_
#define SIGNPLATOONINGAPP_H_

#include <algorithm>
#include <memory>

#include "veins/modules/application/platooning/apps/GeneralPlatooningApp.h"
#include "veins/modules/application/platooning/signs/VehicleCoord.h"

enum class PlatoonManeuver : size_t {
    JOIN_AT_BACK,
    JOIN_AT_FRONT,
    JOIN_IN_THE_MIDDLE,
    SPLIT,
    MERGE,
    MULTIPLE_JOIN_AT_BACK,
    MULTIPLE_JOIN_AT_FRONT,
    MULTIPLE_JOIN_IN_THE_MIDDLE
};

class SignPlatooningApp : public GeneralPlatooningApp {

public:
    SignPlatooningApp();

    /** override from GeneralPlatooningApp */
    virtual void initialize(int stage);

    virtual ~SignPlatooningApp();

    // Decides what to do when a road sign is detected
    void onRoadSignDetection(std::string sign);


protected:

    // Topology print event
    cMessage* topologyCheck;

    // Clear topology
    cMessage* topologyClear;

    // Future Lane is safe to join
    cMessage* safeJoinCheck;

    // Telemetry update event
    cMessage* telemetryCheck;

    // stores n'bors' telemetry
    std::deque<VehicleCoord> nborCoord[5];

    // stores n'bors' last 'seen' lane
    std::map<int, int> lastSeenLane;

    // Road Signs or Constraints
    enum class Sign : size_t
    {
        NARROW,
        TRIANGLE,
        STOP,
        REDUCE
    };

    // Last detected road sign
    Sign sign_detected;

    enum class Heading : size_t
    {
        NONE,
        NORTH,
        SOUTH,
        WEST,
        EAST
    };

    Heading myHeading;

    // Stores all constrained Lanes
    std::vector<std::string> constrainedLanes;

    // Stores all constrained Lanes Indexes
    std::vector<int> constrainedLaneIndexes;

    // Stores all elected leaders
    std::map<int, int> laneLeaders;

    // Stores all detected road_signs
//    std::map<Coord, std::string> road_signs;

    bool roadSignMode;

    /// Time and distance a vehicle is from road sign
    int roadSignPos;
    int lastSeenRoadSign;
    double timeToRoadSign;


protected:

///////////////////////////// TOPOLOGY ////////////////////////////////////////

    // Create new events
    virtual void handleSelfMsg(cMessage* msg);

    /** override from GeneralPlatooningApp */
    virtual void onPlatoonBeacon(const PlatooningBeacon* pb);

    // On new beacon, update topology
    void updateRoadTopology(const PlatooningBeacon* pb);

    // Update self telemetry
    void updatePosition();

    // Remove old telemetry @override
    void removeVehicleFromLane(int vehId, int lane) ;

    // Checks if another vehicle is heading to the same direction
    Heading getDirection(double angleRad);

    // Convert angle from rad to degree
    int convertAngleToDegrees(double angleRad);

///////////////////////////// FLAGS ///////////////////////////////////////////

    // Checks if vehicle is in safe lane
    bool isLaneSafe();

    // Checks if vehicle is leader of the current lane
    bool isLaneLeader();

    // Gets the safest lane to request help
    int getSafestLane();

    // Update leadership and sefety flags
    // TODO Migrate to enum sign_status
    void updateFlags();

///////////////////////////// PLATOON FORMATION ///////////////////////////////

    // Update Platoon Formation
    void setupFormation();

    // Set Platoon Formation after road sign detection
    void startManeuverFormation();

    void startSafeLeaderFormation();

    void startUnsafeLeaderFormation();

    void startSafeFollowerFormation();

    void startUnsafeFollowerFormation();

    // Send request to Safe Lane Leaders
    // NOTE It still does nothing
    void sendHelpRequest(int platoon_leader);


///////////////////////////// MANEUVER AUTHORIZATION //////////////////////////

    void sortTopology();

    // Based on to be joined platoon, checks if a vehicle can join
    bool isSafeToMove(PlatoonManeuver m, int laneIndex);

    double getPlatoonLength(int laneIndex);

    /// Shows the distance from platoon's tail
    double distanceFromPlatoonTail(int laneIndex);

    VehicleCoord getVehicleTopologyInfo(int lane, int position);

///////////////////////////// PRINTS //////////////////////////////////////////

    // Print info about the vehicle and the detected road sign
    void printRoadSignInfo(std::string sign);

    // Prints current road topology
    virtual void printRoadTopology(bool sorted = true); // prints topology

    void printInfo();


};

#endif /* SIGNPLATOONINGAPP_H_ */
