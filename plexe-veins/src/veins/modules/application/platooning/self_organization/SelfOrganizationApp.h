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

#include "veins/modules/application/platooning/signs/TopologyMap.h"

class TopologyMap;



class SignPlatooningApp : public GeneralPlatooningApp {

public:

    SignPlatooningApp();

    /** override from GeneralPlatooningApp */
    virtual void initialize(int stage);

    virtual ~SignPlatooningApp();

    // Decides what to do when a road sign is detected
    void onRoadSignDetection(std::string sign);


    int getCurrentLaneIndex()
    {
        return traciVehicle->getLaneIndex();
    }

    int getVehicleId()
    {
        return myId;
    }


protected:

    // Topology print event
    cMessage* topologyCheck;

    // Clear topology
    cMessage* topologyClear;

    // Future Lane is safe to join
    cMessage* safeJoinCheck;

    // Telemetry update event
    cMessage* positionUpdateMsg;

    // Map of platoon topology
    TopologyMap* topologyMap;

protected:


    // Events

    // Detect self events
    virtual void handleSelfMsg(cMessage* msg);

    /** override from GeneralPlatooningApp */
    virtual void onPlatoonBeacon(const PlatooningBeacon* pb);

    // Update leadership and safety lane flags
    void updateFlags();


    // Platoon Formation

    // Update Platoon Formation
    void setupFormation();

    // Set Platoon Formation after road sign detection
    void startManeuverFormation();

    // Start safe leader movements
    // TODO Transfer this class to maneuver class
    void startSafeLeaderFormation();

    // Start unsafe leader movements
    // TODO Transfer this class to maneuver class
    void startUnsafeLeaderFormation();

    // Start safe follower movements
    // TODO Transfer this class to maneuver class
    void startSafeFollowerFormation();

    // Start unsafe follower movements
    // TODO Transfer this class to maneuver class
    void startUnsafeFollowerFormation();

    // Prints current road topology
    virtual void printRoadTopology();

    // prints info about current vehicle
    void printInfo();

};

#endif /* SIGNPLATOONINGAPP_H_ */
