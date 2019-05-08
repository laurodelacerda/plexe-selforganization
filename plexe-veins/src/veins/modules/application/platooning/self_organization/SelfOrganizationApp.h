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

#ifndef SELFORGANIZATIONAPP_H_
#define SELFORGANIZATIONAPP_H_

#include <algorithm>
#include <memory>

#include "../self_organization/VehicleCoord.h"
#include "TelemetryMap.h"
#include "veins/modules/application/platooning/apps/GeneralPlatooningApp.h"
#include "veins/modules/application/platooning/self_organization/TelemetryMap.h"

class TelemetryMap;



class SelfOrganizationApp : public GeneralPlatooningApp {

public:

    SelfOrganizationApp();

    /** override from GeneralPlatooningApp */
    virtual void initialize(int stage);

    virtual ~SelfOrganizationApp();

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

    bool isInDanger() const {
        return inDanger;
    }

    void setInDanger(bool inDanger) {
        this->inDanger = inDanger;
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
    TelemetryMap* telemetryMap;

    bool inDanger;

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

    // prints info about current vehicle and topology
    void printInfo();

};

#endif /* SELFORGANIZATIONAPP_H_ */
