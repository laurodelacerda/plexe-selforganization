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
#include "veins/modules/application/platooning/self_organization/TelemetryMap.h"


/**
 * General purpose application for Self-Organizing Platoons
 * This module is focused on interaction (by generating communication among vehicles)
 * and road sign detection.
 *
 * @see BaseApp
 * @see Maneuver
 */
class BaseSelfOrganizationApp : public GeneralPlatooningApp {

public:

    /** c'tor for BaseSelfOrganizationApp */
    BaseSelfOrganizationApp()
        : map(nullptr),
          inDanger(false)
    {
    }

    /** d'tor for BaseSelfOrganizationApp */
    virtual ~BaseSelfOrganizationApp();

    virtual void initialize(int stage);

//    void setPlatoonFormation(const std::vector<int>& f)
//    {
//        positionHelper->setPlatoonFormation(f);
//    }



//    int getPlatoonId()
//    {
//        return positionHelper->getPlatoonId();
//    }
//
//    int getLeaderId()
//    {
//        return positionHelper->getLeaderId();
//    }


    // Decides what to do when a road sign is detected
    void onRoadSignDetection(std::string sign_id, std::string sign_type, int lane_index, double range);

    // Monitors vehicle status in relation to road signs
    void onRoadSignTracking();



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

    // Detect self events
    virtual void handleSelfMsg(cMessage* msg);

    /** override from GeneralPlatooningApp */
    virtual void onPlatoonBeacon(const PlatooningBeacon* pb) override;

    // Update lane leadership and safety flags
    void updateFlags();


    // Platoon Formation

    // Update Platoon Formation
    virtual void setupFormation(){}

    // Set Platoon Formation after road sign detection
    virtual void startManeuverFormation(){}

    // Start safe leader movements
    virtual void startSafeLeaderFormation(){}

    // Start unsafe leader movements
    virtual void startUnsafeLeaderFormation(){}

    // Start safe follower movements
    virtual void startSafeFollowerFormation(){}

    // Start unsafe follower movements
    virtual void startUnsafeFollowerFormation(){}

    // prints info about current vehicle and topology
    void printInfo();


protected:

    // Topology print event
    cMessage* topologyCheck;

    // Clear topology
    cMessage* topologyClear;

    // Future Lane is safe to join
    cMessage* safeJoinCheck;

    // Telemetry update event
    cMessage* positionUpdateMsg;

    cMessage* signCheck;

    double alert_distance;

    std::vector<std::string> road_signs;

    // Map of platoon topology
    TelemetryMap* map;

    bool inDanger;

};

#endif /* BASESELFORGANIZATIONAPP_H_ */
