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

#include "BaseSelfOrganizationApp.h"

#include "veins/modules/application/platooning/maneuver/JoinManeuver.h"
#include "veins/modules/application/platooning/maneuver/DynamicJoin.h"

// Timers to adapt longitudinal position
#define TIMER_JOIN_AT_BACK       100
#define TIMER_JOIN_AT_FRONT      100
#define TIMER_JOIN_IN_THE_MIDDLE 100


/**
 * Defines maneuvers strategies based on existing gaps to self-organize vehicles in a Platoon
 */
class SelfOrganizationApp : public BaseSelfOrganizationApp {

public:

    SelfOrganizationApp()
    {
        maneuverControl = nullptr;
    }


    virtual ~SelfOrganizationApp();

    virtual void initialize(int stage);

    virtual void onPlatoonBeacon(const PlatooningBeacon* pb);

    virtual void onManeuverMessage(ManeuverMessage* mm);

    virtual int getBestAvailableEntryPosition(int joiner_id);

    void setManeuverStatus(int veh_id, PlatoonManeuver maneuver);

    virtual bool isJoinAllowed(int position = -1);

    virtual void adjustToManeuver();

    virtual void startJoinManeuver(int platoonId, int leaderId, int position);

    virtual void setupFormation();

    virtual void startManeuverFormation();

    virtual void sendUnicast(cPacket* msg, int destination);


protected:

    virtual void handleSelfMsg(cMessage* msg);

    // Maneuver: Object that controls maneuver status
    JoinManeuver* maneuverControl;

    // Event: Join maneuver adjustment
    cMessage* join_adjust;


};

#endif /* SELFORGANIZATIONAPP_H_ */
