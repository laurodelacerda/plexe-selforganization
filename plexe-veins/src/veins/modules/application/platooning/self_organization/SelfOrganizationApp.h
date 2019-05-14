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

#ifndef MANEUVERMANAGER_H_
#define MANEUVERMANAGER_H_

#include "BaseSelfOrganizationApp.h"
//#include "veins/modules/mobility/traci/TraCIMobility.h"

#include "veins/modules/application/platooning/maneuver/JoinManeuver.h"
#include "veins/modules/application/platooning/maneuver/JoinAtBack.h"
#include "veins/modules/application/platooning/self_organization/JoinAtBackSign.h"

/**
 * Defines maneuvers strategies based on existing gaps to self-organize vehicles in a Platoon
 */
class SelfOrganizationApp : public BaseSelfOrganizationApp {

public:

    SelfOrganizationApp()
    :   maneuverInCourse(PlatoonManeuver::NONE)
    {
        joinManeuver = nullptr;
    }


    virtual ~SelfOrganizationApp();

    virtual void initialize(int stage);


    virtual void onManeuverMessage(ManeuverMessage* mm);

    PlatoonManeuver getManeuverInCourse() const
    {
        return maneuverInCourse;
    }

    void setManeuverInCourse(PlatoonManeuver maneuver)
    {
        this->maneuverInCourse = maneuver;
    }


    virtual void startJoinManeuver(int platoonId, int leaderId, int position);

    virtual void abortJoinManeuver();

    virtual void startSplitManeuver();

    virtual void abortSplitManeuver();

    virtual void startMultipleJoinManeuver();

    virtual void abortMultipleJoinManeuver();


    virtual void setupFormation();

    virtual void startManeuverFormation();

    virtual void startSafeLeaderFormation();

    virtual void startUnsafeLeaderFormation();

    virtual void startSafeFollowerFormation();

    virtual void startUnsafeFollowerFormation();


    virtual void sendUnicast(cPacket* msg, int destination);



protected:

    // Detect self events
//    virtual void handleSelfMsg(cMessage* msg);


    PlatoonManeuver maneuverInCourse;

    bool inManeuver;

    JoinManeuver* joinManeuver;

};

#endif /* MANEUVERMANAGER_H_ */
