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

#ifndef JOINATBACKSIGN_H_
#define JOINATBACKSIGN_H_

#include "BaseSelfOrganizationApp.h"
#include "veins/modules/application/platooning/maneuver/JoinAtBack.h"

using namespace Veins;

class JoinAtBackSign : public JoinAtBack

{

public:

    JoinAtBackSign(GeneralPlatooningApp* app);
    virtual ~JoinAtBackSign();


    virtual void startManeuver(const void* parameters) override;

    virtual void abortManeuver() override;

    virtual void onPlatoonBeacon(const PlatooningBeacon* pb) override;

    virtual void handleJoinPlatoonRequest(const JoinPlatoonRequest* msg) override;

    virtual void handleJoinPlatoonResponse(const JoinPlatoonResponse* msg) override;

    virtual void handleMoveToPosition(const MoveToPosition* msg) override;

    virtual void handleMoveToPositionAck(const MoveToPositionAck* msg) override;

    virtual void handleJoinFormation(const JoinFormation* msg) override;

    virtual void handleJoinFormationAck(const JoinFormationAck* msg) override;

};

#endif /* JOINATBACKSIGN_H_ */
