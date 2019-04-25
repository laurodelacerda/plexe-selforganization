//
// Copyright (C) 2018 Julian Heinovski <julian.heinovski@ccs-labs.org>
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

#ifndef SIGNSCENARIO_H_
#define SIGNSCENARIO_H_

#include "veins/modules/application/platooning/scenarios/BaseScenario.h"
#include "veins/modules/application/platooning/signs/SignPlatooningApp.h"

class SignScenario : public BaseScenario {
public:

    SignScenario()
    {
//        startManeuver = nullptr;
        app = nullptr;
    };

    ~SignScenario();

//    static const int MANEUVER_TYPE = 12347;

    virtual void initialize(int stage);

protected:

    void sendUnicast(cPacket* msg, int destination);

    void handleSelfMsg(cMessage* msg);

    void prepareManeuverCars(int platoonLane);

    void setupFormation();

    void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details);

//     message used to start the maneuver
//    cMessage* startManeuver;

    // application layer, used to stop the simulation
    SignPlatooningApp* app;

    static const simsignalwrap_t mobilityStateChangedSignal;

    std::string lastSignId;
};

#endif
