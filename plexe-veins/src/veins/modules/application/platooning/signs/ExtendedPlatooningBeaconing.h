//
// Copyright (c) 2012-2018 Michele Segata <segata@ccs-labs.org>
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

#ifndef EXTENDEDPLATOONINGBEACONING_H_
#define EXTENDEDPLATOONINGBEACONING_H_

#include "veins/modules/application/platooning/protocols/BaseProtocol.h"

#include "veins/modules/application/platooning/signs/ExtendedPlatooningBeacon_m.h"

class ExtendedPlatooningBeaconing : public BaseProtocol {
protected:
    virtual void handleSelfMsg(cMessage* msg);
    virtual void messageReceived(PlatooningBeacon* pkt, UnicastMessage* unicast);

    void sendPlatooningMessage(int destinationAddress);

public:

    ExtendedPlatooningBeaconing(){}
    virtual ~ExtendedPlatooningBeaconing(){}

    virtual void initialize(int stage);

//    void startBeaconing();
//    void stopBeaconing();

public:

    // id for extended beacon message
    static const int EXTENDED_BEACON = 12346;
};

#endif /* EXTENDEDPLATOONINGBEACONING_H_ */
