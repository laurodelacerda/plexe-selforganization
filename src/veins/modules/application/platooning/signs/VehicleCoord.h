/*
 * VehicleCoord.h
 *
 *  Created on: Jan 28, 2019
 *      Author: lauro
 */

#ifndef VEHICLECOORD_H_
#define VEHICLECOORD_H_

#include "veins/base/utils/Coord.h"

class VehicleCoord
{

public:

    VehicleCoord(){};

    VehicleCoord(int id, int lane, Veins::Coord coord, int lenght = 0)
    : vehId(id),
      vehLane(lane),
      vehCoord(coord)
    {
        lastUpdate = simTime();
    };

    virtual ~VehicleCoord(){};


    int getId()
    {
        return vehId;
    }

    int getLane()
    {
        return vehLane;
    }

    Veins::Coord getCoord()
    {
        return vehCoord;
    }

    simtime_t getLastUpdate()
    {
        return lastUpdate;
    }

    int getLength() const {
        return vehLenght;
    }

private:

    int vehId;
    int vehLane;
    Veins::Coord vehCoord;
    simtime_t lastUpdate;
    int vehLenght;

};

#endif /* SRC_VEINS_MODULES_APPLICATION_PLATOONING_UTILITIES_VEHICLECOORD_H_ */
