/*
 * VehicleCoord.h
 *
 *  Created on: Jan 28, 2019
 *      Author: lauro
 */

#ifndef VEHICLECOORD_H_
#define VEHICLECOORD_H_

#include "veins/base/utils/Coord.h"
#include "veins/modules/application/platooning/CC_Const.h"
#include "veins/modules/application/platooning/messages/PlatooningBeacon_m.h"

struct VehicleCoord
{
    int vehicleId ;
    int vehicleLaneIndex ;
    int vehicleAngle ;
    double vehicleSpeed ;
    double vehicleLength ;
    Veins::Coord vehicleCoord ;
    simtime_t timestamp ;

    VehicleCoord()
    {
        vehicleId = -1;
        vehicleLaneIndex = -1;
        vehicleAngle = -1;
        vehicleLength = -1.0;
        vehicleSpeed = 0.0;
        vehicleCoord = Veins::Coord();
        timestamp = SIMTIME_ZERO;
    }

    VehicleCoord(int id, int lane, Veins::Coord c, int length = 0)
    : vehicleId(id),
      vehicleLaneIndex(lane),
      vehicleLength(length),
      vehicleSpeed(0.0),
      vehicleCoord(c)
    {
        timestamp = simTime();
    }

    VehicleCoord(int id, int lane, int posX, int posY, int length = 0)
    : vehicleId(id),
      vehicleLaneIndex(lane),
      vehicleLength(length),
      vehicleSpeed(0.0),
      vehicleCoord(Veins::Coord(posX, posY))
    {
        timestamp = simTime();
    }

    void fromPacket(const PlatooningBeacon* pb)
    {
        vehicleId        = pb->getVehicleId();
        vehicleLaneIndex = pb->getLaneIndex();
        vehicleAngle     = pb->getAngle();
        vehicleLength    = pb->getLength();
        vehicleSpeed     = pb->getSpeed();
        vehicleCoord     = Veins::Coord(pb->getPositionX(), pb->getPositionY());
        timestamp        = simTime();
    }

    void fromPlexeData(int id, int laneIndex, Plexe::VEHICLE_DATA& data)
    {
        vehicleId        = id ;         // TODO Encontrar uma maneira de obter essa info em Plexe::VEHICLE_DATA
        vehicleLaneIndex = laneIndex ;  // TODO Encontrar uma maneira de obter essa info em Plexe::VEHICLE_DATA
        vehicleAngle     = data.angle;
        vehicleLength    = data.length;
        vehicleSpeed     = data.speed;
        vehicleCoord     = Veins::Coord(data.positionX, data.positionY);
        timestamp        = simTime();
    }

    virtual ~VehicleCoord(){};


    int getId() const
    {
        return vehicleId;
    }

    int getLaneIndex() const
    {
        return vehicleLaneIndex;
    }

    Veins::Coord getCoord() const
    {
        return vehicleCoord;
    }

    simtime_t getTimestamp() const
    {
        return timestamp;
    }

    int getLength() const
    {
        return vehicleLength;
    }

    int getAngle() const
    {
        return vehicleAngle;
    }

    double getPositionX() const
    {
        return vehicleCoord.x;
    }

    double getPositionY() const
    {
        return vehicleCoord.y;
    }

};

#endif /* SRC_VEINS_MODULES_APPLICATION_PLATOONING_UTILITIES_VEHICLECOORD_H_ */
