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

#include "BaseSelfOrganizationApp.h"
#include "veins/modules/application/platooning/protocols/BaseProtocol.h"

using namespace Veins;

Define_Module(BaseSelfOrganizationApp);

void BaseSelfOrganizationApp::initialize(int stage)
{
    GeneralPlatooningApp::initialize(stage);

    if (stage == 1)
    {

        protocol->registerApplication(ROAD_SIGN_TYPE, gate("lowerLayerIn"), gate("lowerLayerOut"), gate("lowerControlIn"), gate("lowerControlOut"));

        SimTime rounded = SimTime(floor(simTime().dbl() * 1000 + 100), SIMTIME_MS);

        printCheck = new cMessage("printCheck");
        scheduleAt(rounded + 2, printCheck);

        positionUpdateMsg = new cMessage("positionUpdateMsg");
        scheduleAt(rounded, positionUpdateMsg);

        safeJoinCheck = new cMessage("safeJoinCheck");

        signCheck = new cMessage("signCheck");

        maneuverStatsCheck = new cMessage("maneuverStatsCheck");
        scheduleAt(rounded, maneuverStatsCheck);

    }

}

BaseSelfOrganizationApp::~BaseSelfOrganizationApp()
{

    cancelAndDelete(printCheck);
    printCheck = nullptr;
    cancelAndDelete(positionUpdateMsg);
    positionUpdateMsg = nullptr;
    cancelAndDelete(safeJoinCheck);
    safeJoinCheck = nullptr;
    cancelAndDelete(signCheck);
    signCheck = nullptr;
    cancelAndDelete(maneuverStatsCheck);
    maneuverStatsCheck = nullptr;
//    delete map ;

}

void BaseSelfOrganizationApp::handleSelfMsg(cMessage* msg)
{
    // Every 100 ms we update vehicle position in container
    if (msg == positionUpdateMsg)
    {
        Plexe::VEHICLE_DATA data;
        traciVehicle->getVehicleData(&data);
        updateMyPosition(myId, traciVehicle->getLaneIndex(), data);

//        updateFlags(); Não deveria atualizar flags para não atrapalhar ações de manobra
        scheduleAt(simTime() + SimTime(100, SIMTIME_MS), positionUpdateMsg);
    }

    if (msg == printCheck)
    {
        printInfo();
        scheduleAt(simTime() + SimTime(100, SIMTIME_MS) + 2, printCheck);
    }

    if (msg == safeJoinCheck)
    {
        cancelEvent(safeJoinCheck);
        startManeuverFormation();
    }

    if (msg == signCheck)
    {
        onRoadSignTracking();
    }

    if (msg == maneuverStatsCheck)
    {
    	collectStats();
    	scheduleAt(simTime() + SimTime(100, SIMTIME_MS), maneuverStatsCheck);
    }

    GeneralPlatooningApp::handleSelfMsg(msg);
}

void BaseSelfOrganizationApp::onPlatoonBeacon(const PlatooningBeacon* pb)
{
    // TODO Check if message has Extended Beaconing Kind
//    map->updateTelemetryMap(pb);
    updateTelemetryMap(pb);

    // Send to lower layers, including maneuvers
    GeneralPlatooningApp::onPlatoonBeacon(pb);

//    https://sumo.dlr.de/wiki/Definition_of_Vehicles,_Vehicle_Types,_and_Routes#Lane-Changing_Models
//    TODO Find some place to call this method once for a while
	traciVehicle->setLaneChangeMode(512);
}

void BaseSelfOrganizationApp::updateTelemetryMap(const PlatooningBeacon* pb)
{
    VehicleCoord vCoord = VehicleCoord();
    vCoord.fromPacket(pb);

    int nb_id   = pb->getVehicleId();
    int nb_lane = pb->getLaneIndex();
    Orientation nb_head = calculateDirection(pb->getAngle());

    int lastSeenAt;

    // n'bors at some lane Direction to the same direction
    if ((nb_lane != -1) && (myDirection == nb_head))
    {
        if(lastSeenLane.count(nb_id) == 0) // New node
        {
            nborCoord[nb_lane].push_back(vCoord);
        }
        else                               // Known node
        {
            lastSeenAt = lastSeenLane[nb_id];
            removeVehicleFromLane(nb_id, lastSeenAt); // Remove node from old lane in topology
            nborCoord[nb_lane].push_back(vCoord);
        }

        lastSeenLane[nb_id] = nb_lane;
    }

    nbor_info[nb_id] = vCoord;

}

void BaseSelfOrganizationApp::updateMyPosition(int vehicleId, int laneIndex, Plexe::VEHICLE_DATA& data)
{
    VehicleCoord vCoord = VehicleCoord();
    vCoord.fromPlexeData(vehicleId, laneIndex, data);

//    VehicleCoord vCoord = VehicleCoord(vehicleId, laneIndex, app->getMobility()->getCurrentPosition(), app->getTraciVehicle()->getLength());

    int lastSeenAt;

    if(lastSeenLane.count(vehicleId) == 0)
    {
        nborCoord[laneIndex].push_back(vCoord);
    }
    else
    {
        lastSeenAt = lastSeenLane[vehicleId];
        removeVehicleFromLane(vehicleId, lastSeenAt);
        nborCoord[laneIndex].push_back(vCoord);
    }

    lastSeenLane[vehicleId] = laneIndex;
    nbor_info[myId] = vCoord;

    myDirection = calculateDirection(vCoord.getAngle());
    platoonDirection = calculatePlatoonDirection(laneIndex);
}

void BaseSelfOrganizationApp::onRoadSignDetection(std::string sign_id, std::string sign_type, int lane_index, double range)
{
    // Adiciona nova sinalização detectada
    addNewRoadSign(sign_id, sign_type, lane_index);

    // Informa a todos os vizinhos sobre a nova sinalização
    Veins::TraCICommandInterface::Poi poi = traci->poi(sign_id);
    Coord sign_pos = poi.getPosition();

    RoadSignWarning* warning = new RoadSignWarning("RoadSignWarning");
    fillRoadSignMessage(warning, sign_id, sign_type, lane_index, sign_pos.x, sign_pos.y);
    sendUnicast(warning, -1);
}

void BaseSelfOrganizationApp::onRoadSignWarningMsg(RoadSignWarning* msg)
{
    addNewRoadSign(msg->getRoadSignId(), msg->getRoadSignType(), msg->getLaneIndex());
}

void BaseSelfOrganizationApp::addNewRoadSign(std::string sign_id, std::string sign_type, int lane_index)
{
    // Adiciona ao container de placas de sinalização
    if (std::find(road_signs.begin(), road_signs.end(), sign_id) == road_signs.end())
    {
        road_signs.push_back(sign_id);

        // Adiciona dados da nova placa de sinalização
        RoadSignInfo info;
        info.sign_id     = sign_id;
        info.sign_type   = sign_type;
        info.lane_index  = lane_index;
        info.same_lane   = traciVehicle->getLaneIndex() == lane_index ;
        info.rule_active = true;
        info_signs.push_back(info);
    }

    // Adiciona ao container de ruas bloqueadas
    if (std::find(blockedLanes.begin(), blockedLanes.end(), lane_index) == blockedLanes.end())
        blockedLanes.push_back(lane_index);

    // Acompanha aproximação com sinalização
    if (!signCheck->isScheduled())
    {
        updateFlags();
        startManeuverFormation();
        scheduleAt(simTime() + SimTime(TIMER_ROAD_SIGN, SIMTIME_MS), signCheck);
    }

}

void BaseSelfOrganizationApp::onRoadSignTracking()
{
//    Enter_Method_Silent();
    Plexe::VEHICLE_DATA data;
    traciVehicle->getVehicleData(&data);

    std::vector<double> distances;

    for(RoadSignInfo i : info_signs)
    {
        Veins::TraCICommandInterface::Poi poi = traci->poi(i.sign_id);

        double dist = traci->getDistance(Coord(data.positionX, data.positionY), poi.getPosition(), false);

        if (dist != DBL_MAX)
            distances.push_back(dist);
//        else // Road Sign is behind
//        {
//
//        }

    }

    double nearest_blockage = 0;

    if (distances.size() > 0)
        nearest_blockage = *std::min_element(distances.begin(), distances.end());
    else
        blocking_distance = DistanceToBlockage::NONE;

//    if (nearest_blockage > DISTANCE_SAFE)
//        blocking_distance = DistanceToBlockage::D_SAFE;
//    else if ((nearest_blockage < DISTANCE_CAUTIOUS) and (nearest_blockage > DISTANCE_DANGEROUS))
//        blocking_distance = DistanceToBlockage::D_CAUTIOUS;
//    else if (nearest_blockage < DISTANCE_DANGEROUS)
//        blocking_distance = DistanceToBlockage::D_DANGEROUS;
//    else
//        blocking_distance = DistanceToBlockage::NONE;


    bool blocked_lane = std::find(blockedLanes.begin(),
    		  	  	   	   	   	  blockedLanes.end(),
								  traciVehicle->getLaneIndex()) != blockedLanes.end();

//    if ((blocked_lane) and (blocking_distance != DistanceToBlockage::NONE))
	if (blocked_lane)
        inDanger = true;
    else
    {
        inDanger = false;
        cancelEvent(signCheck);
    }

    // Check again
    if (!signCheck->isScheduled())
		scheduleAt(simTime() + SimTime(TIMER_ROAD_SIGN, SIMTIME_MS), signCheck);

}

void BaseSelfOrganizationApp::fillRoadSignMessage(RoadSignWarning* msg, std::string sign_id, std::string sign_type, int lane_index, double posX, double posY)
{
    msg->setKind(ROAD_SIGN_TYPE);
    msg->setVehicleId(positionHelper->getId());
    msg->setExternalId(positionHelper->getExternalId().c_str());
    msg->setPlatoonId(positionHelper->getId());

    msg->setRoadSignId(sign_id.c_str());
    msg->setRoadSignType(sign_type.c_str());
    msg->setLaneIndex(lane_index);
    msg->setXPos(posX);
    msg->setYPos(posY);

}

void BaseSelfOrganizationApp::updateFlags()
{
    bool lane_leader = isLaneLeader(myId);
    bool lane_safe   = isLaneSafe(myId);

    if ((lane_leader) && (lane_safe)) 			// Safe Leader
    {
//        role = PlatoonRole::LEADER;
        inDanger = false;
        findHost()->getDisplayString().updateWith("r=8,green");
    }
    else if ((lane_leader) && (!lane_safe))		// Unsafe Leader
    {
        role = PlatoonRole::UNSAFE_LEADER;
        inDanger = true;
        findHost()->getDisplayString().updateWith("r=8,red");
    }
    else if ((!lane_leader) && (lane_safe)) 	// Safe Follower
    {
        role = PlatoonRole::FOLLOWER;
        inDanger = false;
        findHost()->getDisplayString().updateWith("r=8,blue");
    }
    else
    {
        role = PlatoonRole::UNSAFE_FOLLOWER;	// Unsafe Follower
        inDanger = true;
        findHost()->getDisplayString().updateWith("r=0,gray");
    }
}

void BaseSelfOrganizationApp::handleLowerMsg(cMessage* msg)
{
    UnicastMessage* unicast = check_and_cast<UnicastMessage*>(msg);

    cPacket* enc = unicast->getEncapsulatedPacket();
    ASSERT2(enc, "received a UnicastMessage with nothing inside");

    if (enc->getKind() == ROAD_SIGN_TYPE)
    {
        RoadSignWarning* rs  = check_and_cast<RoadSignWarning*>(unicast->decapsulate());
        onRoadSignWarningMsg(rs);
        delete rs;
    }
    else {
        GeneralPlatooningApp::handleLowerMsg(msg);
    }
}

void BaseSelfOrganizationApp::calculatePlatoonGaps(std::vector<double> &gaps, int lane_index, double veh_lenght)
{
    // Number of gaps is equal to number of vehicles plus 1
    if (nborCoord[lane_index].size() > 0)
    {
        for(int j = 0; j < nborCoord[lane_index].size() + 1; j++)
        {
            double iv_gap;

            // Compute distances for front and back gaps
            if (j == 0)
            {
                // get current position
                Coord currentPosition(nborCoord[lane_index].at(0).getPositionX(), nborCoord[lane_index].at(0).getPositionY(), 0);

                // compute distance
                // TODO adds platoon default security gap
//                iv_gap = currentPosition.distance(currentPosition) + nborCoord[lane_index].at(j).getLength();
                iv_gap = veh_lenght;

            }
            if ((j > 0) and (j < nborCoord[lane_index].size()))
            {
                // get front vehicle position
//                Coord frontPosition(nborCoord[i].at(j-1).getPositionX(), nborCoord[i].at(j-1).getPositionY(), 0);

                // get current position
//                Coord currentPosition(nborCoord[i].at(j).getPositionX(), nborCoord[i].at(j).getPositionY(), 0);

//                    iv_gap = fabs(app->getTraci()->getDistance(nborCoord[i].at(j).getCoord(), nborCoord[i].at(j-1).getCoord(), false));
                iv_gap = fabs(traci->getDistance(nborCoord[lane_index].at(j).getCoord(), nborCoord[lane_index].at(j-1).getCoord(), false));

                // Measuring influence of car's length
                iv_gap -= (nborCoord[lane_index].at(j-1).getLength() + nborCoord[lane_index].at(j).getLength()) / 2;

                // compute distance
                // TODO adds platoon default security gap
//                iv_gap = currentPosition.distance(frontPosition) - nborCoord[i].at(j).getLength();
            }
            else if (j == nborCoord[lane_index].size())
            {
                // get current position
//                    Coord currentPosition(nborCoord[i].at(j-1).getCoord());

                // compute distance
                // TODO adds platoon default security gap
//                iv_gap = nborCoord[lane_index].at(j-1).getLength();
                iv_gap = veh_lenght;
            }
            gaps.push_back(iv_gap);
        }
    }
}

double BaseSelfOrganizationApp::calculateGap(int lane_index, int position)
{
    int platoon_size = nborCoord[lane_index].size();
    double platoon_gap;

    if (position == -1)
    {
        platoon_gap = calculateDistanceToPlatoonMember(lane_index, platoon_size - 1);
        /// Vehicle should be at a safe distance from platoon's tail to perform maneuver
//        std::cout << "\nVehicle " << myId << " distance from platoon " << nborCoord[getSafestLaneIndex()].at(0).getId() << "'s tail is " << platoon_dist << std::endl;
    }
    else if (position == 0) // Gap to perform PlatoonManeuver::JOIN_AT_FRONT
    {
        platoon_gap = calculateDistanceToPlatoonMember(lane_index, 0);
    }
    else if ((position > 0) and (position < platoon_size))  // Gap to perform PlatoonManeuver::JOIN_IN_THE_MIDDLE
    {
        platoon_gap = calculateDistancePlatoonMembers(lane_index, position);
    }

    return platoon_gap;

//    if (platoon_gap > 0) // Vehicle is behind platoon's tail
//        return true;
//    else                  // Vehicle is in front of platoon's tail
//        return false;
}

int BaseSelfOrganizationApp::convertAngleToDegrees(double angleRad)
{
    double angleDeg;

    angleDeg = (180 / 3.14) * angleRad;
    angleDeg = fmod(angleDeg, 360);

    if (angleDeg < 0)
        angleDeg += 360;

    return angleDeg;
}

Orientation BaseSelfOrganizationApp::calculateDirection(double angleRad)
{
    int angleDeg = convertAngleToDegrees(angleRad);

    Orientation orientation;

    switch(angleDeg)
    {
    case 0 ... 44:
        orientation = Orientation::EAST;
        break;

    case 45 ... 134:
        orientation = Orientation::NORTH;
        break;

    case 135 ... 224:
        orientation = Orientation::WEST;
        break;

    case 225 ... 314:
        orientation = Orientation::SOUTH;
        break;

    case 315 ... 359:
        orientation = Orientation::EAST;
        break;
    }

    return orientation;
}

Orientation BaseSelfOrganizationApp::calculatePlatoonDirection(int lane_index)
{
    double sum_angle = 0;
    double num_veh = nborCoord[lane_index].size();

    if (num_veh > 0)
        for (VehicleCoord c : nborCoord[lane_index]) sum_angle += c.getAngle();
    else
        return Orientation::NONE;

    return calculateDirection(sum_angle/num_veh);
}

void BaseSelfOrganizationApp::removeVehicleFromLane(int vehicleId, int laneIndex)
{
    for (auto it = nborCoord[laneIndex].begin(); it != nborCoord[laneIndex].end(); )
    {

        if ((*it).getId() == vehicleId)
            it = nborCoord[laneIndex].erase(it);
        else
            ++it;
    }
}

bool comparePosNorth (VehicleCoord a, VehicleCoord b)
{
    return (a.getCoord().y > b.getCoord().y);
}

bool comparePosSouth (VehicleCoord a, VehicleCoord b)
{
    return (a.getCoord().y < b.getCoord().y);
}

bool comparePosEast (VehicleCoord a, VehicleCoord b)
{
    return (a.getCoord().x > b.getCoord().x);
}

bool comparePosWest (VehicleCoord a, VehicleCoord b)
{
    return (a.getCoord().x < b.getCoord().x);
}

void BaseSelfOrganizationApp::sortTopology()
{
    for(int i = 0; i < (sizeof(nborCoord)/sizeof(nborCoord[0])); i++)
    {
        if (myDirection == Orientation::NORTH)
            std::sort(nborCoord[i].begin(), nborCoord[i].end(), comparePosNorth);
        else if (myDirection == Orientation::SOUTH)
            std::sort(nborCoord[i].begin(), nborCoord[i].end(), comparePosSouth);
        else if (myDirection == Orientation::EAST)
            std::sort(nborCoord[i].begin(), nborCoord[i].end(), comparePosEast);
        else if (myDirection == Orientation::WEST)
            std::sort(nborCoord[i].begin(), nborCoord[i].end(), comparePosWest);
    }
}

double BaseSelfOrganizationApp::getPlatoonLength(int laneIndex)
{
    double leaderPos;
    double tailPos;

    if (nborCoord[laneIndex].size() == 0)
    {
        return -1;
    }
    else
    {
        VehicleCoord leader_data = getVehicleTopologyInfo(laneIndex, 0);

        int size = nborCoord[laneIndex].size();

        VehicleCoord tail_data   = getVehicleTopologyInfo(laneIndex, size - 1);

        double tail_length = tail_data.getLength();

        if ((myDirection == Orientation::NORTH) or (myDirection == Orientation::SOUTH))
        {
            leaderPos = leader_data.getCoord().y;
            tailPos   = tail_data.getCoord().y;
        }
        else if ((myDirection == Orientation::EAST) or (myDirection == Orientation::WEST))
        {
            leaderPos = leader_data.getCoord().x;
            tailPos   = tail_data.getCoord().x;
        }

        return std::fabs(leaderPos - tailPos) + tail_length;
    }

}

double BaseSelfOrganizationApp::calculateDistanceToPlatoonMember(int lane_index, int position)
{
    int platoon_size = nborCoord[lane_index].size();

    if ((platoon_size == 0) or (position >= platoon_size))
        return -1;
    else
    {
        Plexe::VEHICLE_DATA data;
        traciVehicle->getVehicleData(&data);
        Coord my_pos(data.positionX, data.positionY);

        double distance;

        Coord  nbor_pos = nborCoord[lane_index].at(position).getCoord();
        double nbor_len = nborCoord[lane_index].at(position).getLength();

        if (position == -1)     // Gap to perform PlatoonManeuver::JOIN_AT_BACK
            distance = traci->getDistance(my_pos, nbor_pos, false) + ((nbor_len + data.length)/2); // + safety_guard
        else if (position == 0) // Gap to perform PlatoonManeuver::JOIN_AT_FRONT
            distance = traci->getDistance(my_pos, nbor_pos, false) + ((nbor_len + data.length)/2); // + safety_guard
        else                    // Gap to perform PlatoonManeuver::JOIN_IN_THE_MIDDLE
            distance = traci->getDistance(my_pos, nbor_pos, false) - ((nbor_len + data.length)/2); // + safety_guard
    }

}

double BaseSelfOrganizationApp::calculateDistancePlatoonMembers(int lane_index, int position)
{
    int platoon_size = nborCoord[lane_index].size();
    double distance;

    if ((platoon_size == 0) or (position >= platoon_size))
        return -1;
    else
    {
        VehicleCoord front = nborCoord[lane_index].at(position - 1);
        VehicleCoord back  = nborCoord[lane_index].at(position);

        //        return app->getTraci()->getDistance(front.getCoord(), back.getCoord(), true);
        distance = traci->getDistance(front.getCoord(), back.getCoord(), false) - ((front.getLength() + back.getLength())/2);

        return distance;
    }
}

VehicleCoord BaseSelfOrganizationApp::getVehicleTopologyInfo(int laneIndex, int position)
{
    // WARNING Verificar se é uma lane trafegável
    return nborCoord[laneIndex].at(position);
}

bool BaseSelfOrganizationApp::isLaneSafe(int vehicleId)
{
    if (std::find(blockedLanes.begin(), blockedLanes.end(), lastSeenLane[vehicleId]) != blockedLanes.end())
        return false;
    else
        return true;
}

bool BaseSelfOrganizationApp::isLaneLeader(int vehicleId)
{
//    int lane_index = app->getCurrentLaneIndex();
    int lane_index = traciVehicle->getLaneIndex();

    this->sortTopology();

    for(int i = 0; i < (sizeof(nborCoord)/sizeof(nborCoord[0])); i++)
    {
//        std::sort(nborCoord[i].begin(), nborCoord[i].end(), comparePosEastA);

        if (nborCoord[i].size() > 0)
            laneLeaders[i] = nborCoord[i].at(0).getId();
        else
            laneLeaders[i] = -99;
    }

    if (laneLeaders[lane_index] == vehicleId)
        return true;
    else
        return false;
}

int BaseSelfOrganizationApp::getSafestLaneIndex()
{
//    int current_lane = app->getCurrentLaneIndex();
    int current_lane = traciVehicle->getLaneIndex();

    std::vector<int> safeLanes;

    for (auto const& it : laneLeaders)
    {
        /// BUG Vehicles are finding their own lanes
        /// TODO Get Index of Lanes in SUMO source file
        if (std::find(blockedLanes.begin(), blockedLanes.end(), it.first) == blockedLanes.end())
            safeLanes.push_back(it.first);
    }

    // Find nearest and safest lane
    int safest_lane = 100;
    int least_distance;
    int aux = 100;

    for (int j = 0; j < safeLanes.size(); ++j)
    {
        least_distance = std::abs(current_lane - safeLanes.at(j));

        if (least_distance < aux)
        {
            aux = least_distance;
            safest_lane = safeLanes.at(j);
        }

    }

    return safest_lane;
}

int BaseSelfOrganizationApp::getSafestLaneLeader()
{
	if (nborCoord[this->getSafestLaneIndex()].size() > 0)
		return nborCoord[this->getSafestLaneIndex()].at(0).getId();
	else
		return -1;
}

std::vector<int> BaseSelfOrganizationApp::getMapFormation(int laneIndex)
{
    std::vector<int> formation;

    this->sortTopology();

    for(int i = 0; i < nborCoord[laneIndex].size(); i++)
    {
        formation.push_back(nborCoord[laneIndex].at(i).getId());
    }

    return formation;
}

std::vector<int> BaseSelfOrganizationApp::getLaneLeaders()
{
    std::vector<int> leader_list;

    for (auto &i : laneLeaders) leader_list.push_back(i.second);

    return leader_list;
}

std::vector<int> BaseSelfOrganizationApp::getBlockedLanes()
{
    return blockedLanes;
}
