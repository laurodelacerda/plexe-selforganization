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

#include "SelfOrganizationApp.h"

using namespace Veins;

Define_Module(SelfOrganizationApp);

void SelfOrganizationApp::initialize(int stage)
{
    BaseSelfOrganizationApp::initialize(stage);

    if (stage == 1)
    {
//        std::string joinManeuverName = par("joinManeuver").stdstringValue();
//        if (joinManeuverName == "JoinAtBack")
//            joinManeuver = new JoinAtBack(this);
//        else if (joinManeuverName == "JoinAtBackSign")
//            joinManeuver = new JoinAtBackSign(this);
//        else
//            throw new cRuntimeError("Invalid join maneuver implementation chosen");
        maneuverControl = new DynamicJoin(this);

        join_adjust = new cMessage("join_adjust");
    }
}

void SelfOrganizationApp::onPlatoonBeacon(const PlatooningBeacon* pb)
{
//    if ((ongoing_maneuver == PlatoonManeuver::JOIN_AT_BACK) or
//        (ongoing_maneuver == PlatoonManeuver::JOIN_AT_FRONT) or
//        (ongoing_maneuver == PlatoonManeuver::JOIN_IN_THE_MIDDLE))
//    {
//    if (inManeuver)
//    if ((role == PlatoonRole::JOINER) or (role == PlatoonRole::LEADER) or (role == PlatoonRole::FOLLOWER))
    maneuverControl->onPlatoonBeacon(pb);
//    }
//    else if (ongoing_maneuver == PlatoonManeuver::SPLIT)
//    {
//        splitManeuver->onPlatoonBeacon(pb);
//    }
    BaseSelfOrganizationApp::onPlatoonBeacon(pb);
}

bool SelfOrganizationApp::isJoinAllowed(int position)
{
    if ((role == PlatoonRole::LEADER or role == PlatoonRole::NONE)) // and !inManeuver)
    {
        int lane_index   = traciVehicle->getLaneIndex();
        int platoon_size = positionHelper->getPlatoonSize();
        int veh_id       = nborCoord[lane_index][position].getId();

        if (position >= platoon_size) // Permission to Join at Back
            return true;

        if (maneuver_status[veh_id] == PlatoonManeuver::IDLE)
            return true;
        else
            return false;
    }
    else
        return false;
}

void SelfOrganizationApp::onManeuverMessage(ManeuverMessage* mm)
{
//    if (UpdatePlatoonFormation* msg = dynamic_cast<UpdatePlatoonFormation*>(mm))
//    {
//        handleUpdatePlatoonFormation(msg);
//        delete msg;
//    }
//    else
//    {
//        if ((ongoing_maneuver == PlatoonManeuver::JOIN_AT_BACK) or
//            (ongoing_maneuver == PlatoonManeuver::JOIN_AT_FRONT) or
//            (ongoing_maneuver == PlatoonManeuver::JOIN_IN_THE_MIDDLE))
//        {
//        if (joinManeuver != nullptr)
    maneuverControl->onManeuverMessage(mm);
//        }

//    }

    GeneralPlatooningApp::onManeuverMessage(mm);
}

void SelfOrganizationApp::startJoinManeuver(int platoonId, int leaderId, int position)
{
//    ASSERT((getPlatoonRole() == PlatoonRole::NONE) or
//           (getPlatoonRole() == PlatoonRole::UNSAFE_FOLLOWER) or
//           (getPlatoonRole() == PlatoonRole::UNSAFE_LEADER));

    ASSERT(!isInManeuver());

    JoinManeuverParameters params;
    params.platoonId = platoonId;
    params.leaderId = leaderId;
    params.position = position;

//    if (position == -1)
//        ongoing_maneuver = PlatoonManeuver::JOIN_AT_BACK;
//    else if (position == 0)
//        ongoing_maneuver = PlatoonManeuver::JOIN_AT_FRONT;
//    else if (position > 0)
//        ongoing_maneuver = PlatoonManeuver::JOIN_IN_THE_MIDDLE;
//    else
//        abortJoinManeuver();
//
//    if (position >= -1)
    maneuverControl->startManeuver(&params);

}

void SelfOrganizationApp::setupFormation()
{
    std::vector<int> formation = getFormation(traciVehicle->getLaneIndex());

    positionHelper->setPlatoonFormation(formation);
}

void SelfOrganizationApp::startManeuverFormation()
{
    Plexe::VEHICLE_DATA data;
    traciVehicle->getVehicleData(&data);

    std::vector<int> formation = getFormation(traciVehicle->getLaneIndex());
    positionHelper->setPlatoonFormation(formation);

//    if (role == PlatoonRole::LEADER)
//    {
//        positionHelper->setPlatoonId(positionHelper->getId());
//        positionHelper->setIsLeader(true);
//        positionHelper->setPlatoonLane(traciVehicle->getLaneIndex());
//        positionHelper->setPlatoonSpeed(data.speed);
//    }
//    if (role == PlatoonRole::FOLLOWER)
//    {
//        positionHelper->setPlatoonId(formation.at(0));
//        positionHelper->setIsLeader(false);
//        positionHelper->setPlatoonLane(traciVehicle->getLaneIndex());
//
//        traciVehicle->setActiveController(Plexe::CACC);
//        traciVehicle->setFixedLane(traciVehicle->getLaneIndex());
//        traciVehicle->setCACCConstantSpacing(20);
//    }
//    if (role == PlatoonRole::UNSAFE_LEADER)
//    {
//        positionHelper->setPlatoonId(positionHelper->getId());
//
//        //    int safest_lane   = getSafestLaneIndex();
//        int safest_leader = getSafestLaneLeader();
//
//        if (ongoing_maneuver == PlatoonManeuver::IDLE)
//            startJoinManeuver(safest_leader, safest_leader, -1);
//    }
//    if (role == PlatoonRole::UNSAFE_FOLLOWER)
//    {
//        positionHelper->setPlatoonId(formation.at(0));
//        positionHelper->setIsLeader(false);
//        positionHelper->setPlatoonLane(traciVehicle->getLaneIndex());
//
//        traciVehicle->setFixedLane(traciVehicle->getLaneIndex());
//
//        if (inDanger && !safeJoinCheck->isScheduled())
//            scheduleAt(simTime() + SimTime(100, SIMTIME_MS), safeJoinCheck);
//    }

    if ((inDanger) and (role != PlatoonRole::LEADER) and (role != PlatoonRole::FOLLOWER))
    {
    	int safest_leader = getSafestLaneLeader();

    	if (ongoing_maneuver == PlatoonManeuver::IDLE)
			startJoinManeuver(safest_leader, safest_leader, -1);
    }
}

SelfOrganizationApp::~SelfOrganizationApp()
{
    delete maneuverControl;
    cancelAndDelete(join_adjust);
    join_adjust = nullptr;
}

int SelfOrganizationApp::getBestAvailableEntryPosition(int joiner_id)
{
//     TODO Algoritmo de tomada de decisão para alocação de veículos em comboios

//    Algoritmo Simples
//      Distância euclidiana para à frente/trás do comboio
//
//
//     O algoritmo tomador de decisão observa critérios como:
//     a) lacunas disponíveis
//     b) velocidade dos veículos envolvidos,
//     c) distância euclidiana de aproximação à lacuna
//     d) distância em relação ao bloqueio

//     Para sugerir o seguinte pseudocódigo:
//    (1)   Ordenar as lacunas disponíveis por critério de distância euclidiana em relação
//          à posição atual do veículo solicitante
//    (2)   Iterar sobre as lacunas, verificando:
//    (2.1) Se lacuna mais próxima não estiver "ocupada" e couber veículo solicitante e a
//          velocidade do veículo for suficiente para alcançar a lacuna, então sugerir a
//          posição de entrada ao veículo solicitante.
//          Caso contrário, iterar sobre outra lacuna.
//    (3.1) Se não for possível entrada com lacunas disponíveis, sugerir entrada pela cauda

//    Ps.: Há outros critérios que podem ser adotados para ordenar as lacunas, como
//    I) maior lacuna
//    II) espaçamento ótimo

//    Contudo, não será possível discutí-los para este trabalho

    this->sortTopology();

    VehicleCoord joiner_info = nbor_info[joiner_id];

    int lane_index   = traciVehicle->getLaneIndex();
    int platoon_size = nborCoord[lane_index].size();

    Coord frontPos = nborCoord[lane_index].at(0).getCoord();
    Coord backPos  = nborCoord[lane_index].at(platoon_size - 1).getCoord();

    Orientation orientation = calculateDirection(mobility->getAngleRad());

    double dist_front = 0;
	double dist_back  = 0;

	if (orientation == Orientation::EAST)
	{
		frontPos.x = frontPos.x + traciVehicle->getLength()/2 ;
		backPos.x  = backPos.x - traciVehicle->getLength()/2 ;

        dist_front = joiner_info.getCoord().distance(frontPos);
        dist_back  = joiner_info.getCoord().distance(backPos);
	}

    if (dist_front <= dist_back)  // Join at Front
        return 0;
    else                        // Join at Back
        return platoon_size;

//    return 1;

}

bool SelfOrganizationApp::isSafeToManeuver(int lane_index, int position)
{

	if (lane_index == traciVehicle->getLaneIndex()) return false;

	// Current position
	Plexe::VEHICLE_DATA data;
	traciVehicle->getVehicleData(&data);
	Coord myPos = Coord(data.positionX, data.positionY);

	// Platoon info
	int platoon_size = nborCoord[lane_index].size();
	double distance  = 0;

	if (ongoing_maneuver == PlatoonManeuver::JOIN_AT_BACK)
	{
        Coord tail = nborCoord[lane_index][platoon_size - 1].getCoord();

//        distance = traci->getDistance(myPos, tail, true);
        distance = myPos.distance(tail);

        if (distance == DBL_MAX) // In front of the tail
            return false;
        else if (distance > SAFETY_GAP) // Lacuna de segurança do comboio
            return true;
	}
	else if (ongoing_maneuver == PlatoonManeuver::JOIN_AT_FRONT)
	{
		Coord head = nborCoord[lane_index][0].getCoord();

//        distance = traci->getDistance(myPos, head_pos, true);
        distance = myPos.distance(head); // - traciVehicle->getLength();

        if (distance == DBL_MAX) // Behind the head
            return false;
        else if (std::fabs(distance) > SAFETY_GAP) // Lacuna de segurança do comboio
            return true;
        else
        	return false;
	}
	else if (ongoing_maneuver == PlatoonManeuver::JOIN_AT_MIDDLE)
	{

	}

//	//    Verificar minha posição em relação à posição que se deseja ocupar no pelotão, sendo -1 a entrada por trás.
//
//	//    Caso minha coordenada esteja suficientemente segura, retornar true
//
//	    Plexe::VEHICLE_DATA data;
//	    traciVehicle->getVehicleData(&data);
//
//	    Coord myPos = Coord(data.positionX, data.positionY);
//	//    Veins::TraCICoord traciPosition = mobility->getManager()->omnet2traci(mobility->getCurrentPosition());
//	//    double distance = position.distance(frontPosition) - pb->getLength();
//
//	    // Joiner is not moving at all
//	    if (lane_index == traciVehicle->getLaneIndex()) return false;
//
//
//	    this->sortTopology();
//	    int platoon_size = nborCoord[lane_index].size();
//
//	    double distance = 0;
//
//
//	    if (position == 0) // Join at Front
//	    {
//	        VehicleCoord head = nborCoord[lane_index][0];
//	        Coord    head_pos = head.getCoord();
//	        double  head_size = head.getLength();
//	//        Coord myPos     = mobility->getCurrentPosition();
//
//	        distance = traci->getDistance(head_pos, myPos, true);
//	        double dist = myPos.distance(head_pos) - head_size;
//
//	        if (distance == DBL_MAX) // Veículo solicitante está na frente da cabeça do comboio
//	        {
//	            distance = traci->getDistance(myPos, head_pos, true);
//	//            return false;
//	        }
//	        else if ((distance != DBL_MAX) and (std::fabs(dist) > SAFETY_GAP)) // Lacuna de segurança do comboio
//	            return true;
//	    }
//	//    else if ((position > 0) and (position < platoon_size)) // Join at Middle
//	//    {
//	//        VehicleCoord front = nborCoord[lane_index][position - 1];
//	//        VehicleCoord back  = nborCoord[lane_index][position];
//	//
//	//        Coord front_pos = front.getCoord();
//	//        Coord back_pos  = back.getCoord();
//	//
//	//        double front_size = front.getLength();
//	//
//	//        double dist_back  = traci->getDistance(myPos, back_pos, false);
//	//        double dist_front = traci->getDistance(front_pos, myPos, false) - front_size;
//	//
//	//        if ((dist_back >= SAFETY_GAP) and (dist_front >= SAFETY_GAP))
//	//            return true;
//	//        else
//	//            return false;
//	//    }
//	    else if (position == platoon_size) // Join at Back
//	    {
//	        Coord last_pos = nborCoord[lane_index][platoon_size - 1].getCoord();
//	//        Coord my_pos   = mobility->getCurrentPosition();
//
//	        distance = traci->getDistance(myPos, last_pos, true);
//
//	        if (distance == DBL_MAX) // Veículo solicitante está na frente da cauda do comboio
//	//            distance = traci->getDistance(last_pos, my_pos, true);
//	            return false;
//	        else if (distance > SAFETY_GAP) // Lacuna de segurança do comboio
//	            return true;
//
//	    }

}

void SelfOrganizationApp::adjustToManeuver()
{
    if (ongoing_maneuver == PlatoonManeuver::JOIN_AT_BACK)
        scheduleAt(simTime() + SimTime(TIMER_JOIN_AT_BACK, SIMTIME_MS), join_adjust);
    else if (ongoing_maneuver == PlatoonManeuver::JOIN_AT_FRONT)
        scheduleAt(simTime() + SimTime(TIMER_JOIN_AT_FRONT, SIMTIME_MS), join_adjust);
    else if (ongoing_maneuver == PlatoonManeuver::JOIN_AT_MIDDLE)
        scheduleAt(simTime() + SimTime(TIMER_JOIN_AT_FRONT, SIMTIME_MS), join_adjust);
}

void SelfOrganizationApp::sendUnicast(cPacket* msg, int destination)
{
    Enter_Method_Silent();
    take(msg);
    UnicastMessage* unicast = new UnicastMessage("UnicastMessage", msg->getKind());
    unicast->setDestination(destination);
    unicast->setChannel(Channels::CCH);
    unicast->encapsulate(msg);
    sendDown(unicast);
}

void SelfOrganizationApp::setManeuverStatus(int veh_id, PlatoonManeuver maneuver)
{
    maneuver_status[veh_id] = maneuver ;
}

void SelfOrganizationApp::handleSelfMsg(cMessage* msg)
{
    if (msg == join_adjust)
        maneuverControl->moveToPosition();

    BaseSelfOrganizationApp::handleSelfMsg(msg);
}

void SelfOrganizationApp::printInfo()
{
    std::string role_str ;
    std::string control_str ;
    std::string join_str;

    if (getPlatoonRole() == PlatoonRole::LEADER)
        role_str = "LEADER";
    else if (getPlatoonRole() == PlatoonRole::FOLLOWER)
        role_str = "FOLLOWER";
    else if (getPlatoonRole() == PlatoonRole::JOINER)
        role_str = "JOINER";
    else if (getPlatoonRole() == PlatoonRole::UNSAFE_LEADER)
        role_str = "UNSAFE_LEADER";
    else if (getPlatoonRole() == PlatoonRole::UNSAFE_FOLLOWER)
        role_str = "UNSAFE_FOLLOWER";
    else
        role_str = "NONE";


    if (traciVehicle->getActiveController() == Plexe::ACC)
    	control_str = "Plexe::ACC";
    else if (traciVehicle->getActiveController() == Plexe::DRIVER)
    	control_str = "Plexe::DRIVER";
    else if (traciVehicle->getActiveController() == Plexe::CACC)
    	control_str = "Plexe::CACC";
	else if (traciVehicle->getActiveController() == Plexe::FAKED_CACC)
		control_str = "Plexe::FAKED_ACC";
	else
		control_str = "Other";


//    if (maneuverControl->getJoinManeuverState() == JoinManeuverState::IDLE)
//    	join_str = "IDLE";
//    else if (maneuverControl->getJoinManeuverState() == JoinManeuverState::J_WAIT_REPLY)
//    	join_str = "J_WAIT_REPLY";
//	else if (maneuverControl->getJoinManeuverState() == JoinManeuverState::J_WAIT_INFORMATION)
//		join_str = "J_WAIT_INFORMATION";
//	else if (maneuverControl->getJoinManeuverState() == JoinManeuverState::J_MOVE_IN_POSITION)
//		join_str = "J_MOVE_IN_POSITION";
//	else if (maneuverControl->getJoinManeuverState() == JoinManeuverState::J_ADJUST)
//		join_str = "J_ADJUST";
//	else if (maneuverControl->getJoinManeuverState() == JoinManeuverState::J_WAIT_JOIN)
//		join_str = "J_WAIT_JOIN";
//	else if (maneuverControl->getJoinManeuverState() == JoinManeuverState::L_WAIT_J_HANDOFF)
//		join_str = "L_WAIT_J_HANDOFF";
//	else if (maneuverControl->getJoinManeuverState() == JoinManeuverState::L_WAIT_JOINER_IN_POSITION)
//		join_str = "L_WAIT_JOINER_IN_POSITION";
//	else if (maneuverControl->getJoinManeuverState() == JoinManeuverState::L_WAIT_JOINER_TO_JOIN)
//		join_str = "L_WAIT_JOINER_TO_JOIN";




    // Info about vehicle/platoon
    std::cout << "\n\n@@@Vehicle " << myId << " at " << simTime() << " @@@"
			 	 << "\nSpeed: " 	   << mobility->getSpeed()
				 << "\nIn Danger: "    << inDanger
				 << "\nIn Maneuver: "  << inManeuver
    			 << "\n-------------"
                 << "\nPlatoon Role: " << role_str
				 << "\nPlatoon Id: "   << positionHelper->getPlatoonId()
                 << "\nPlatoon Pos: "  << positionHelper->getPosition()
                 << "\nLeader Id: "    << positionHelper->getLeaderId()
                 << "\nPlatoon Size: " << positionHelper->getPlatoonSize()
                 << "\nPlat Length: "  << getPlatoonLength(traciVehicle->getLaneIndex())
				 << "\nCruise Model: " << traciVehicle->getActiveController()
				 << "\nC Control: "    << control_str
   				 << "\nHeadway (s): "  << traciVehicle->getACCHeadwayTime()
				 << "\nSpacing (m): "  << traciVehicle->getCACCConstantSpacing()
	 	 	 	 << "\n-------------"
	 	 	 	 <<	"\nJoin status: "	   << join_str;
//                 << "\n\n";


    // Info about formation by lane
    std::cout << "\n-------------" ;
    for (int i = 0; i < 4; i++)
    {
        std::cout << "\nLane" << "[" << i << "]: " ;

        std::vector<int> formation = getFormation(i);
        for (auto &j : formation)
            std::cout << j << " ";
    }

    // Info about blocked lanes
    std::vector<int> blocked_lanes = getBlockedLanes();

    std::cout << "\n-------------\nBlocked Lanes: "  ;
    for (int j = 0; j < blocked_lanes.size(); j++)
    {
        std::cout << blocked_lanes.at(j) << " ";
    }


    // Info about inter-vehicular Gaps
    std::array<std::vector<double>, 5> gaps;

    for(int i = 0; i < 5; i++)
        calculatePlatoonGaps(gaps[i], i, traciVehicle->getLength());

    std::cout << "\n-------------\n" ;
    for(int k = 0; k < 5; k++)
    {
        std::cout << "Gaps Lane " << k << ": ";
        for(double d : gaps[k])
        {
            std::cout << d << " ";
        }
        std::cout << "\n";
    }

}
