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
    if ((role == PlatoonRole::LEADER or role == PlatoonRole::NONE) and !inManeuver)
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

    if (role == PlatoonRole::LEADER)
    {
        positionHelper->setPlatoonId(positionHelper->getId());
        positionHelper->setIsLeader(true);
        positionHelper->setPlatoonLane(traciVehicle->getLaneIndex());
        positionHelper->setPlatoonSpeed(data.speed);
    }
    if (role == PlatoonRole::FOLLOWER)
    {
        positionHelper->setPlatoonId(formation.at(0));
        positionHelper->setIsLeader(false);
        positionHelper->setPlatoonLane(traciVehicle->getLaneIndex());

        traciVehicle->setActiveController(Plexe::CACC);
        traciVehicle->setFixedLane(traciVehicle->getLaneIndex());
        traciVehicle->setCACCConstantSpacing(20);
    }
    if (role == PlatoonRole::UNSAFE_LEADER)
    {
        positionHelper->setPlatoonId(formation.at(0));

        //    int safest_lane   = getSafestLaneIndex();
        int safest_leader = getSafestLaneLeader();

        if (ongoing_maneuver == PlatoonManeuver::IDLE)
            startJoinManeuver(safest_leader, safest_leader, -1);
    }
    if (role == PlatoonRole::UNSAFE_FOLLOWER)
    {
        positionHelper->setPlatoonId(formation.at(0));
        positionHelper->setIsLeader(false);
        positionHelper->setPlatoonLane(traciVehicle->getLaneIndex());

        traciVehicle->setFixedLane(traciVehicle->getLaneIndex());

        if (inDanger && !safeJoinCheck->isScheduled())
            scheduleAt(simTime() + SimTime(100, SIMTIME_MS), safeJoinCheck);
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


    VehicleCoord joiner_info = nbor_info[joiner_id];

    int lane_index   = traciVehicle->getLaneIndex();
    int platoon_size = nborCoord[lane_index].size();

    Coord frontPos = nborCoord[lane_index].at(0).getCoord();
    Coord backPos  = nborCoord[lane_index].at(platoon_size - 1).getCoord();

    double dist_front = joiner_info.getCoord().distance(frontPos);
    double dist_back  = joiner_info.getCoord().distance(backPos);


    if (dist_front < dist_back) // join-at-front
        return 0;
    else                        // join-at-back
        return platoon_size;

//    return 1;

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

void SelfOrganizationApp::adjustToManeuver()
{
    // TODO Realizar alterações de aceleração e desaceleração para entrada no comboio
    if (ongoing_maneuver == PlatoonManeuver::JOIN_AT_BACK)
    {
//        traciVehicle->slowDown(targetPlatoonData->platoonSpeed - 1, 0);
        scheduleAt(simTime() + SimTime(TIMER_JOIN_AT_BACK, SIMTIME_MS), join_adjust);
    }
    else if (ongoing_maneuver == PlatoonManeuver::JOIN_AT_FRONT)
    {
        scheduleAt(simTime() + SimTime(TIMER_JOIN_AT_FRONT, SIMTIME_MS), join_adjust);
    }
    else if (ongoing_maneuver == PlatoonManeuver::JOIN_AT_MIDDLE)
    {
        scheduleAt(simTime() + SimTime(TIMER_JOIN_AT_FRONT, SIMTIME_MS), join_adjust);
    }
}

void SelfOrganizationApp::setManeuverStatus(int veh_id, PlatoonManeuver maneuver)
{
    maneuver_status[veh_id] = maneuver ;
}

void SelfOrganizationApp::handleSelfMsg(cMessage* msg)
{
    if (msg == join_adjust)
    {
        maneuverControl->moveToPosition();
    }

    BaseSelfOrganizationApp::handleSelfMsg(msg);
}
