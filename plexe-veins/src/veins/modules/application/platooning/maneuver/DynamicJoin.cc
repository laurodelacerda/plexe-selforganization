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

#include "DynamicJoin.h"

#include "veins/modules/application/platooning/apps/GeneralPlatooningApp.h"

using namespace Veins;

DynamicJoin::DynamicJoin(GeneralPlatooningApp* app)
    : JoinManeuver(app),
      joinManeuverState(JoinManeuverState::IDLE)
{
    // TODO Auto-generated constructor stub
}

DynamicJoin::~DynamicJoin()
{

}

void DynamicJoin::abortManeuver()
{
    joinManeuverState = JoinManeuverState::IDLE;
    app->setPlatoonRole(PlatoonRole::NONE);
    app->setInManeuver(false);
}

void DynamicJoin::startManeuver(const void* parameters)
{

    std::cout << "(" << simTime() << ")" <<  " > Vehicle: " << positionHelper->getId() << " - startManeuver" << std::endl;
    JoinManeuverParameters* pars = (JoinManeuverParameters*) parameters;

    if (joinManeuverState == JoinManeuverState::IDLE)
    {
    	// Assertions
        ASSERT((app->getPlatoonRole() == PlatoonRole::NONE) or
        	   (app->getPlatoonRole() == PlatoonRole::UNSAFE_LEADER) or
			   (app->getPlatoonRole() == PlatoonRole::UNSAFE_FOLLOWER));

        ASSERT(!app->isInManeuver());

        // collect information about target Platoon
        targetPlatoonData.reset(new TargetPlatoonData());
        targetPlatoonData->platoonId     = pars->platoonId;
        targetPlatoonData->platoonLeader = pars->leaderId;
        targetPlatoonData->joinIndex     = pars->position;

        // send join request to leader
        JoinPlatoonRequest* req = createJoinPlatoonRequest(positionHelper->getId(),
                                                           positionHelper->getExternalId(),
                                                           targetPlatoonData->platoonId,
                                                           targetPlatoonData->platoonLeader,
                                                           traciVehicle->getLaneIndex(),
                                                           mobility->getCurrentPosition().x,
                                                           mobility->getCurrentPosition().y);

        app->sendUnicast(req, targetPlatoonData->platoonLeader);
        joinManeuverState = JoinManeuverState::J_WAIT_REPLY;
    }

}

void DynamicJoin::onPlatoonBeacon(const PlatooningBeacon* pb)
{
//    std::cout << " > Vehicle: " << positionHelper->getId() << " - onPlatoonBeacon from "<< pb->getVehicleId() << std::endl;

    if ((joinManeuverState == JoinManeuverState::J_MOVE_IN_POSITION) or (joinManeuverState == JoinManeuverState::J_ADJUST))
    {
            // check correct role
            if ((app->getPlatoonRole() != PlatoonRole::JOINER)        and
                (app->getPlatoonRole() != PlatoonRole::UNSAFE_LEADER) and
                (app->getPlatoonRole() != PlatoonRole::FOLLOWER)) return;

            // if the message comes from the leader
            if (pb->getVehicleId() == targetPlatoonData->newFormation.at(0))
            {
                traciVehicle->setLeaderVehicleFakeData(pb->getControllerAcceleration(), pb->getAcceleration(), pb->getSpeed());
            }

            // if the message comes from the front vehicle
            int frontPosition = targetPlatoonData->joinIndex - 1;
            int frontId = targetPlatoonData->newFormation.at(frontPosition);

            if (pb->getVehicleId() == frontId)
            {
                // get front vehicle position
                Coord frontPosition(pb->getPositionX(), pb->getPositionY(), 0);

                // get my position
                Veins::TraCICoord traciPosition = mobility->getManager()->omnet2traci(mobility->getCurrentPosition());
                Coord position(traciPosition.x, traciPosition.y);

                // compute my distance to front vehicle
                double distance = position.distance(frontPosition) - pb->getLength();

                traciVehicle->setFrontVehicleFakeData(pb->getControllerAcceleration(), pb->getAcceleration(), pb->getSpeed(), distance);

                // if we are in position, tell the leader about that
                if (distance < PLATOON_ZONE) // TODO fixed value? make dependent on controller and headway time
                {
                    if (joinManeuverState == JoinManeuverState::J_ADJUST)
                    {
                        traciVehicle->setActiveController(Plexe::CACC);
                        return;
                    }

                    // send move to position response to confirm the parameters
                    MoveToPositionAck* ack = createMoveToPositionAck(positionHelper->getId(), positionHelper->getExternalId(), targetPlatoonData->platoonId, targetPlatoonData->platoonLeader, targetPlatoonData->platoonSpeed, targetPlatoonData->platoonLane, targetPlatoonData->newFormation);
                    app->sendUnicast(ack, targetPlatoonData->newFormation.at(0));
                    joinManeuverState = JoinManeuverState::J_WAIT_JOIN;
                }
            }
    }
}

void DynamicJoin::handleJoinPlatoonRequest(const JoinPlatoonRequest* msg)
{
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - handleJoinPlatoonRequest" << std::endl;

	// Assertions
    if (app->getPlatoonRole() != PlatoonRole::LEADER && app->getPlatoonRole() != PlatoonRole::NONE) return;

    // Get position and evaluate permission
    int  position   = app->getBestAvailableEntryPosition(msg->getVehicleId());
    bool permission = app->isJoinAllowed(position);


    std::cout << " > Vehicle: " << positionHelper->getId() << " - permission : " << permission << " - position: " << position << std::endl;

    // Send answer to Joiner
    JoinPlatoonResponse* response = createJoinPlatoonResponse(positionHelper->getId(),
                                                              positionHelper->getExternalId(),
                                                              msg->getPlatoonId(),
                                                              msg->getVehicleId(),
                                                              permission);
    app->sendUnicast(response, msg->getVehicleId());

    // If permission is granted, set platoon parameters
    if (!permission) return;

    app->setPlatoonRole(PlatoonRole::LEADER);
    app->setInManeuver(false); // NOTE Leader should not be in maneuver
    traciVehicle->setFixedLane(traciVehicle->getLaneIndex());   // Disable lane changing

    positionHelper->setPlatoonId(positionHelper->getId());
    positionHelper->setLeaderId(positionHelper->getId());
    positionHelper->setPlatoonSpeed(mobility->getSpeed());
    positionHelper->setPlatoonLane(traciVehicle->getLaneIndex());

    // Save joiner data
    joinerData.reset(new JoinerData());
    joinerData->from(msg);

    // Add leader/joiner to the the platoon formation
    app->setupFormation();
    joinerData->newFormation = positionHelper->getPlatoonFormation();

    // Remove joiner id if it is already at formation
    joinerData->newFormation.erase(std::remove(joinerData->newFormation.begin(),
    										   joinerData->newFormation.end(),
											   joinerData->joinerId),
    										   joinerData->newFormation.end());


    // Add leader
    if (positionHelper->getPlatoonSize() > 0)
    {
    	// If there is another leader Id, remove it
    	if (joinerData->newFormation.at(0) != positionHelper->getId())
		    joinerData->newFormation.erase(std::remove(joinerData->newFormation.begin(),
		    										   joinerData->newFormation.end(),
													   positionHelper->getId()), joinerData->newFormation.end());

    }
    else
		joinerData->newFormation.push_back(positionHelper->getId());

    // Add joiner
    if (position == 0) joinerData->newFormation.insert(joinerData->newFormation.begin(), joinerData->joinerId);
    else if (position == positionHelper->getPlatoonSize()) joinerData->newFormation.push_back(joinerData->joinerId);
    else // TODO Join at the middle
    {
//        int front_id = joinerData->newFormation.at(position);
//        double gap = 5 * traciVehicle->getCACCConstantSpacing();
//
//        auto it = joinerData->newFormation.begin() + position;
//        joinerData->newFormation.insert(it, joinerData->joinerId);
//
//        SplitFormation* spl = createSplitFormation(positionHelper->getId(),
//                                                  positionHelper->getExternalId(),
//                                                  positionHelper->getPlatoonId(),
//                                                  front_id,
//                                                  positionHelper->getPlatoonSpeed(),
//                                                  positionHelper->getPlatoonLane(),
//                                                  gap,
//                                                  joinerData->newFormation);
//
//        app->sendUnicast(spl, front_id);
    }

    // Update platoon formation
    positionHelper->setPlatoonFormation(joinerData->newFormation);

    // Send the platoon data to the joiner
    MoveToPosition* mtp = createMoveToPosition(positionHelper->getId(),
                                               positionHelper->getExternalId(),
                                               positionHelper->getPlatoonId(),
                                               joinerData->joinerId,
                                               positionHelper->getPlatoonSpeed(),
                                               positionHelper->getPlatoonLane(),
                                               joinerData->newFormation);
    app->sendUnicast(mtp, joinerData->joinerId);

    joinManeuverState = JoinManeuverState::L_WAIT_JOINER_IN_POSITION;
}

void DynamicJoin::handleJoinPlatoonResponse(const JoinPlatoonResponse* msg)
{
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - handleJoinPlatoonResponse" << std::endl;

    if (joinManeuverState != JoinManeuverState::J_WAIT_REPLY) return;
    if (msg->getPlatoonId() != targetPlatoonData->platoonId) return;
    if (msg->getVehicleId() != targetPlatoonData->platoonLeader) return;

    // evaluate permission
    if (msg->getPermitted())
    {
        // wait for information about the join maneuver
        joinManeuverState = JoinManeuverState::J_WAIT_INFORMATION;
        // disable lane changing during maneuver
        traciVehicle->setFixedLane(traciVehicle->getLaneIndex());
    }
    else
        abortManeuver();

}

void DynamicJoin::handleMoveToPosition(const MoveToPosition* msg)
{
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - handleMoveToPosition" << std::endl;

	// Assertions
    if (joinManeuverState != JoinManeuverState::J_WAIT_INFORMATION) return;
    if (msg->getPlatoonId() != targetPlatoonData->platoonId) return;
    if (msg->getVehicleId() != targetPlatoonData->platoonLeader) return;

    // Save platoon data
    targetPlatoonData->from(msg);

    int platoon_size  = targetPlatoonData->newFormation.size();

    // No negative position
    ASSERT(targetPlatoonData->joinIndex >= 0);

    // Set maneuver info
    if (targetPlatoonData->joinIndex == 0)
    	app->setOngoingManeuver(PlatoonManeuver::JOIN_AT_FRONT);
    else if (targetPlatoonData->joinIndex == platoon_size - 1)
    	app->setOngoingManeuver(PlatoonManeuver::JOIN_AT_BACK);
    else if (targetPlatoonData->joinIndex > 0 and targetPlatoonData->joinIndex - 1 < platoon_size)
        app->setOngoingManeuver(PlatoonManeuver::JOIN_AT_MIDDLE);
    else
    {
        app->abortJoinManeuver();
        return;
    }

    app->setInManeuver(true);
    app->setPlatoonRole(PlatoonRole::JOINER);
    moveToPosition();
}

void DynamicJoin::moveToPosition()
{
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - moveToPosition" << std::endl;

    Plexe::VEHICLE_DATA data;
    traciVehicle->getVehicleData(&data);

    bool   safe          = app->isSafeToManeuver(targetPlatoonData->platoonLane, targetPlatoonData->joinIndex);
    double platoon_speed = targetPlatoonData->platoonSpeed;
    double my_speed      = data.speed;
    double opt_speed;

    if (safe)
    {
        if (app->getOngoingManeuver() == PlatoonManeuver::JOIN_AT_BACK)
        {
            if (traciVehicle->getLaneIndex() != targetPlatoonData->platoonLane)
                traciVehicle->setFixedLane(targetPlatoonData->platoonLane, false);

            traciVehicle->setCACCConstantSpacing(15);
            traciVehicle->setLeaderVehicleFakeData(0, 0, targetPlatoonData->platoonSpeed);
            traciVehicle->setFrontVehicleFakeData(0, 0, targetPlatoonData->platoonSpeed, 15);
            traciVehicle->setCruiseControlDesiredSpeed(targetPlatoonData->platoonSpeed + (30 / 3.6));
            traciVehicle->setActiveController(Plexe::FAKED_CACC);

//            app->setPlatoonRole(PlatoonRole::JOINER);
            joinManeuverState = JoinManeuverState::J_MOVE_IN_POSITION;
        }
        else if (app->getOngoingManeuver() == PlatoonManeuver::JOIN_AT_MIDDLE)
        {

        }
        else if (app->getOngoingManeuver() == PlatoonManeuver::JOIN_AT_FRONT)
        {
            if (traciVehicle->getLaneIndex() != targetPlatoonData->platoonLane)
                traciVehicle->setFixedLane(targetPlatoonData->platoonLane, false);

            traciVehicle->setActiveController(Plexe::ACC);
            traciVehicle->setCruiseControlDesiredSpeed(targetPlatoonData->platoonSpeed);
            traciVehicle->setSpeed(targetPlatoonData->platoonSpeed);

            HandoffLeadershipRequest* req = createHandoffLeadershipRequest(positionHelper->getId(), positionHelper->getExternalId(),
                                            targetPlatoonData->platoonId, targetPlatoonData->platoonLeader, traciVehicle->getLaneIndex(),
                                            mobility->getCurrentPosition().x, mobility->getCurrentPosition().y);

            app->sendUnicast(req, targetPlatoonData->platoonLeader);

//            app->setPlatoonRole(PlatoonRole::JOINER);

            joinManeuverState = JoinManeuverState::J_WAIT_L_HANDOFF;
        }
    }
    else
    {
        if (app->getOngoingManeuver() == PlatoonManeuver::JOIN_AT_BACK)
        {
            opt_speed = (platoon_speed * 2) / 3 ;
            // TODO Vehicular Dynamics Verificar qual desaceleração é a mais adequada
            if (my_speed > opt_speed)
                traciVehicle->slowDown(data.speed - 1, 0);
//                traciVehicle->setFixedAcceleration(1, FIXED_DECELERATION);

        }
        else if (app->getOngoingManeuver() == PlatoonManeuver::JOIN_AT_FRONT)
        {
            opt_speed = platoon_speed + (platoon_speed/3) ;

            if (my_speed < opt_speed)
                traciVehicle->setSpeed(data.speed + 1);
//                traciVehicle->setFixedAcceleration(1, FIXED_ACCELERATION);
        }

        app->adjustToManeuver();

//        abortManeuver();
        // request position
//        JoinManeuverParameters params;
//        params.platoonId = targetPlatoonData->platoonId;
//        params.leaderId  = targetPlatoonData->platoonLeader;
//        params.position  = -1;
//        startManeuver(&params);

    }
}

void DynamicJoin::handleHandoffLeadershipRequest(const HandoffLeadershipRequest* msg)
{
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - handleHandoffLeadershipRequest" << std::endl;

    if (app->getPlatoonRole() != PlatoonRole::LEADER) return;
//    if (joinManeuverState != JoinManeuverState::L_WAIT_JOINER_IN_POSITION) return;
    if (msg->getPlatoonId() != positionHelper->getPlatoonId()) return;
//    if (msg->getVehicleId() != joinerData->joinerId) return;

    // TODO Analisar permissão com maior criterio, verificando se veículo está mesmo posicionado à frente
    bool permission = true;

    // Handoff leadership

    if (permission)
    {
    	// Formation with a new leader
    	std::vector<int> formation = positionHelper->getPlatoonFormation();
//    	formation.insert(formation.begin(), joinerData->joinerId);
    	formation.insert(formation.begin(), msg->getVehicleId());
    	positionHelper->setPlatoonFormation(formation);

    	HandoffLeadershipResponse* res = createHandoffLeadershipResponse(
    			positionHelper->getId(), positionHelper->getExternalId(), positionHelper->getPlatoonId(), msg->getVehicleId(),
				permission, positionHelper->getPlatoonLane(), positionHelper->getPlatoonFormation());

    	app->sendUnicast(res, msg->getVehicleId());
		app->setPlatoonRole(PlatoonRole::FOLLOWER);
//		joinManeuverState = JoinManeuverState::L_WAIT_J_HANDOFF;
		joinManeuverState = JoinManeuverState::J_ADJUST;
    }

}

void DynamicJoin::handleHandoffLeadershipResponse(const HandoffLeadershipResponse* msg)
{
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - handleHandoffLeadershipResponse" << std::endl;
    if (app->getPlatoonRole() != PlatoonRole::JOINER) return;
    if (joinManeuverState != JoinManeuverState::J_WAIT_L_HANDOFF) return;
    if (msg->getPlatoonId() != targetPlatoonData->platoonId) return;
    if (msg->getVehicleId() != targetPlatoonData->platoonLeader) return;


    bool permission = msg->getPermitted();

    if (!permission)
    {
        abortManeuver();
        return;
    }

//    targetPlatoonData->from(msg);
//    positionHelper->setLeaderId(positionHelper->getId());
    positionHelper->setPlatoonLane(targetPlatoonData->platoonLane);
    positionHelper->setPlatoonSpeed(targetPlatoonData->platoonSpeed);
    positionHelper->setPlatoonId(positionHelper->getId());
    positionHelper->setLeaderId(positionHelper->getId());

    std::vector<int> formation;
    for (unsigned i = 0; i < msg->getNewPlatoonFormationArraySize(); i++) formation.push_back(msg->getNewPlatoonFormation(i));
//    positionHelper->setPlatoonFormation(targetPlatoonData->newFormation);
    positionHelper->setPlatoonFormation(formation);


//    traciVehicle->setFixedAcceleration(1, 0);
//    traciVehicle->setFixedAcceleration(true, 0);
    traciVehicle->setFixedLane(msg->getPlatoonLane(), false);
    traciVehicle->setSpeed(targetPlatoonData->platoonSpeed);
//    traciVehicle->setSpeed(msg->getPlatoonSpeed());
    traciVehicle->setMaxSpeed(targetPlatoonData->platoonSpeed); //  + (1/3*targetPlatoonData->platoonSpeed)


    app->setPlatoonRole(PlatoonRole::LEADER);
    joinManeuverState = JoinManeuverState::IDLE;
    app->setInManeuver(false);


    // Anunciar o novo líder
    for (unsigned int i = 1; i < positionHelper->getPlatoonSize(); i++)
    {
        LeadershipUpdate* dup = createLeadershipUpdate(positionHelper->getId(),
                positionHelper->getExternalId(), positionHelper->getId(), -1,
                positionHelper->getPlatoonSpeed(), traciVehicle->getLaneIndex(),
                positionHelper->getPlatoonFormation());

        int dest = positionHelper->getMemberId(i);
        dup->setDestinationId(dest);
        app->sendUnicast(dup, dest);
    }

}

void DynamicJoin::handleLeadershipUpdate(const LeadershipUpdate* msg)
{
//    if (msg->getPlatoonId() != targetPlatoonData->platoonId) return;
//    if (msg->getVehicleId() != targetPlatoonData->platoonLeader) return;
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - handleLeadershipUpdate" << std::endl;

	if (app->getPlatoonRole() != PlatoonRole::FOLLOWER) return;

    targetPlatoonData.reset(new TargetPlatoonData());
    targetPlatoonData->from(msg);

    positionHelper->setPlatoonId(targetPlatoonData->platoonLeader);
    positionHelper->setLeaderId(targetPlatoonData->platoonLeader);
//    positionHelper->setPlatoonLane(targetPlatoonData->platoonLane);
    positionHelper->setPlatoonSpeed(targetPlatoonData->platoonSpeed);

//    traciVehicle->setCACCConstantSpacing(15);
    traciVehicle->setLeaderVehicleFakeData(0, 0, targetPlatoonData->platoonSpeed + 1/3*targetPlatoonData->platoonSpeed);
    traciVehicle->setFrontVehicleFakeData(0, 0, targetPlatoonData->platoonSpeed, 15);
    traciVehicle->setCruiseControlDesiredSpeed(targetPlatoonData->platoonSpeed + (30 / 3.6));
    traciVehicle->setActiveController(Plexe::FAKED_CACC);

    std::vector<int> formation;
    for (unsigned i = 0; i < msg->getNewPlatoonFormationArraySize(); i++) formation.push_back(msg->getNewPlatoonFormation(i));
//    positionHelper->setPlatoonFormation(targetPlatoonData->newFormation);
    positionHelper->setPlatoonFormation(formation);

//    app->setInManeuver(true); // Vehicles are not in maneuver, they are ajusting their position to take advantage of platoon
    joinManeuverState = JoinManeuverState::J_ADJUST;

}

void DynamicJoin::handleMoveToPositionAck(const MoveToPositionAck* msg)
{
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - handleMoveToPositionAck" << std::endl;
    if (app->getPlatoonRole() != PlatoonRole::LEADER) return;
    if (joinManeuverState != JoinManeuverState::L_WAIT_JOINER_IN_POSITION) return;
    if (msg->getPlatoonId() != positionHelper->getPlatoonId()) return;

//    TODO Tratar o caso de ter múltiplos joiners
//    if (msg->getVehicleId() != joinerData->joinerId) return;

    for (unsigned i = 0; i < msg->getNewPlatoonFormationArraySize(); i++) ASSERT(msg->getNewPlatoonFormation(i) == joinerData->newFormation.at(i));

    // the joiner is now in position and is ready to join

    // tell the joiner to join the platoon
    JoinFormation* jf = createJoinFormation(positionHelper->getId(), positionHelper->getExternalId(), positionHelper->getPlatoonId(), joinerData->joinerId, positionHelper->getPlatoonSpeed(), traciVehicle->getLaneIndex(), joinerData->newFormation);
    app->sendUnicast(jf, joinerData->joinerId);
    joinManeuverState = JoinManeuverState::L_WAIT_JOINER_TO_JOIN;

}

void DynamicJoin::handleJoinFormation(const JoinFormation* msg)
{
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - handleJoinFormation" << std::endl;
    // NOTE Alterado para aceitar veículos que entram no pelotão e passam a ser "FOLLOWER"
//    if ((app->getPlatoonRole() != PlatoonRole::JOINER) || app->getPlatoonRole() != PlatoonRole::FOLLOWER) return;
    if (joinManeuverState != JoinManeuverState::J_WAIT_JOIN) return;
    if (msg->getPlatoonId() != targetPlatoonData->platoonId) return;
    if (msg->getVehicleId() != targetPlatoonData->platoonLeader) return;

    for (unsigned i = 0; i < msg->getNewPlatoonFormationArraySize(); i++) ASSERT(msg->getNewPlatoonFormation(i) == targetPlatoonData->newFormation[i]);

    // we got confirmation from the leader
    // switch from faked CACC to real CACC
    traciVehicle->setActiveController(Plexe::CACC);
    // set spacing to 5 meters to get close to the platoon
    traciVehicle->setCACCConstantSpacing(5);

    // update platoon information
    positionHelper->setPlatoonId(msg->getPlatoonId());
    positionHelper->setPlatoonLane(targetPlatoonData->platoonLane);
    positionHelper->setPlatoonSpeed(targetPlatoonData->platoonSpeed);
    std::vector<int> formation;
    for (unsigned i = 0; i < msg->getNewPlatoonFormationArraySize(); i++) formation.push_back(msg->getNewPlatoonFormation(i));
    positionHelper->setPlatoonFormation(formation);

    // tell the leader that we're now in the platoon
    JoinFormationAck* jfa = createJoinFormationAck(positionHelper->getId(), positionHelper->getExternalId(), positionHelper->getPlatoonId(), targetPlatoonData->platoonLeader, positionHelper->getPlatoonSpeed(), traciVehicle->getLaneIndex(), formation);
    app->sendUnicast(jfa, positionHelper->getLeaderId());

    app->setPlatoonRole(PlatoonRole::FOLLOWER);
    joinManeuverState = JoinManeuverState::IDLE;
    app->setOngoingManeuver(PlatoonManeuver::IDLE);

    app->setInManeuver(false);


}

void DynamicJoin::handleJoinFormationAck(const JoinFormationAck* msg)
{
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - handleJoinFormationAck" << std::endl;

    if (app->getPlatoonRole() != PlatoonRole::LEADER) return;
    if (joinManeuverState != JoinManeuverState::L_WAIT_JOINER_TO_JOIN) return;
    if (msg->getPlatoonId() != positionHelper->getPlatoonId()) return;
    if (msg->getVehicleId() != joinerData->joinerId) return;
    for (unsigned i = 0; i < msg->getNewPlatoonFormationArraySize(); i++) ASSERT(msg->getNewPlatoonFormation(i) == joinerData->newFormation.at(i));

    // the joiner has joined the platoon
    // add the joiner to the list of vehicles in the platoon
    positionHelper->setPlatoonFormation(joinerData->newFormation);

    // send to all vehicles in Platoon
    for (unsigned int i = 1; i < positionHelper->getPlatoonSize(); i++)
    {
        UpdatePlatoonFormation* dup = app->createUpdatePlatoonFormation(positionHelper->getId(), positionHelper->getExternalId(), positionHelper->getPlatoonId(), -1, positionHelper->getPlatoonSpeed(), traciVehicle->getLaneIndex(), joinerData->newFormation);
        int dest = positionHelper->getMemberId(i);
        dup->setDestinationId(dest);
        app->sendUnicast(dup, dest);
    }

    joinManeuverState = JoinManeuverState::IDLE;
    app->setInManeuver(false);
}

void DynamicJoin::handleSplitFormation(const SplitFormation* msg)
{
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - handleSplitFormation" << std::endl;

//    if (app->getPlatoonRole() != PlatoonRole::NONE) return;
//    if (app->getPlatoonRole() != PlatoonRole::FOLLOWER) return;
    if (joinManeuverState != JoinManeuverState::IDLE) return;
    if (msg->getPlatoonId() != positionHelper->getPlatoonId()) return;
    if (msg->getVehicleId() != positionHelper->getLeaderId()) return;


    traciVehicle->setSpeed(0.7 * msg->getPlatoonSpeed());
    traciVehicle->setCACCConstantSpacing(msg->getGap());

    std::vector<int> formation;
    for (unsigned i = 0; i < msg->getNewPlatoonFormationArraySize(); i++) formation.push_back(msg->getNewPlatoonFormation(i));
    positionHelper->setPlatoonFormation(formation);

}

