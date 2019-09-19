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

#include "ClusterJoin.h"

#include "veins/modules/application/platooning/apps/GeneralPlatooningApp.h"

using namespace Veins;

ClusterJoin::ClusterJoin(GeneralPlatooningApp* app)
    : JoinManeuver(app),
      joinManeuverState(JoinManeuverState::IDLE)
{
    // TODO Auto-generated constructor stub
}

ClusterJoin::~ClusterJoin()
{

}

void ClusterJoin::abortManeuver()
{
    joinManeuverState = JoinManeuverState::IDLE;
    app->setPlatoonRole(PlatoonRole::NONE);
    app->setInManeuver(false);
}

void ClusterJoin::startManeuver(const void* parameters)
{

    std::cout << "(" << simTime() << ")" <<  " > Vehicle: " << positionHelper->getId() << " - startManeuver" << std::endl;
    JoinManeuverParameters* pars = (JoinManeuverParameters*) parameters;

    Plexe::VEHICLE_DATA data;
    traciVehicle->getVehicleData(&data);

	// Get Geo-positional formation
	std::vector<int> formation = app->getMapFormation(traciVehicle->getLaneIndex());

    // Am I in a safe Lane?
    if (app->isLaneSafe(positionHelper->getId()))
    {
    	if (app->isLaneLeader(positionHelper->getId())) // Change Safe Leader speed and control
    	{
    		app->setPlatoonRole(PlatoonRole::LEADER);
        	app->setInManeuver(false);

//    		traciVehicle->setActiveController(Plexe::ACC);
    		traciVehicle->setSpeed(data.speed * SPEED_CONTROL_FACTOR);
    		traciVehicle->setCruiseControlDesiredSpeed(data.speed);

    		positionHelper->setPlatoonId(positionHelper->getId());
    		positionHelper->setLeaderId(positionHelper->getId());
    	    positionHelper->setPlatoonLane(traciVehicle->getLaneIndex());
    	    positionHelper->setPlatoonSpeed(data.speed * SPEED_CONTROL_FACTOR);
    	    positionHelper->setPlatoonFormation(formation);

    	    // Send message asking followers to adjust
    	    for (unsigned int i = 1; i < positionHelper->getPlatoonSize(); i++)
    	    {
        		InitPlatoon* ini = new InitPlatoon("InitPlatoon");
        	    app->fillManeuverMessage(ini, positionHelper->getId(), positionHelper->getExternalId(), positionHelper->getPlatoonId(), -1);

        	    ini->setPlatoonSpeed(positionHelper->getPlatoonSpeed());
        	    ini->setPlatoonLane(positionHelper->getPlatoonLane());

    	        int dest = positionHelper->getMemberId(i);
    	        ini->setDestinationId(dest);
    	        app->sendUnicast(ini, dest);
    	    }

    	    joinManeuverState = JoinManeuverState::L_ADJUST;

    	    app->collectStats();
    	}

		return;
    }
    else
    {
    	int my_position = app->getBestAvailableEntryPosition(positionHelper->getId());

    	std::vector<int> joiners;

    	for (unsigned int i = 1; i < formation.size(); i++)
    	{
    		int pos = app->getBestAvailableEntryPosition(formation.at(i));
    		if (pos == my_position) joiners.push_back(formation.at(i));
    	}

    	if (joiners.size() > 0)
    	{
			for (int i : joiners)
			{
				InitPlatoon* ini = new InitPlatoon("InitPlatoon");
				app->fillManeuverMessage(ini, positionHelper->getId(), positionHelper->getExternalId(), positionHelper->getPlatoonId(), -1);

				ini->setPlatoonSpeed(positionHelper->getPlatoonSpeed());
				ini->setPlatoonLane(positionHelper->getPlatoonLane());

				ini->setDestinationId(i);
				app->sendUnicast(ini, i);
			}
    	}

    }



}

void ClusterJoin::onPlatoonBeacon(const PlatooningBeacon* pb)
{
//    std::cout << " > Vehicle: " << positionHelper->getId() << " - onPlatoonBeacon from "<< pb->getVehicleId() << std::endl;

	if (app->getPlatoonRole() == PlatoonRole::FOLLOWER)
	{
        if (pb->getVehicleId() == positionHelper->getPlatoonFormation().at(0))
        {
            traciVehicle->setLeaderVehicleFakeData(pb->getControllerAcceleration(), pb->getAcceleration(), pb->getSpeed());
        }

        // if the message comes from the front vehicle
		int frontPosition = positionHelper->getPosition() - 1;
        int frontId = positionHelper->getPlatoonFormation().at(frontPosition);

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

        }
	}

}

void ClusterJoin::handleJoinPlatoonRequest(const JoinPlatoonRequest* msg)
{
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - handleJoinPlatoonRequest" << std::endl;

}

void ClusterJoin::handleJoinPlatoonResponse(const JoinPlatoonResponse* msg)
{
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - handleJoinPlatoonResponse" << std::endl;

}

void ClusterJoin::handleMoveToPosition(const MoveToPosition* msg)
{
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - handleMoveToPosition" << std::endl;


}

void ClusterJoin::moveToPosition()
{
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - moveToPosition" << std::endl;

}

void ClusterJoin::handleHandoffLeadershipRequest(const HandoffLeadershipRequest* msg)
{
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - handleHandoffLeadershipRequest" << std::endl;

}

void ClusterJoin::handleHandoffLeadershipResponse(const HandoffLeadershipResponse* msg)
{
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - handleHandoffLeadershipResponse" << std::endl;


}

void ClusterJoin::handleLeadershipUpdate(const LeadershipUpdate* msg)
{
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - handleLeadershipUpdate" << std::endl;

}

void ClusterJoin::handleMoveToPositionAck(const MoveToPositionAck* msg)
{
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - handleMoveToPositionAck" << std::endl;

}

void ClusterJoin::handleJoinFormation(const JoinFormation* msg)
{
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - handleJoinFormation" << std::endl;

}

void ClusterJoin::handleJoinFormationAck(const JoinFormationAck* msg)
{
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - handleJoinFormationAck" << std::endl;

}

void ClusterJoin::handleSplitFormation(const SplitFormation* msg)
{
	std::cout << "(" << simTime() << ")" << " > Vehicle: " << positionHelper->getId() << " - handleSplitFormation" << std::endl;

}

void ClusterJoin::handleInitPlatoon(const InitPlatoon* msg)
{
	if (app->isLaneSafe(positionHelper->getId()))
	{
		app->setPlatoonRole(PlatoonRole::FOLLOWER);
		app->setInManeuver(false);
	}
	else
	{
		app->setPlatoonRole(PlatoonRole::UNSAFE_FOLLOWER);
		app->setInManeuver(true);
	}

	std::vector<int> formation = app->getMapFormation(traciVehicle->getLaneIndex());

//	traciVehicle->setSpeed(data.speed * SPEED_CONTROL_FACTOR);
	traciVehicle->setActiveController(Plexe::CACC);
	traciVehicle->setCruiseControlDesiredSpeed(msg->getPlatoonSpeed() + 10);
	traciVehicle->setCACCConstantSpacing(5);

	positionHelper->setPlatoonId(msg->getId());
	positionHelper->setLeaderId(msg->getId());
	positionHelper->setPlatoonLane(traciVehicle->getLaneIndex());
	positionHelper->setPlatoonSpeed(msg->getPlatoonSpeed());
	positionHelper->setPlatoonFormation(formation);

	joinManeuverState = JoinManeuverState::J_ADJUST;
}

void ClusterJoin::handleInitPlatoonAck(const InitPlatoonAck* msg)
{



}
