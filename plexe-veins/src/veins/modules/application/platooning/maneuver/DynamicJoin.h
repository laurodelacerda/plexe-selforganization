
#ifndef DYNAMIC_JOIN_H
#define DYNAMIC_JOIN_H

#include <algorithm>

#include "veins/modules/application/platooning/maneuver/JoinManeuver.h"
#include "veins/modules/application/platooning/utilities/BasePositionHelper.h"

#include "veins/modules/mobility/traci/TraCIConstants.h"
#include "veins/base/utils/Coord.h"

#define PLATOON_ZONE 30
#define FIXED_ACCELERATION 2
#define FIXED_DECELERATION -2

using namespace Veins;

class DynamicJoin : public JoinManeuver {

public:

    DynamicJoin(GeneralPlatooningApp* app);

    virtual ~DynamicJoin();

    virtual void startManeuver(const void* parameters) override;

    virtual void abortManeuver() override;

    virtual void onPlatoonBeacon(const PlatooningBeacon* pb) override;

    virtual void handleJoinPlatoonRequest(const JoinPlatoonRequest* msg) override;

    virtual void handleJoinPlatoonResponse(const JoinPlatoonResponse* msg) override;

    virtual void handleMoveToPosition(const MoveToPosition* msg) override;

    virtual void handleMoveToPositionAck(const MoveToPositionAck* msg) override;

    virtual void handleJoinFormation(const JoinFormation* msg) override;

    virtual void handleJoinFormationAck(const JoinFormationAck* msg) override;

    virtual void handleHandoffLeadershipRequest(const HandoffLeadershipRequest* msg) override;

    virtual void handleHandoffLeadershipResponse(const HandoffLeadershipResponse* msg) override;

    virtual void handleLeadershipUpdate(const LeadershipUpdate* msg) override;

    virtual void handleSplitFormation(const SplitFormation* msg) override;

    virtual void moveToPosition();

protected:

    enum class JoinManeuverState {
        IDLE, ///< The maneuver did not start
        // Joiner
        J_WAIT_REPLY, ///< The joiner sent a JoinRequest waits for a reply from the Platoon leader
        J_WAIT_INFORMATION, ///< The joiner waits for information about the Platoon
        J_MOVE_IN_POSITION, ///< The joiner moves to its position
        J_WAIT_L_HANDOFF, ///< [JaF] The ex-leader is waiting for leadership handoff
        J_ADJUST,         ///< [JaF] The ex-leader is adjusting its position
        J_WAIT_JOIN, ///< The joiner waits for the join permission
        // Leader
        L_WAIT_J_HANDOFF, ///< [JaF] The current leader is waiting for leadership handoff
//        L_ADJUST,         ///< [JaF] The current leader is waiting for platoon adjustment
        L_WAIT_JOINER_IN_POSITION, ///< The leader waits for the joiner to be in position, the followers made space already
        L_WAIT_JOINER_TO_JOIN, ///< The leader waits for the joiner to join
    };

    /** data that a joiner stores about a Platoon it wants to join */
    struct TargetPlatoonData {
        int platoonId; ///< the id of the platoon to join
        int platoonLeader; ///< the if ot the leader of the platoon
        int platoonLane; ///< the lane the platoon is driving on
        double platoonSpeed; ///< the speed of the platoon
        int joinIndex; ///< position in the new platoon formation, 0 based !
        std::vector<int> newFormation; ///< the new formation of the platoon
        Coord lastFrontPos; ///< the last kwown position of the front vehicle

        /** c'tor for TargetPlatoonData */
        TargetPlatoonData()
        {
            platoonId = INVALID_PLATOON_ID;
            platoonLeader = INVALID_INT_VALUE;
            platoonLane = INVALID_INT_VALUE;
            platoonSpeed = INVALID_DOUBLE_VALUE;
            joinIndex = INVALID_INT_VALUE;
        }

        /**
         * Initializes the TargetPlatoonData object with the contents of a
         * MoveToPosition
         *
         * @param MoveToPosition msg to initialize from
         * @see MoveToPosition
         */
        void from(const MoveToPosition* msg)
        {
            platoonId = msg->getPlatoonId();
            platoonLeader = msg->getVehicleId();
            platoonLane = msg->getPlatoonLane();
            platoonSpeed = msg->getPlatoonSpeed();
            newFormation.resize(msg->getNewPlatoonFormationArraySize());
            for (unsigned int i = 0; i < msg->getNewPlatoonFormationArraySize(); i++) {
                newFormation[i] = msg->getNewPlatoonFormation(i);
            }
            const auto it = std::find(newFormation.begin(), newFormation.end(), msg->getDestinationId());
            if (it != newFormation.end()) {
                joinIndex = std::distance(newFormation.begin(), it);
                ASSERT(newFormation.at(joinIndex) == msg->getDestinationId());
            }
        }

        void from(const LeadershipUpdate* msg)
        {
            platoonId = msg->getPlatoonId();
            platoonLeader = msg->getVehicleId();
            platoonLane = msg->getPlatoonLane();
            platoonSpeed = msg->getPlatoonSpeed();
            newFormation.resize(msg->getNewPlatoonFormationArraySize());
            for (unsigned int i = 0; i < msg->getNewPlatoonFormationArraySize(); i++) {
                newFormation[i] = msg->getNewPlatoonFormation(i);
            }
            const auto it = std::find(newFormation.begin(), newFormation.end(), msg->getDestinationId());
            if (it != newFormation.end()) {
                joinIndex = std::distance(newFormation.begin(), it);
                ASSERT(newFormation.at(joinIndex) == msg->getDestinationId());
            }
        }



        /**
         * Get the id of the front vehicle
         *
         * @return int the id of the front vehicle
         */
        int frontId() const
        {
            return newFormation.at(joinIndex - 1);
        }
    };

    /** data that a leader stores about a joining vehicle */
    struct JoinerData {
        int joinerId; ///< the id of the vehicle joining the Platoon
        int joinerLane; ///< the lane chosen for joining the Platoon
        std::vector<int> newFormation;

        /** c'tor for JoinerData */
        JoinerData()
        {
            joinerId = INVALID_INT_VALUE;
            joinerLane = INVALID_INT_VALUE;
        }

        /**
         * Initializes the JoinerData object with the contents of a
         * JoinPlatoonRequest
         *
         * @param JoinPlatoonRequest jr to initialize from
         * @see JoinPlatoonRequest
         */
        void from(const JoinPlatoonRequest* msg)
        {
            joinerId = msg->getVehicleId();
            joinerLane = msg->getCurrentLaneIndex();
        }
    };

    /** the current state in the join maneuver */
    JoinManeuverState joinManeuverState;

    /** the data about the target platoon */
    std::unique_ptr<TargetPlatoonData> targetPlatoonData;

    /** the data about the current joiner */
    std::unique_ptr<JoinerData> joinerData;

};

#endif
