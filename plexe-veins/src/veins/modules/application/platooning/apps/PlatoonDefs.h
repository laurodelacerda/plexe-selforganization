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

#ifndef PLATOONDEFS_H_
#define PLATOONDEFS_H_

#include <cstddef>
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

/** possible roles of this vehicle */
enum class PlatoonRole : size_t {
    ///< The vehicle is not in a Platoon
    NONE,
    ///< The vehicle is the leader of its Platoon
    LEADER,
    ///< The vehicle is a normal follower in its Platoon
    FOLLOWER,
    ///< The vehicle is in the process of joining a Platoon
    JOINER,
    /// The vehicle is the Platoon leader in a non safe lane
    UNSAFE_LEADER,
    /// The vehicle is in a Platoon located at a non safe lane
    UNSAFE_FOLLOWER,
    /// The vehicle is leading a Platoon in the process of joining another Platoon
    J_UNSAFE_LEADER,
    /// The vehicle is in a Platoon located at a non safe lane which is joining another Platoon
    J_UNSAFE_FOLLOWER
};

enum class DistanceToBlockage : size_t
{
    NONE,
    D_SAFE,
    D_CAUTIOUS,
    D_DANGEROUS
};

enum class VehicleRelativePos : size_t
{
    AHEAD,
    WAY_AHEAD,
    BEHIND,
    WAY_BEHIND
};

// Platoon Maneuvers
enum class PlatoonManeuver : size_t
{
    IDLE,
    JOIN_AT_BACK,
    JOIN_AT_FRONT,
    JOIN_AT_MIDDLE,
    SPLIT,
    MERGE,
    MULTIPLE_JOIN_AT_BACK,
    MULTIPLE_JOIN_AT_FRONT,
    MULTIPLE_JOIN_AT_MIDDLE
};

// Orientation
// TODO Maybe Veins has this class
enum class Orientation : size_t
{
    NONE,
    NORTH,
    SOUTH,
    WEST,
    EAST,
    NORTHWEST, // TODO Implement this
    SOUTHWEST, // TODO Implement this
    NORTHEAST, // TODO Implement this
    SOUTHEAST  // TODO Implement this
};

//     Road Signs or Constraints
enum class SignType : size_t
{
    NARROW,
    TRIANGLE,
    STOP,
    REDUCE
};

struct RoadSignInfo
{
    // Road Sign Id
    std::string sign_id;
    // Road Sign Type
    std::string sign_type;
    // Road Sign Lane
    int lane_index;
    // Vehicle is in the same Lane of Road Sign
    bool same_lane;
    // Road Sign Effect
    bool rule_active;
};

struct IV_Gaps
{
    Veins::Coord front_vehicle;
    Veins::Coord back_vehicle;
};


#endif /* PLATOONDEFS_H_ */
