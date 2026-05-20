/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef ROAD_OBJECT_EXPANSION_HPP_
#define ROAD_OBJECT_EXPANSION_HPP_

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "RoadManager.hpp"

namespace roadmanager
{
    /// 2-D world-space point used for polygon corner storage.
    struct Vec2
    {
        double x = 0.0;
        double y = 0.0;
    };

    struct RMExpandedObject
    {
        enum class Kind
        {
            SINGLE_ANCHOR,
            DISCRETE_REPEAT_INSTANCE,
            CONTINUOUS_REPEAT_SEGMENT,
            OUTLINE_OBJECT
        };

        Kind kind = Kind::SINGLE_ANCHOR;

        id_t   sourceObjectId = ID_UNDEFINED;
        int    repeatIndex    = -1;
        int    segmentIndex   = -1;
        int    outlineIndex   = -1;
        double s              = 0.0;
        double sEnd           = 0.0;
        double t              = 0.0;
        double tEnd           = 0.0;
        double zOffset        = 0.0;
        double zOffsetEnd     = 0.0;
        double hdg            = 0.0;
        double pitch          = 0.0;
        double roll           = 0.0;

        std::optional<double> length;
        std::optional<double> width;
        std::optional<double> height;
        std::optional<double> heightEnd;
        std::optional<double> radius;

        /// World-space XY polygon corners, populated only when has_world_corners == true.
        /// Corner ordering (index): 0=start/neg-t  1=start/pos-t  2=end/pos-t  3=end/neg-t
        bool                has_world_corners = false;
        std::array<Vec2, 4> world_corners     = {};
    };

    // Placeholder expander for early migration stages.
    // Current behavior emits one expanded anchor object per object definition.
    std::vector<RMExpandedObject> ExpandRoadObjectDefinitions(const Road &road);

    // Debug helper to compare legacy object anchors vs expanded objects.
    std::string BuildRoadObjectDebugDump(const OpenDrive &odr, unsigned int maxDetailsPerRoad = 5);

}  // namespace roadmanager

#endif  // ROAD_OBJECT_EXPANSION_HPP_
