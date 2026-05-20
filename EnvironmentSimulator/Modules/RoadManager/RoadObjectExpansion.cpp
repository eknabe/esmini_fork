/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "RoadObjectExpansion.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <sstream>

namespace roadmanager
{
    namespace
    {
        constexpr double kEpsilon                 = 1e-6;
        constexpr double kDefaultContinuousSegLen = 10.0;

        double Lerp(double a, double b, double factor)
        {
            return a + factor * (b - a);
        }

        double Resolve(const std::optional<double> &value, double fallback)
        {
            return value.has_value() ? value.value() : fallback;
        }

        /// Convert road frame (s, t) to world-space XY by walking the road's geometry list.
        void RoadST2WorldXY(const Road &road, double s, double t, double &out_x, double &out_y)
        {
            Geometry *geom = nullptr;
            for (unsigned int gi = 0; gi < road.GetNumberOfGeometries(); ++gi)
            {
                Geometry *g = road.GetGeometry(gi);
                if (g == nullptr)
                {
                    continue;
                }
                // Accept the first geometry whose range covers s.
                if (s <= g->GetS() + g->GetLength() + kEpsilon)
                {
                    geom = g;
                    break;
                }
            }
            if (geom == nullptr)
            {
                // Fallback: use the last geometry on the road.
                const unsigned int n = road.GetNumberOfGeometries();
                if (n > 0)
                {
                    geom = road.GetGeometry(n - 1);
                }
            }
            if (geom == nullptr)
            {
                out_x = 0.0;
                out_y = 0.0;
                return;
            }

            double       cx, cy, h;
            const double ds = std::max(0.0, std::min(s - geom->GetS(), geom->GetLength()));
            geom->EvaluateDS(ds, &cx, &cy, &h);

            // Lateral offset perpendicular to road heading (positive t = left of travel direction).
            out_x = cx - t * std::sin(h);
            out_y = cy + t * std::cos(h);
        }

        /// Intersect two infinite lines in 2D, each defined by two points.
        /// Returns true when a clean intersection was found; on near-parallel lines
        /// returns false and sets out to the midpoint of a1 and b0 as a safe fallback.
        bool IntersectLines2D(Vec2 a0, Vec2 a1, Vec2 b0, Vec2 b1, Vec2 &out)
        {
            const double d1x = a1.x - a0.x;
            const double d1y = a1.y - a0.y;
            const double d2x = b1.x - b0.x;
            const double d2y = b1.y - b0.y;
            const double det = d1x * d2y - d1y * d2x;
            if (std::fabs(det) < kEpsilon)
            {
                // Parallel – use midpoint of the two nominal boundary points.
                out.x = 0.5 * (a1.x + b0.x);
                out.y = 0.5 * (a1.y + b0.y);
                return false;
            }
            const double dx    = b0.x - a0.x;
            const double dy    = b0.y - a0.y;
            const double param = (dx * d2y - dy * d2x) / det;
            out.x              = a0.x + param * d1x;
            out.y              = a0.y + param * d1y;
            return true;
        }

        std::optional<double> ResolveOptionalInterpolated(const std::optional<double> &start,
                                                          const std::optional<double> &end,
                                                          const std::optional<double> &fallback,
                                                          double                       factor)
        {
            if (start.has_value() || end.has_value())
            {
                const double s = Resolve(start, Resolve(fallback, 0.0));
                const double e = Resolve(end, s);
                return Lerp(s, e, factor);
            }
            return fallback;
        }

        std::optional<double> ResolveSizeInterpolatedWithRadius(const RMRepeatDefinition &rep,
                                                                const RMObjectDefinition &def,
                                                                bool                      use_length,
                                                                double                    factor)
        {
            const std::optional<double> radius = ResolveOptionalInterpolated(rep.radiusStart, rep.radiusEnd, def.radius, factor);
            if (radius.has_value())
            {
                return 2.0 * radius.value();
            }

            if (use_length)
            {
                return ResolveOptionalInterpolated(rep.lengthStart, rep.lengthEnd, def.length, factor);
            }
            return ResolveOptionalInterpolated(rep.widthStart, rep.widthEnd, def.width, factor);
        }

        std::optional<double> ResolveDefinitionSizeWithRadius(const RMObjectDefinition &def, bool use_length)
        {
            if (def.radius.has_value())
            {
                return 2.0 * def.radius.value();
            }
            return use_length ? def.length : def.width;
        }

        std::optional<double> ResolveRadiusWithInterpolation(const RMRepeatDefinition &rep, const RMObjectDefinition &def, double factor)
        {
            return ResolveOptionalInterpolated(rep.radiusStart, rep.radiusEnd, def.radius, factor);
        }

        std::optional<double> ResolveRadiusFromDefinition(const RMObjectDefinition &def)
        {
            return def.radius;
        }

        RMExpandedObject BuildFromDefinition(const RMObjectDefinition &def)
        {
            RMExpandedObject out;
            out.kind           = RMExpandedObject::Kind::SINGLE_ANCHOR;
            out.sourceObjectId = def.id;
            out.s              = def.s;
            out.sEnd           = def.s;
            out.t              = def.t;
            out.tEnd           = def.t;
            out.zOffset        = def.zOffset;
            out.zOffsetEnd     = def.zOffset;
            out.hdg            = def.hdg;
            out.pitch          = def.pitch;
            out.roll           = def.roll;
            out.length         = ResolveDefinitionSizeWithRadius(def, true);
            out.width          = ResolveDefinitionSizeWithRadius(def, false);
            out.height         = def.height;
            out.heightEnd      = out.height;
            out.radius         = ResolveRadiusFromDefinition(def);
            return out;
        }
    }  // namespace

    std::vector<RMExpandedObject> ExpandRoadObjectDefinitions(const Road &road)
    {
        std::vector<RMExpandedObject> expanded;
        expanded.reserve(road.GetNumberOfObjectDefinitions() * 2);

        for (unsigned int i = 0; i < road.GetNumberOfObjectDefinitions(); ++i)
        {
            const RMObjectDefinition *def = road.GetObjectDefinition(i);
            if (def == nullptr)
            {
                continue;
            }

            if (def->repeats.empty())
            {
                if (!def->outlines.empty())
                {
                    // Each outline independently defines the shape of this object;
                    // emit one OUTLINE_OBJECT entry per outline so the viewer can
                    // delegate to CreateOutlineObject().
                    for (int oi = 0; oi < static_cast<int>(def->outlines.size()); ++oi)
                    {
                        RMExpandedObject out;
                        out.kind           = RMExpandedObject::Kind::OUTLINE_OBJECT;
                        out.sourceObjectId = def->id;
                        out.outlineIndex   = oi;
                        out.s              = def->s;
                        out.t              = def->t;
                        out.zOffset        = def->zOffset;
                        out.zOffsetEnd     = def->zOffset;
                        out.hdg            = def->hdg;
                        out.pitch          = def->pitch;
                        out.roll           = def->roll;
                        expanded.push_back(out);
                    }
                }
                else
                {
                    expanded.push_back(BuildFromDefinition(*def));
                }
                continue;
            }

            for (size_t repeat_idx = 0; repeat_idx < def->repeats.size(); ++repeat_idx)
            {
                const RMRepeatDefinition &rep                   = def->repeats[repeat_idx];
                const double              rep_length            = std::max(0.0, rep.length);
                const bool                strict_outline_repeat = !def->outlines.empty();
                const double              t_start               = strict_outline_repeat ? Resolve(rep.tStart, 0.0) : Resolve(rep.tStart, def->t);
                const double              t_end                 = Resolve(rep.tEnd, t_start);
                const double              z_start = strict_outline_repeat ? Resolve(rep.zOffsetStart, 0.0) : Resolve(rep.zOffsetStart, def->zOffset);
                const double              z_end   = Resolve(rep.zOffsetEnd, z_start);

                if (rep.distance > kEpsilon)
                {
                    const bool has_span = rep_length > kEpsilon;

                    for (double cur_s = 0.0;; cur_s += rep.distance)
                    {
                        if (has_span && cur_s > rep_length + kEpsilon)
                        {
                            break;
                        }
                        if (!has_span && cur_s > kEpsilon)
                        {
                            break;
                        }

                        const double factor = has_span ? std::min(1.0, std::max(0.0, cur_s / rep_length)) : 0.0;

                        RMExpandedObject out;
                        out.kind           = RMExpandedObject::Kind::DISCRETE_REPEAT_INSTANCE;
                        out.sourceObjectId = def->id;
                        out.repeatIndex    = static_cast<int>(repeat_idx);
                        out.s              = rep.s + cur_s;
                        out.sEnd           = out.s;
                        out.t              = Lerp(t_start, t_end, factor);
                        out.tEnd           = out.t;
                        out.zOffset        = Lerp(z_start, z_end, factor);
                        out.zOffsetEnd     = out.zOffset;
                        out.hdg            = def->hdg;
                        out.pitch          = def->pitch;
                        out.roll           = def->roll;
                        out.length         = ResolveSizeInterpolatedWithRadius(rep, *def, true, factor);
                        out.width          = ResolveSizeInterpolatedWithRadius(rep, *def, false, factor);
                        out.height         = ResolveOptionalInterpolated(rep.heightStart, rep.heightEnd, def->height, factor);
                        out.heightEnd      = out.height;
                        out.radius         = ResolveRadiusWithInterpolation(rep, *def, factor);

                        // Keep generated instances within road range.
                        if (out.s <= road.GetLength() + kEpsilon)
                        {
                            if (def->outlines.empty())
                            {
                                expanded.push_back(out);
                            }
                            else
                            {
                                for (int oi = 0; oi < static_cast<int>(def->outlines.size()); ++oi)
                                {
                                    RMExpandedObject outline_out = out;
                                    outline_out.kind             = RMExpandedObject::Kind::OUTLINE_OBJECT;
                                    outline_out.outlineIndex     = oi;
                                    expanded.push_back(outline_out);
                                }
                            }
                        }

                        if (!has_span)
                        {
                            break;
                        }
                    }
                }
                else
                {
                    double                      segment_length = kDefaultContinuousSegLen;
                    const std::optional<double> def_length     = ResolveDefinitionSizeWithRadius(*def, true);
                    if (def_length.has_value() && def_length.value() > kEpsilon)
                    {
                        segment_length = std::min(segment_length, def_length.value());
                    }
                    if (segment_length < kEpsilon)
                    {
                        segment_length = kDefaultContinuousSegLen;
                    }

                    const unsigned int n_segments =
                        rep_length > kEpsilon ? std::max(1u, static_cast<unsigned int>(std::ceil(rep_length / segment_length))) : 1u;

                    // Collect all segments for this repeat before applying edge stitching.
                    std::vector<RMExpandedObject>          segs;
                    std::vector<std::pair<double, double>> half_widths;  // (hw_start, hw_end) per segment
                    segs.reserve(n_segments);
                    half_widths.reserve(n_segments);

                    for (unsigned int seg_idx = 0; seg_idx < n_segments; ++seg_idx)
                    {
                        const double f0     = static_cast<double>(seg_idx) / static_cast<double>(n_segments);
                        const double f1     = static_cast<double>(seg_idx + 1) / static_cast<double>(n_segments);
                        const double cur_s0 = rep_length * f0;
                        const double cur_s1 = rep_length * f1;

                        RMExpandedObject out;
                        out.kind           = RMExpandedObject::Kind::CONTINUOUS_REPEAT_SEGMENT;
                        out.sourceObjectId = def->id;
                        out.repeatIndex    = static_cast<int>(repeat_idx);
                        out.segmentIndex   = static_cast<int>(seg_idx);
                        out.s              = rep.s + cur_s0;
                        out.sEnd           = rep.s + cur_s1;
                        out.t              = Lerp(t_start, t_end, f0);
                        out.tEnd           = Lerp(t_start, t_end, f1);
                        out.zOffset        = Lerp(z_start, z_end, f0);
                        out.zOffsetEnd     = Lerp(z_start, z_end, f1);
                        out.hdg            = def->hdg;
                        out.pitch          = def->pitch;
                        out.roll           = def->roll;
                        out.length         = ResolveSizeInterpolatedWithRadius(rep, *def, true, 0.5 * (f0 + f1));
                        out.width          = ResolveSizeInterpolatedWithRadius(rep, *def, false, 0.5 * (f0 + f1));
                        out.height         = ResolveOptionalInterpolated(rep.heightStart, rep.heightEnd, def->height, f0);
                        out.heightEnd      = ResolveOptionalInterpolated(rep.heightStart, rep.heightEnd, def->height, f1);
                        out.radius         = ResolveRadiusWithInterpolation(rep, *def, 0.5 * (f0 + f1));

                        if (out.s <= road.GetLength() + kEpsilon)
                        {
                            out.sEnd = std::min(out.sEnd, road.GetLength());

                            // Per-segment widths at both ends for corner computation.
                            const double w0 = Resolve(ResolveSizeInterpolatedWithRadius(rep, *def, false, f0), 0.0);
                            const double w1 = Resolve(ResolveSizeInterpolatedWithRadius(rep, *def, false, f1), 0.0);
                            half_widths.emplace_back(0.5 * w0, 0.5 * w1);
                            segs.push_back(out);
                        }
                    }

                    // Compute world-space polygon corners for every segment.
                    // Corner order: 0=start/neg-t  1=start/pos-t  2=end/pos-t  3=end/neg-t
                    for (size_t si = 0; si < segs.size(); ++si)
                    {
                        RMExpandedObject &seg = segs[si];
                        const double      hw0 = half_widths[si].first;
                        const double      hw1 = half_widths[si].second;

                        RoadST2WorldXY(road, seg.s, seg.t - hw0, seg.world_corners[0].x, seg.world_corners[0].y);
                        RoadST2WorldXY(road, seg.s, seg.t + hw0, seg.world_corners[1].x, seg.world_corners[1].y);
                        RoadST2WorldXY(road, seg.sEnd, seg.tEnd + hw1, seg.world_corners[2].x, seg.world_corners[2].y);
                        RoadST2WorldXY(road, seg.sEnd, seg.tEnd - hw1, seg.world_corners[3].x, seg.world_corners[3].y);
                        seg.has_world_corners = true;
                    }

                    // Geometric edge stitching: for each pair of adjacent segments snap the
                    // shared boundary vertices to the intersection of their respective edge
                    // lines in world-space XY, eliminating gaps/overlaps on curved roads.
                    for (size_t si = 0; si + 1 < segs.size(); ++si)
                    {
                        RMExpandedObject &cur  = segs[si];
                        RMExpandedObject &next = segs[si + 1];

                        Vec2 stitch_pos, stitch_neg;

                        // Positive-t edge: corners[1] → corners[2]
                        IntersectLines2D(cur.world_corners[1], cur.world_corners[2], next.world_corners[1], next.world_corners[2], stitch_pos);
                        // Negative-t edge: corners[0] → corners[3]
                        IntersectLines2D(cur.world_corners[0], cur.world_corners[3], next.world_corners[0], next.world_corners[3], stitch_neg);

                        cur.world_corners[2]  = stitch_pos;
                        next.world_corners[1] = stitch_pos;
                        cur.world_corners[3]  = stitch_neg;
                        next.world_corners[0] = stitch_neg;
                    }

                    std::transform(segs.begin(), segs.end(), std::back_inserter(expanded), [](RMExpandedObject &seg) { return std::move(seg); });
                }
            }
        }

        return expanded;
    }

    std::string BuildRoadObjectDebugDump(const OpenDrive &odr, unsigned int maxDetailsPerRoad)
    {
        std::ostringstream ss;
        ss << "RoadObjectDebugDump roads=" << odr.GetNumOfRoads() << "\n";

        for (unsigned int r = 0; r < odr.GetNumOfRoads(); ++r)
        {
            const Road *road = odr.GetRoadByIdx(r);
            if (road == nullptr)
            {
                continue;
            }

            const std::vector<RMExpandedObject> expanded     = ExpandRoadObjectDefinitions(*road);
            const unsigned int                  legacy_n     = road->GetNumberOfObjects();
            const unsigned int                  defs_n       = road->GetNumberOfObjectDefinitions();
            unsigned int                        single_n     = 0;
            unsigned int                        discrete_n   = 0;
            unsigned int                        continuous_n = 0;
            unsigned int                        outline_n    = 0;

            for (const auto &exp : expanded)
            {
                if (exp.kind == RMExpandedObject::Kind::SINGLE_ANCHOR)
                {
                    single_n++;
                }
                else if (exp.kind == RMExpandedObject::Kind::DISCRETE_REPEAT_INSTANCE)
                {
                    discrete_n++;
                }
                else if (exp.kind == RMExpandedObject::Kind::CONTINUOUS_REPEAT_SEGMENT)
                {
                    continuous_n++;
                }
                else if (exp.kind == RMExpandedObject::Kind::OUTLINE_OBJECT)
                {
                    outline_n++;
                }
            }

            ss << "road_id=" << road->GetId() << " legacy_objects=" << legacy_n << " definitions=" << defs_n << " expanded=" << expanded.size()
               << " expanded_single=" << single_n << " expanded_discrete=" << discrete_n << " expanded_continuous=" << continuous_n
               << " expanded_outline=" << outline_n << "\n";

            const unsigned int n = std::min(defs_n, legacy_n);
            for (unsigned int i = 0; i < n && i < maxDetailsPerRoad; ++i)
            {
                const RMObject           *legacy = road->GetRoadObject(i);
                const RMObjectDefinition *def    = road->GetObjectDefinition(i);
                if (legacy == nullptr)
                {
                    continue;
                }
                if (def == nullptr)
                {
                    continue;
                }

                ss << "  idx=" << i << " id=" << def->id << " ds=" << std::fabs(legacy->GetS() - def->s)
                   << " dt=" << std::fabs(legacy->GetT() - def->t) << " dzOffset=" << std::fabs(legacy->GetZOffset() - def->zOffset)
                   << " dh=" << std::fabs(legacy->GetHOffset() - def->hdg) << " dp=" << std::fabs(legacy->GetPitch() - def->pitch)
                   << " dr=" << std::fabs(legacy->GetRoll() - def->roll) << "\n";
            }
        }

        return ss.str();
    }

}  // namespace roadmanager
