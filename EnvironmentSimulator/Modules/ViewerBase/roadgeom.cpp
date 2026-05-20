/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#include "roadgeom.hpp"
#include "RoadManager.hpp"
#include "RoadObjectExpansion.hpp"
#include "logger.hpp"

#include <osg/StateSet>
#include <osg/Group>
#include <osg/Switch>
#include <osg/TexEnv>
#include <osg/LOD>
#include <osg/MatrixTransform>
#include <osg/Material>
#include <osgGA/StateSetManipulator>
#include <osg/PolygonOffset>
#include <osgDB/ReadFile>
#include <osgUtil/SmoothingVisitor>
#include <osg/ShapeDrawable>
#include <osg/ComputeBoundsVisitor>
#include <osgUtil/Optimizer>    // to flatten transform nodes
#include <osgUtil/Tessellator>  // to tessellate multiple contours
#include <osgDB/WriteFile>

#include <limits>
#include <unordered_map>
#include <unordered_set>

#include "CommonMini.hpp"

// cppcheck-suppress [unknownMacro]
USE_OSGPLUGIN(osg2)
USE_OSGPLUGIN(jpeg)
USE_SERIALIZER_WRAPPER_LIBRARY(osg)
USE_COMPRESSOR_WRAPPER(ZLibCompressor)

#define GEOM_TOLERANCE            (0.2 - SMALL_NUMBER)  // Minimum distance between two vertices along road s-axis
#define TEXTURE_SCALE             2.0                   // Scale factor for asphalt and grass textures 2.0 means whole texture fits in 2 x 2 m square
#define MAX_GEOM_ERROR_HORIZONTAL 0.25                  // maximum distance from the 3D geometry to the OSI lines, on road surface plane
#define MAX_GEOM_ERROR_VERTICAL   0.1                   // maximum distance from the 3D geometry to the OSI lines, vertical to road surface
#define MAX_GEOM_LENGTH           50                    // maximum length of a road geometry mesh segment
#define MIN_GEOM_LENGTH           0.1                   // minimum length of a road geometry mesh segment, adjust if possible
#define ROADMARK_TEXTURE_SCALE    3.0                   // scale factor for roadmark textures, 3.0 means whole texture fits in 3 x 3 m square

#define POLYGON_OFFSET_SIDEWALK  2.0
#define POLYGON_OFFSET_ROADMARKS 1.0
#define POLYGON_OFFSET_BORDER    -1.0
#define POLYGON_OFFSET_GRASS     -2.0

#define ROADMARK_Z_OFFSET 0.01

#define DEFAULT_LENGTH_FOR_CONTINUOUS_OBJS 10.0
#define LOD_DIST_ROAD_FEATURES             500

const static double      friction_max       = 5.0;
const static double      friction_default   = 1.0;
const static std::string prefix_road        = "road_";
const static std::string prefix_road_object = "road_object_";
const static std::string prefix_tunnel_wall = "tunnel_wall_";
const static std::string prefix_tunnel_roof = "tunnel_roof_";
const static std::string prefix_road_signal = "road_signal_";
const static std::string prefix_roadmark    = "roadmark_";

namespace roadgeom
{
    class FindNamedNode : public osg::NodeVisitor
    {
    public:
        FindNamedNode(const std::string& name) : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), _name(name)
        {
        }

        // This method gets called for every node in the scene graph. Check each node
        // to see if its name matches out target. If so, save the node's address.
        using osg::NodeVisitor::apply;
        void apply(osg::Group& node) override
        {
            if (node.getName().find(_name) != std::string::npos)
            {
                _node = &node;
            }
            else
            {
                // Keep traversing the rest of the scene graph.
                traverse(node);
            }
        }

        osg::Node* getNode()
        {
            return _node.get();
        }

    protected:
        std::string              _name;
        osg::ref_ptr<osg::Group> _node;
    };

    bool compare_s_values(double s0, double s1)
    {
        return (fabs(s1 - s0) < 0.1);
    }

    uint64_t GenerateMaterialKey(double r, double g, double b, double a, uint8_t t, uint8_t f)
    {
        uint8_t r8 = static_cast<uint8_t>(std::max(0.0, std::min(255.0, r * 255.0)));
        uint8_t g8 = static_cast<uint8_t>(std::max(0.0, std::min(255.0, g * 255.0)));
        uint8_t b8 = static_cast<uint8_t>(std::max(0.0, std::min(255.0, b * 255.0)));
        uint8_t a8 = static_cast<uint8_t>(std::max(0.0, std::min(255.0, a * 255.0)));

        // code the color as a 64-bit integer in the format 0xRRGGBBAATTFF (T = texture_type, F = friction)
        return (static_cast<uint64_t>(r8) << 40) | (static_cast<uint64_t>(g8) << 32) | (static_cast<uint64_t>(b8) << 24) |
               (static_cast<uint64_t>(a8) << 16) | (static_cast<uint64_t>(t) << 8) | static_cast<uint64_t>(f);
    }

    osg::Vec4 ODR2OSGColor(roadmanager::RoadMarkColor color)
    {
        const float(&rgb)[3] = SE_Color::Color2RBG(roadmanager::ODRColor2SEColor(color));
        return osg::Vec4(rgb[0], rgb[1], rgb[2], 1.0);
    }

    roadmanager::Outline* BuildExpandedOutlineFromDefinition(const roadmanager::RMObjectDefinition& def,
                                                             const roadmanager::RMExpandedObject&   exp,
                                                             int                                    outline_index,
                                                             id_t                                   road_id)
    {
        if (outline_index < 0 || outline_index >= static_cast<int>(def.outlines.size()))
        {
            return nullptr;
        }

        const roadmanager::RMOutlineDefinition& src_outline    = def.outlines[outline_index];
        roadmanager::Outline*                   outline        = new roadmanager::Outline(src_outline.id, src_outline.fillType, src_outline.closed);
        const double                            z_offset_delta = exp.zOffset - def.zOffset;

        for (const auto& corner_def : src_outline.corners)
        {
            roadmanager::OutlineCorner* corner = nullptr;

            if (corner_def.coordSystem == roadmanager::RMCornerCoordSystem::ROAD)
            {
                // For cornerRoad, preserve the road-frame (ds, dt) offsets from the definition
                // anchor and apply them to the instance anchor. Each instance thereby gets its
                // corners re-evaluated against the actual road geometry at that s position,
                // correctly handling changes in curvature along repeated instances.
                const double ds = corner_def.s.value_or(def.s) - def.s;
                const double dt = corner_def.t.value_or(def.t) - def.t;
                const double dz = corner_def.dz.value_or(0.0) + z_offset_delta;
                const double h  = corner_def.height.value_or(0.0);

                corner = new roadmanager::OutlineCornerRoad(road_id, exp.s + ds, exp.t + dt, dz, h, exp.s, exp.t, exp.hdg);
            }
            else
            {
                const double u      = corner_def.u.value_or(0.0);
                const double v      = corner_def.v.value_or(0.0);
                const double zLocal = corner_def.zLocal.value_or(0.0) + z_offset_delta;
                const double h      = corner_def.height.value_or(0.0);

                corner = new roadmanager::OutlineCornerLocal(road_id, exp.s, exp.t, u, v, zLocal, h, exp.hdg);
            }

            if (corner != nullptr)
            {
                outline->AddCorner(corner);
            }
        }

        return outline;
    }

    std::string LocateRoadObjectModelFile(const std::string& object_name, const std::string& odr_filename, const std::string& exe_path)
    {
        if (object_name.empty())
        {
            return "";
        }

        const std::vector<std::string> search_paths = {DirNameOf(odr_filename) + "/../models", DirNameOf(exe_path) + "/../resources/models"};

        bool        found        = false;
        std::string located_path = LocateFile(object_name, search_paths, "Road object 3D model", found, false);
        if (!found && FileNameExtOf(object_name).empty())
        {
            located_path = LocateFile(object_name + ".osgb", search_paths, "Road object 3D model", found, false);
        }

        return found ? located_path : "";
    }

    void GetNodeDimensions(osg::Node& node, double& dim_x, double& dim_y, double& dim_z)
    {
        osg::ComputeBoundsVisitor cbv;
        node.accept(cbv);

        const osg::BoundingBox& bounding_box = cbv.getBoundingBox();
        dim_x                                = bounding_box._max.x() - bounding_box._min.x();
        dim_y                                = bounding_box._max.y() - bounding_box._min.y();
        dim_z                                = bounding_box._max.z() - bounding_box._min.z();
    }

    std::optional<roadmanager::RoadMarkColor> ParseRoadMarkColorString(std::string color_name)
    {
        std::transform(color_name.begin(), color_name.end(), color_name.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

        if (color_name == "standard")
            return roadmanager::RoadMarkColor::STANDARD;
        if (color_name == "blue")
            return roadmanager::RoadMarkColor::BLUE;
        if (color_name == "green")
            return roadmanager::RoadMarkColor::GREEN;
        if (color_name == "red")
            return roadmanager::RoadMarkColor::RED;
        if (color_name == "white")
            return roadmanager::RoadMarkColor::WHITE;
        if (color_name == "yellow")
            return roadmanager::RoadMarkColor::YELLOW;
        if (color_name == "orange")
            return roadmanager::RoadMarkColor::ORANGE;
        if (color_name == "violet")
            return roadmanager::RoadMarkColor::VIOLET;
        if (color_name == "black")
            return roadmanager::RoadMarkColor::BLACK;

        return std::nullopt;
    }

    osg::Vec4 ResolveMarkingColor(const roadmanager::RMObjectMarkingDefinition& marking, const osg::Vec4& fallback)
    {
        if (!marking.color.has_value())
        {
            return fallback;
        }

        std::optional<roadmanager::RoadMarkColor> parsed = ParseRoadMarkColorString(marking.color.value());
        if (!parsed.has_value())
        {
            return fallback;
        }

        return ODR2OSGColor(parsed.value());
    }

    bool EdgeReferenceMatches(int edge_ref, size_t edge_index, size_t edge_count)
    {
        if (edge_ref >= 1 && edge_ref <= static_cast<int>(edge_count))
        {
            return edge_index == static_cast<size_t>(edge_ref - 1);
        }
        if (edge_ref >= 0 && edge_ref < static_cast<int>(edge_count))
        {
            return edge_index == static_cast<size_t>(edge_ref);
        }

        return false;
    }

    struct MarkingEdge
    {
        osg::Vec3d a;
        osg::Vec3d b;
    };

    bool IntersectLines2D(const osg::Vec3d& a0, const osg::Vec3d& a1, const osg::Vec3d& b0, const osg::Vec3d& b1, osg::Vec3d& out)
    {
        constexpr double kEpsilon = 1e-6;
        const double     d1x      = a1.x() - a0.x();
        const double     d1y      = a1.y() - a0.y();
        const double     d2x      = b1.x() - b0.x();
        const double     d2y      = b1.y() - b0.y();
        const double     det      = d1x * d2y - d1y * d2x;
        if (std::fabs(det) < kEpsilon)
        {
            out.x() = 0.5 * (a1.x() + b0.x());
            out.y() = 0.5 * (a1.y() + b0.y());
            return false;
        }

        const double dx    = b0.x() - a0.x();
        const double dy    = b0.y() - a0.y();
        const double param = (dx * d2y - dy * d2x) / det;
        out.x()            = a0.x() + param * d1x;
        out.y()            = a0.y() + param * d1y;
        return true;
    }

    osg::ref_ptr<osg::Group> RoadGeom::CreateObjectMarkingsGeomFromPolyline(const std::vector<osg::Vec3d>&                points,
                                                                            const roadmanager::RMObjectMarkingDefinition& marking,
                                                                            const osg::Vec4&                              color,
                                                                            bool                                          closed_loop)
    {
        if (points.size() < 2)
        {
            return nullptr;
        }

        const double half_width = 0.5 * std::max(0.01, marking.width.value_or(0.12));

        std::vector<osg::Vec3d> filtered_points;
        filtered_points.reserve(points.size());
        for (const osg::Vec3d& point : points)
        {
            if (filtered_points.empty())
            {
                filtered_points.push_back(point);
                continue;
            }

            const osg::Vec3d& prev = filtered_points.back();
            const double      dx   = point.x() - prev.x();
            const double      dy   = point.y() - prev.y();
            if (std::sqrt(dx * dx + dy * dy) < SMALL_NUMBER)
            {
                continue;
            }

            filtered_points.push_back(point);
        }

        if (closed_loop && filtered_points.size() >= 2)
        {
            const osg::Vec3d& first = filtered_points.front();
            const osg::Vec3d& last  = filtered_points.back();
            const double      dx    = first.x() - last.x();
            const double      dy    = first.y() - last.y();
            if (std::sqrt(dx * dx + dy * dy) < SMALL_NUMBER)
            {
                filtered_points.pop_back();
            }
        }

        if (filtered_points.size() < 2)
        {
            return nullptr;
        }

        const size_t point_count   = filtered_points.size();
        const size_t segment_count = closed_loop ? point_count : (point_count - 1);

        std::vector<osg::Vec3d> segment_normals(segment_count);
        for (size_t i = 0; i < segment_count; ++i)
        {
            const osg::Vec3d& a = filtered_points[i];
            const osg::Vec3d& b = filtered_points[(i + 1) % point_count];
            osg::Vec3d        dir(b.x() - a.x(), b.y() - a.y(), 0.0);
            const double      len = std::sqrt(dir.x() * dir.x() + dir.y() * dir.y());
            if (len < SMALL_NUMBER)
            {
                segment_normals[i].set(0.0, 0.0, 0.0);
            }
            else
            {
                segment_normals[i].set(-dir.y() / len, dir.x() / len, 0.0);
            }
        }

        std::vector<osg::Vec3d> left_points(point_count);
        std::vector<osg::Vec3d> right_points(point_count);

        auto offset_point = [&](const osg::Vec3d& point, const osg::Vec3d& normal, bool left_side) -> osg::Vec3d
        {
            const double sign = left_side ? 1.0 : -1.0;
            return osg::Vec3d(point.x() + sign * normal.x() * half_width, point.y() + sign * normal.y() * half_width, point.z());
        };

        if (closed_loop)
        {
            for (size_t i = 0; i < point_count; ++i)
            {
                const size_t prev_seg = (i + point_count - 1) % point_count;
                const size_t next_seg = i % point_count;

                const osg::Vec3d& prev_point = filtered_points[(i + point_count - 1) % point_count];
                const osg::Vec3d& cur_point  = filtered_points[i];
                const osg::Vec3d& next_point = filtered_points[(i + 1) % point_count];
                const osg::Vec3d& prev_norm  = segment_normals[prev_seg];
                const osg::Vec3d& next_norm  = segment_normals[next_seg];

                osg::Vec3d left_prev_a(prev_point.x() + prev_norm.x() * half_width, prev_point.y() + prev_norm.y() * half_width, cur_point.z());
                osg::Vec3d left_prev_b(cur_point.x() + prev_norm.x() * half_width, cur_point.y() + prev_norm.y() * half_width, cur_point.z());
                osg::Vec3d left_next_a(cur_point.x() + next_norm.x() * half_width, cur_point.y() + next_norm.y() * half_width, cur_point.z());
                osg::Vec3d left_next_b(next_point.x() + next_norm.x() * half_width, next_point.y() + next_norm.y() * half_width, cur_point.z());
                IntersectLines2D(left_prev_a, left_prev_b, left_next_a, left_next_b, left_points[i]);
                left_points[i].z() = cur_point.z();

                osg::Vec3d right_prev_a(prev_point.x() - prev_norm.x() * half_width, prev_point.y() - prev_norm.y() * half_width, cur_point.z());
                osg::Vec3d right_prev_b(cur_point.x() - prev_norm.x() * half_width, cur_point.y() - prev_norm.y() * half_width, cur_point.z());
                osg::Vec3d right_next_a(cur_point.x() - next_norm.x() * half_width, cur_point.y() - next_norm.y() * half_width, cur_point.z());
                osg::Vec3d right_next_b(next_point.x() - next_norm.x() * half_width, next_point.y() - next_norm.y() * half_width, cur_point.z());
                IntersectLines2D(right_prev_a, right_prev_b, right_next_a, right_next_b, right_points[i]);
                right_points[i].z() = cur_point.z();
            }
        }
        else
        {
            left_points.front()  = offset_point(filtered_points.front(), segment_normals.front(), true);
            right_points.front() = offset_point(filtered_points.front(), segment_normals.front(), false);
            left_points.back()   = offset_point(filtered_points.back(), segment_normals.back(), true);
            right_points.back()  = offset_point(filtered_points.back(), segment_normals.back(), false);

            for (size_t i = 1; i + 1 < point_count; ++i)
            {
                const osg::Vec3d& prev_point = filtered_points[i - 1];
                const osg::Vec3d& cur_point  = filtered_points[i];
                const osg::Vec3d& next_point = filtered_points[i + 1];
                const osg::Vec3d& prev_norm  = segment_normals[i - 1];
                const osg::Vec3d& next_norm  = segment_normals[i];

                osg::Vec3d left_prev_a(prev_point.x() + prev_norm.x() * half_width, prev_point.y() + prev_norm.y() * half_width, cur_point.z());
                osg::Vec3d left_prev_b(cur_point.x() + prev_norm.x() * half_width, cur_point.y() + prev_norm.y() * half_width, cur_point.z());
                osg::Vec3d left_next_a(cur_point.x() + next_norm.x() * half_width, cur_point.y() + next_norm.y() * half_width, cur_point.z());
                osg::Vec3d left_next_b(next_point.x() + next_norm.x() * half_width, next_point.y() + next_norm.y() * half_width, cur_point.z());
                IntersectLines2D(left_prev_a, left_prev_b, left_next_a, left_next_b, left_points[i]);
                left_points[i].z() = cur_point.z();

                osg::Vec3d right_prev_a(prev_point.x() - prev_norm.x() * half_width, prev_point.y() - prev_norm.y() * half_width, cur_point.z());
                osg::Vec3d right_prev_b(cur_point.x() - prev_norm.x() * half_width, cur_point.y() - prev_norm.y() * half_width, cur_point.z());
                osg::Vec3d right_next_a(cur_point.x() - next_norm.x() * half_width, cur_point.y() - next_norm.y() * half_width, cur_point.z());
                osg::Vec3d right_next_b(next_point.x() - next_norm.x() * half_width, next_point.y() - next_norm.y() * half_width, cur_point.z());
                IntersectLines2D(right_prev_a, right_prev_b, right_next_a, right_next_b, right_points[i]);
                right_points[i].z() = cur_point.z();
            }
        }

        // Build arc-length parameterization over segments
        std::vector<double> arc_s(segment_count + 1, 0.0);
        for (size_t i = 0; i < segment_count; ++i)
        {
            const osg::Vec3d& a  = filtered_points[i];
            const osg::Vec3d& b  = filtered_points[(i + 1) % point_count];
            const double      dx = b.x() - a.x();
            const double      dy = b.y() - a.y();
            arc_s[i + 1]         = arc_s[i] + std::sqrt(dx * dx + dy * dy);
        }
        double       line_length  = std::max(0.0, marking.lineLength.value_or(marking.length.value_or(0.0)));
        double       space_length = std::max(0.0, marking.spaceLength.value_or(0.0));
        const double start_offset = std::max(0.0, marking.startOffset.value_or(0.0));
        const double stop_offset  = std::max(0.0, marking.stopOffset.value_or(0.0));

        if (space_length < SMALL_NUMBER)
        {
            space_length = 0.0;
            line_length  = std::numeric_limits<double>::infinity();
        }

        const bool dashed = space_length > SMALL_NUMBER && line_length > SMALL_NUMBER;

        // Sample center/left/right offset points at arc-length t
        auto sample = [&](double t) -> std::tuple<osg::Vec3d, osg::Vec3d, osg::Vec3d>
        {
            const double total_len = arc_s.back();
            t                      = std::max(0.0, std::min(total_len, t));
            for (size_t i = 0; i < segment_count; ++i)
            {
                if (t <= arc_s[i + 1] + SMALL_NUMBER)
                {
                    const size_t ni      = (i + 1) % point_count;
                    const double seg_len = arc_s[i + 1] - arc_s[i];
                    const double alpha   = (seg_len > SMALL_NUMBER) ? std::max(0.0, std::min(1.0, (t - arc_s[i]) / seg_len)) : 0.0;
                    auto         lerp3   = [](const osg::Vec3d& a, const osg::Vec3d& b, double f)
                    { return osg::Vec3d(a.x() + f * (b.x() - a.x()), a.y() + f * (b.y() - a.y()), a.z() + f * (b.z() - a.z())); };
                    return {lerp3(filtered_points[i], filtered_points[ni], alpha),
                            lerp3(left_points[i], left_points[ni], alpha),
                            lerp3(right_points[i], right_points[ni], alpha)};
                }
            }
            return {filtered_points.back(), left_points.back(), right_points.back()};
        };

        osg::ref_ptr<osg::Group> group = new osg::Group;

        auto emit_quad = [&](double t0, double t1)
        {
            if (t1 - t0 < SMALL_NUMBER)
            {
                return;
            }

            auto [c0, l0, r0] = sample(t0);
            auto [c1, l1, r1] = sample(t1);

            osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array;
            verts->push_back(osg::Vec3(static_cast<float>(l0.x()), static_cast<float>(l0.y()), static_cast<float>(l0.z())));
            verts->push_back(osg::Vec3(static_cast<float>(r0.x()), static_cast<float>(r0.y()), static_cast<float>(r0.z())));
            verts->push_back(osg::Vec3(static_cast<float>(r1.x()), static_cast<float>(r1.y()), static_cast<float>(r1.z())));
            verts->push_back(osg::Vec3(static_cast<float>(l1.x()), static_cast<float>(l1.y()), static_cast<float>(l1.z())));

            osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
            geom->setVertexArray(verts);
            geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));

            osg::ref_ptr<osg::Vec4Array> color_arr = new osg::Vec4Array;
            color_arr->push_back(color);
            geom->setColorArray(color_arr);
            geom->setColorBinding(osg::Geometry::BIND_OVERALL);
            geom->setDataVariance(osg::Object::STATIC);
            osgUtil::SmoothingVisitor::smooth(*geom, 0.0);

            osg::ref_ptr<osg::Geode> geode = new osg::Geode;
            geode->addDrawable(geom);
            geode->getOrCreateStateSet()->setAttributeAndModes(
                GetOrCreateMaterial("ObjectMark", color, static_cast<uint8_t>(RoadGeom::MaterialType::OBJECT_MARKING)));
            group->addChild(geode);
        };

        auto emit_segment_aligned_dash = [&](size_t seg_index, double local_t0, double local_t1)
        {
            if (local_t1 - local_t0 < SMALL_NUMBER)
            {
                return;
            }

            const size_t      ni      = (seg_index + 1) % point_count;
            const osg::Vec3d& a       = filtered_points[seg_index];
            const osg::Vec3d& b       = filtered_points[ni];
            const double      seg_len = std::sqrt((b.x() - a.x()) * (b.x() - a.x()) + (b.y() - a.y()) * (b.y() - a.y()));
            if (seg_len < SMALL_NUMBER)
            {
                return;
            }

            const double f0 = std::max(0.0, std::min(1.0, local_t0 / seg_len));
            const double f1 = std::max(0.0, std::min(1.0, local_t1 / seg_len));

            const osg::Vec3d p0(a.x() + f0 * (b.x() - a.x()), a.y() + f0 * (b.y() - a.y()), a.z() + f0 * (b.z() - a.z()));
            const osg::Vec3d p1(a.x() + f1 * (b.x() - a.x()), a.y() + f1 * (b.y() - a.y()), a.z() + f1 * (b.z() - a.z()));

            const osg::Vec3d& seg_n = segment_normals[seg_index];
            const osg::Vec3d  n(seg_n.x() * half_width, seg_n.y() * half_width, 0.0);

            osg::Vec3d start_left(p0.x() + n.x(), p0.y() + n.y(), p0.z());
            osg::Vec3d start_right(p0.x() - n.x(), p0.y() - n.y(), p0.z());
            osg::Vec3d end_left(p1.x() + n.x(), p1.y() + n.y(), p1.z());
            osg::Vec3d end_right(p1.x() - n.x(), p1.y() - n.y(), p1.z());

            // Keep dash caps perpendicular everywhere except on actual polyline joins,
            // where we use precomputed miter corner points for seamless stitching.
            const bool touches_seg_start = std::fabs(local_t0) <= SMALL_NUMBER;
            const bool touches_seg_end   = std::fabs(local_t1 - seg_len) <= SMALL_NUMBER;
            const bool covers_full_edge  = touches_seg_start && touches_seg_end;

            auto has_join_at_point = [&](size_t point_index) -> bool
            {
                if (closed_loop)
                {
                    return point_count >= 3;
                }
                return point_index > 0 && point_index + 1 < point_count;
            };

            if (covers_full_edge && touches_seg_start && has_join_at_point(seg_index))
            {
                start_left  = left_points[seg_index];
                start_right = right_points[seg_index];
            }
            if (covers_full_edge && touches_seg_end && has_join_at_point(ni))
            {
                end_left  = left_points[ni];
                end_right = right_points[ni];
            }

            osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array;
            verts->push_back(osg::Vec3(static_cast<float>(start_left.x()), static_cast<float>(start_left.y()), static_cast<float>(start_left.z())));
            verts->push_back(
                osg::Vec3(static_cast<float>(start_right.x()), static_cast<float>(start_right.y()), static_cast<float>(start_right.z())));
            verts->push_back(osg::Vec3(static_cast<float>(end_right.x()), static_cast<float>(end_right.y()), static_cast<float>(end_right.z())));
            verts->push_back(osg::Vec3(static_cast<float>(end_left.x()), static_cast<float>(end_left.y()), static_cast<float>(end_left.z())));

            osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
            geom->setVertexArray(verts);
            geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));

            osg::ref_ptr<osg::Vec4Array> color_arr = new osg::Vec4Array;
            color_arr->push_back(color);
            geom->setColorArray(color_arr);
            geom->setColorBinding(osg::Geometry::BIND_OVERALL);
            geom->setDataVariance(osg::Object::STATIC);
            osgUtil::SmoothingVisitor::smooth(*geom, 0.0);

            osg::ref_ptr<osg::Geode> geode = new osg::Geode;
            geode->addDrawable(geom);
            geode->getOrCreateStateSet()->setAttributeAndModes(
                GetOrCreateMaterial("ObjectMark", color, static_cast<uint8_t>(RoadGeom::MaterialType::OBJECT_MARKING)));
            group->addChild(geode);
        };

        for (size_t i = 0; i < segment_count; ++i)
        {
            const double seg_s   = arc_s[i];
            const double seg_e   = arc_s[i + 1];
            const double seg_len = seg_e - seg_s;
            if (seg_len < SMALL_NUMBER)
            {
                continue;
            }

            const double local_start = std::min(start_offset, seg_len);
            const double local_end   = std::max(local_start, seg_len - stop_offset);
            if (local_end <= local_start + SMALL_NUMBER)
            {
                continue;
            }

            if (dashed)
            {
                double local_t = local_start;
                while (local_t < local_end - SMALL_NUMBER)
                {
                    const double local_t_end = std::min(local_t + line_length, local_end);
                    // For dashed markings, use edge centerline + constant segment normal to keep each dash rectangular.
                    emit_segment_aligned_dash(i, local_t, local_t_end);
                    local_t += line_length + space_length;
                }
            }
            else
            {
                double local_draw_len = local_end - local_start;
                if (line_length > SMALL_NUMBER)
                {
                    local_draw_len = std::min(local_draw_len, line_length);
                }

                const bool use_segment_aligned_solid = line_length > SMALL_NUMBER || start_offset > SMALL_NUMBER || stop_offset > SMALL_NUMBER;
                if (use_segment_aligned_solid)
                {
                    emit_segment_aligned_dash(i, local_start, local_start + local_draw_len);
                }
                else
                {
                    emit_quad(seg_s + local_start, seg_s + local_start + local_draw_len);
                }
            }
        }

        return group->getNumChildren() > 0 ? group : nullptr;
    }

    osg::ref_ptr<osg::Group> RoadGeom::CreateObjectMarkingsGeom(const std::vector<MarkingEdge>&               edges,
                                                                const roadmanager::RMObjectMarkingDefinition& marking,
                                                                const osg::Vec4&                              color)
    {
        if (edges.empty())
        {
            return nullptr;
        }

        const double mark_width   = std::max(0.01, marking.width.value_or(0.12));
        double       line_length  = std::max(0.0, marking.lineLength.value_or(marking.length.value_or(0.0)));
        double       space_length = std::max(0.0, marking.spaceLength.value_or(0.0));
        const double start_offset = std::max(0.0, marking.startOffset.value_or(0.0));
        const double stop_offset  = std::max(0.0, marking.stopOffset.value_or(0.0));

        if (space_length < SMALL_NUMBER)
        {
            space_length = 0.0;
            line_length  = std::numeric_limits<double>::infinity();
        }

        const bool dashed = space_length > SMALL_NUMBER && line_length > SMALL_NUMBER;

        osg::ref_ptr<osg::Group> group = new osg::Group;
        for (const MarkingEdge& edge : edges)
        {
            osg::Vec3d   dir(edge.b.x() - edge.a.x(), edge.b.y() - edge.a.y(), 0.0);
            const double edge_len = std::sqrt(dir.x() * dir.x() + dir.y() * dir.y());
            if (edge_len < SMALL_NUMBER)
            {
                continue;
            }

            const double eff_start = start_offset;
            const double eff_end   = std::max(eff_start, edge_len - stop_offset);
            if (eff_end <= eff_start + SMALL_NUMBER)
            {
                continue;
            }

            osg::Vec3d n(-dir.y() / edge_len, dir.x() / edge_len, 0.0);
            n *= (0.5 * mark_width);

            auto point_at = [&](double t) -> osg::Vec3d
            {
                const double f = t / edge_len;
                return osg::Vec3d(edge.a.x() + dir.x() * f, edge.a.y() + dir.y() * f, edge.a.z() + (edge.b.z() - edge.a.z()) * f);
            };

            double t = eff_start;
            while (t < eff_end - SMALL_NUMBER)
            {
                const double t_end = dashed ? std::min(t + line_length, eff_end) : eff_end;
                if (t_end - t < SMALL_NUMBER)
                {
                    break;
                }

                const osg::Vec3d p0 = point_at(t);
                const osg::Vec3d p1 = point_at(t_end);

                osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array;
                verts->push_back(osg::Vec3(static_cast<float>(p0.x() + n.x()), static_cast<float>(p0.y() + n.y()), static_cast<float>(p0.z())));
                verts->push_back(osg::Vec3(static_cast<float>(p0.x() - n.x()), static_cast<float>(p0.y() - n.y()), static_cast<float>(p0.z())));
                verts->push_back(osg::Vec3(static_cast<float>(p1.x() - n.x()), static_cast<float>(p1.y() - n.y()), static_cast<float>(p1.z())));
                verts->push_back(osg::Vec3(static_cast<float>(p1.x() + n.x()), static_cast<float>(p1.y() + n.y()), static_cast<float>(p1.z())));

                osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
                geom->setVertexArray(verts);
                geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));

                osg::ref_ptr<osg::Vec4Array> color_arr = new osg::Vec4Array;
                color_arr->push_back(color);
                geom->setColorArray(color_arr);
                geom->setColorBinding(osg::Geometry::BIND_OVERALL);
                geom->setDataVariance(osg::Object::STATIC);
                osgUtil::SmoothingVisitor::smooth(*geom, 0.0);

                osg::ref_ptr<osg::Geode> geode = new osg::Geode;
                geode->addDrawable(geom);
                geode->getOrCreateStateSet()->setAttributeAndModes(
                    GetOrCreateMaterial("ObjectMark", color, static_cast<uint8_t>(RoadGeom::MaterialType::OBJECT_MARKING)));
                group->addChild(geode);

                if (!dashed)
                {
                    break;  // solid: single quad per edge
                }
                t += line_length + space_length;
            }
        }

        return group->getNumChildren() > 0 ? group : nullptr;
    }

    osg::ref_ptr<osg::Material> RoadGeom::GetOrCreateMaterial(const std::string& basename,
                                                              osg::Vec4          color,
                                                              uint8_t            texture_type,
                                                              uint8_t            has_friction)
    {
        uint64_t key = GenerateMaterialKey(color[0], color[1], color[2], color[3], texture_type, has_friction);

        if (std_materials_.find(key) != std_materials_.end())
        {
            return std_materials_[key];
        }
        else
        {
            // create and store new materiak
            osg::ref_ptr<osg::Material> material = new osg::Material;
            material->setName(fmt::format("Material_{}_{}_0x{:02x}{:02x}{:02x}{:02x}_{:02x}_{:02x}",
                                          number_of_materials,
                                          basename,
                                          static_cast<unsigned int>(color[0] * 255),
                                          static_cast<unsigned int>(color[1] * 255),
                                          static_cast<unsigned int>(color[2] * 255),
                                          static_cast<unsigned int>(color[3] * 255),
                                          static_cast<unsigned int>(texture_type),
                                          static_cast<unsigned int>(has_friction)));

            LOG_DEBUG("Creating material {}", material->getName());
            material->setDiffuse(osg::Material::FRONT_AND_BACK, color);
            material->setAmbient(osg::Material::FRONT_AND_BACK, color);
            material->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
            material->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
            material->setShininess(osg::Material::FRONT_AND_BACK, 1.0f);
            if (static_cast<uint8_t>(MaterialType::ASPHALT) == texture_type)
            {
                material->setUserValue("friction", lane_friction_);  // default friction value
            }

            // store material for reuse
            std_materials_[key] = material;

            // keep track of the number of created materials
            number_of_materials++;

            return material;
        }
    }

    osg::ref_ptr<osg::Texture2D> RoadGeom::ReadTexture(std::string filename, bool log_missing_file)
    {
        osg::ref_ptr<osg::Texture2D> tex = 0;
        osg::ref_ptr<osg::Image>     img = 0;
        bool                         found;
        std::string                  file_path = LocateFile(filename,
                                                            {DirNameOf(odrManager_->GetOpenDriveFilename()) + "/../models", exe_dir_ + "/../resources/models"},
                                           "Texture file",
                                           found,
                                           log_missing_file);

        if (found)
        {
            img = osgDB::readImageFile(file_path);

            if (img)
            {
                tex = new osg::Texture2D(img.get());
                tex->setUnRefImageDataAfterApply(true);
                tex->setWrap(osg::Texture2D::WrapParameter::WRAP_S, osg::Texture2D::WrapMode::REPEAT);
                tex->setWrap(osg::Texture2D::WrapParameter::WRAP_T, osg::Texture2D::WrapMode::REPEAT);
            }
            else
            {
                LOG_WARN("Failed to load texture file: {}", filename);
            }
        }

        return tex;
    }

    osg::Vec4 RoadGeom::GetFrictionColor(const double friction)
    {
        osg::Vec4 new_color = color_asphalt_->at(0);
        if (friction < friction_default - SMALL_NUMBER)  // low friction, make it blueish
        {
            double factor = (1.0 - friction) / friction_default;
            new_color[0] -= 0.75 * factor;
            new_color[1] -= 0.75 * factor;
            new_color[2] += factor;
        }
        else if (friction > friction_default + SMALL_NUMBER)  // high friction, make it redish
        {
            double factor = (MIN(friction, friction_max) - friction_default) / (friction_max - friction_default);
            new_color[0] += factor;
            new_color[1] -= 0.75 * factor;
            new_color[2] -= 0.75 * factor;
        }

        new_color[0] = CLAMP(0.0, 1.0, new_color[0]);
        new_color[1] = CLAMP(0.0, 1.0, new_color[1]);
        new_color[2] = CLAMP(0.0, 1.0, new_color[2]);

        return new_color;
    }

    void RoadGeom::AddRoadMarkGeom(osg::ref_ptr<osg::Vec3Array>        vertices,
                                   osg::ref_ptr<osg::DrawElementsUInt> indices,
                                   osg::Group*                         rm_group,
                                   const roadmanager::LaneRoadMark&    road_mark,
                                   double                              fade)
    {
        osg::ref_ptr<osg::Vec4Array> color_array = new osg::Vec4Array;
        color_array->push_back(ODR2OSGColor(road_mark.GetColor()));
        color_array->back()[3] = 1.0 - fade;  // Set alpha value

        // Finally create and add geometry
        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
        geom->setUseDisplayList(true);
        geom->setVertexArray(vertices.get());
        geom->addPrimitiveSet(indices.get());

        // Use PolygonOffset feature to avoid z-fighting with road surface
        geom->getOrCreateStateSet()->setAttributeAndModes(new osg::PolygonOffset(-POLYGON_OFFSET_ROADMARKS, -SIGN(POLYGON_OFFSET_ROADMARKS)));

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;

        // create material with unique name
        osg::ref_ptr<osg::Material> materialRoadmark_ =
            GetOrCreateMaterial("RoadMark_" + roadmanager::LaneRoadMark::RoadMarkColor2Str(road_mark.GetColor()),
                                color_array->back(),
                                static_cast<uint8_t>(RoadGeom::MaterialType::ROADMARK));

        // also embed color in geometry, e.g. for post processing in full stack simulations
        geom->setColorArray(color_array.get());
        geom->setColorBinding(osg::Geometry::BIND_OVERALL);

        if (fade > SMALL_NUMBER)
        {
            geom->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
            geom->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
        }

        geode->getOrCreateStateSet()->setAttributeAndModes(materialRoadmark_.get());

        if (!SE_Env::Inst().GetOptions().GetOptionSet("generate_without_textures"))
        {
            // set texture coordinates anyway, for potential post processing
            osg::ref_ptr<osg::Vec2Array> texcoords   = new osg::Vec2Array(indices.get()->getNumIndices());
            double                       rm_texscale = 1.0 / ROADMARK_TEXTURE_SCALE;
            for (unsigned int i = 0; i < indices.get()->getNumIndices(); i++)
            {
                (*texcoords)[i].set(osg::Vec2(static_cast<float>(rm_texscale * ((*vertices)[(*indices)[i]][0])),
                                              static_cast<float>(rm_texscale * ((*vertices)[(*indices)[i]][1]))));
            }
            geom->setTexCoordArray(0, texcoords.get());

            if (texture_map_[MaterialType::ROADMARK])
            {
                // set color to white for texture mapping. but keep alpha
                color_array->back()[0] = 1.0f;
                color_array->back()[1] = 1.0f;
                color_array->back()[2] = 1.0f;
                geode->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture_map_[MaterialType::ROADMARK].get());
            }
        }

        osgUtil::SmoothingVisitor::smooth(*geom, 0.0);
        geode->addDrawable(geom.get());
        SetNodeName(*geode, prefix_roadmark, rm_group->getNumChildren(), road_mark.Type2Str());
        rm_group->addChild(geode);
    }

    int RoadGeom::AddRoadMarks(roadmanager::Lane* lane, osg::Group* rm_group, const osg::Vec3d& origin)
    {
        for (unsigned int i = 0; i < lane->GetNumberOfRoadMarks(); i++)
        {
            roadmanager::LaneRoadMark* lane_roadmark = lane->GetLaneRoadMarkByIdx(static_cast<int>(i));

            if (lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::NONE_TYPE)
            {
                continue;
            }

            for (unsigned int m = 0; m < lane_roadmark->GetNumberOfRoadMarkTypes(); m++)
            {
                roadmanager::LaneRoadMarkType* lane_roadmarktype = lane_roadmark->GetLaneRoadMarkTypeByIdx(m);

                for (unsigned int n = 0; n < lane_roadmarktype->GetNumberOfRoadMarkTypeLines(); n++)
                {
                    roadmanager::LaneRoadMarkTypeLine* lane_roadmarktypeline = lane_roadmarktype->GetLaneRoadMarkTypeLineByIdx(n);
                    roadmanager::OSIPoints*            curr_osi_rm           = lane_roadmarktypeline->GetOSIPoints();

                    if (lane_roadmark->GetType() == roadmanager::LaneRoadMark::RoadMarkType::BOTTS_DOTS)
                    {
                        for (unsigned int q = 0; q < curr_osi_rm->GetPoints().size(); q++)
                        {
                            const double                    botts_dot_size = 0.15;
                            static osg::ref_ptr<osg::Geode> dot            = 0;

                            if (dot == 0)
                            {
                                osg::ref_ptr<osg::TessellationHints> th = new osg::TessellationHints();
                                th->setDetailRatio(0.3f);
                                osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable(
                                    new osg::Cylinder(osg::Vec3(0.0, 0.0, ROADMARK_Z_OFFSET), botts_dot_size, 0.3 * botts_dot_size),
                                    th);
                                shape->setColor(ODR2OSGColor(lane_roadmark->GetColor()));
                                dot = new osg::Geode;
                                dot->addDrawable(shape);
                            }

                            roadmanager::PointStruct osi_point0 = curr_osi_rm->GetPoint(q);

                            osg::ref_ptr<osg::PositionAttitudeTransform> tx = new osg::PositionAttitudeTransform;
                            tx->setPosition(osg::Vec3(static_cast<float>(osi_point0.x - origin[0]),
                                                      static_cast<float>(osi_point0.y - origin[1]),
                                                      static_cast<float>(osi_point0.z)));
                            tx->addChild(dot);
                            SetNodeName(*tx, prefix_roadmark, rm_group->getNumChildren(), lane_roadmark->Type2Str());
                            rm_group->addChild(tx);
                        }
                    }
                    else
                    {
                        std::vector<roadmanager::PointStruct> osi_points = curr_osi_rm->GetPoints();

                        if (osi_points.size() < 2)
                        {
                            // No line - skip
                            continue;
                        }

                        double l0p0l[3] = {0.0, 0.0, 0.0};  // previous line, startpoint, left side
                        double l0p0r[3] = {0.0, 0.0, 0.0};  // previous line, startpoint, right side
                        double l0p1l[3] = {0.0, 0.0, 0.0};  // previous line, endpoint, left side
                        double l0p1r[3] = {0.0, 0.0, 0.0};  // previous line, endpoint, right side
                        double l1p0l[3] = {0.0, 0.0, 0.0};  // current line, startpoint, left side
                        double l1p0r[3] = {0.0, 0.0, 0.0};  // current line, startpoint, right side
                        double l1p1l[3] = {0.0, 0.0, 0.0};  // current line, endpoint, left side
                        double l1p1r[3] = {0.0, 0.0, 0.0};  // current line, endpoint, right side

                        osg::ref_ptr<osg::Vec3Array>        vertices;
                        osg::ref_ptr<osg::DrawElementsUInt> indices;

                        unsigned int startpoint = 0;

                        for (unsigned int q = 0; q < static_cast<unsigned int>(osi_points.size()); q++)
                        {
                            // Find offset points of solid roadmark at each OSI point
                            // each line has two points, beginning and end
                            // from each point one left and one right point will be calculated based on width of marking
                            // l1 is current line, l0 is previous

                            if (q == startpoint)
                            {
                                vertices = new osg::Vec3Array();
                                indices  = new osg::DrawElementsUInt(GL_TRIANGLE_STRIP);
                            }

                            if (q < osi_points.size() - 1)
                            {
                                // calculate roadmark vertices, y offset based on width, z offset based on ROADMARK_Z_OFFSET, and
                                // rotation based on heading, pitch and roll of the OSI point

                                const double w    = lane_roadmarktypeline->GetWidth() / 2;
                                double       v[3] = {};

                                // right starting point
                                RotateVec3d(osi_points[q].h, osi_points[q].p, osi_points[q].r, 0.0, w, ROADMARK_Z_OFFSET, v[0], v[1], v[2]);
                                l1p0l[0] = osi_points[q].x + v[0] - origin[0];
                                l1p0l[1] = osi_points[q].y + v[1] - origin[1];
                                l1p0l[2] = osi_points[q].z + v[2];

                                // left end point
                                RotateVec3d(osi_points[q + 1].h,
                                            osi_points[q + 1].p,
                                            osi_points[q + 1].r,
                                            0.0,
                                            w,
                                            ROADMARK_Z_OFFSET,
                                            v[0],
                                            v[1],
                                            v[2]);
                                l1p1l[0] = osi_points[q + 1].x + v[0] - origin[0];
                                l1p1l[1] = osi_points[q + 1].y + v[1] - origin[1];
                                l1p1l[2] = osi_points[q + 1].z + v[2];

                                // left starting point
                                RotateVec3d(osi_points[q].h, osi_points[q].p, osi_points[q].r, 0.0, -w, ROADMARK_Z_OFFSET, v[0], v[1], v[2]);
                                l1p0r[0] = osi_points[q].x + v[0] - origin[0];
                                l1p0r[1] = osi_points[q].y + v[1] - origin[1];
                                l1p0r[2] = osi_points[q].z + v[2];

                                // right end point
                                RotateVec3d(osi_points[q + 1].h,
                                            osi_points[q + 1].p,
                                            osi_points[q + 1].r,
                                            0.0,
                                            -w,
                                            ROADMARK_Z_OFFSET,
                                            v[0],
                                            v[1],
                                            v[2]);
                                l1p1r[0] = osi_points[q + 1].x + v[0] - origin[0];
                                l1p1r[1] = osi_points[q + 1].y + v[1] - origin[1];
                                l1p1r[2] = osi_points[q + 1].z + v[2];
                            }
                            else if (!osi_points[q].endpoint)
                            {
                                LOG_ERROR("Unexpected last point without endpoint q {}", q);
                            }

                            if (q == startpoint)
                            {
                                // First point in a line sequence, no adjustment needed
                                (*vertices).push_back(
                                    osg::Vec3(static_cast<float>(l1p0l[0]), static_cast<float>(l1p0l[1]), static_cast<float>(l1p0l[2])));
                                (*vertices).push_back(
                                    osg::Vec3(static_cast<float>(l1p0r[0]), static_cast<float>(l1p0r[1]), static_cast<float>(l1p0r[2])));
                            }
                            else if (osi_points[q].endpoint)
                            {
                                // Last point of a line sequence, no adjustment needed
                                double* left  = (q < osi_points.size() - 1) ? l1p0l : l1p1l;
                                double* right = (q < osi_points.size() - 1) ? l1p0r : l1p1r;
                                (*vertices).push_back(
                                    osg::Vec3(static_cast<float>(left[0]), static_cast<float>(left[1]), static_cast<float>(left[2])));
                                (*vertices).push_back(
                                    osg::Vec3(static_cast<float>(right[0]), static_cast<float>(right[1]), static_cast<float>(right[2])));
                            }
                            else
                            {
                                // Find intersection of non parallel lines
                                double isect[2];

                                // left side
                                if (GetIntersectionOfTwoLineSegments(l0p0l[0],
                                                                     l0p0l[1],
                                                                     l0p1l[0],
                                                                     l0p1l[1],
                                                                     l1p0l[0],
                                                                     l1p0l[1],
                                                                     l1p1l[0],
                                                                     l1p1l[1],
                                                                     isect[0],
                                                                     isect[1]) == 0)
                                {
                                    (*vertices).push_back(
                                        osg::Vec3(static_cast<float>(isect[0]), static_cast<float>(isect[1]), static_cast<float>(l0p1l[2])));
                                }
                                else
                                {
                                    // lines parallel, no adjustment needed
                                    (*vertices).push_back(
                                        osg::Vec3(static_cast<float>(l1p0l[0]), static_cast<float>(l1p0l[1]), static_cast<float>(l1p0l[2])));
                                }

                                // right side
                                if (GetIntersectionOfTwoLineSegments(l0p0r[0],
                                                                     l0p0r[1],
                                                                     l0p1r[0],
                                                                     l0p1r[1],
                                                                     l1p0r[0],
                                                                     l1p0r[1],
                                                                     l1p1r[0],
                                                                     l1p1r[1],
                                                                     isect[0],
                                                                     isect[1]) == 0)
                                {
                                    (*vertices).push_back(
                                        osg::Vec3(static_cast<float>(isect[0]), static_cast<float>(isect[1]), static_cast<float>(l0p1r[2])));
                                }
                                else
                                {
                                    // lines parallel, no adjustment needed
                                    (*vertices).push_back(
                                        osg::Vec3(static_cast<float>(l1p0r[0]), static_cast<float>(l1p0r[1]), static_cast<float>(l1p0r[2])));
                                }
                            }

                            if (q < osi_points.size() - 1)
                            {
                                // Shift points one step forward
                                memcpy(l0p0l, l1p0l, sizeof(l0p0l));
                                memcpy(l0p0r, l1p0r, sizeof(l0p0r));
                                memcpy(l0p1l, l1p1l, sizeof(l0p0l));
                                memcpy(l0p1r, l1p1r, sizeof(l0p0r));
                            }

                            // Set indices
                            (*indices).push_back(static_cast<unsigned int>(2 * (q - startpoint)));
                            (*indices).push_back(static_cast<unsigned int>(2 * (q - startpoint) + 1));

                            if (osi_points[q].endpoint)
                            {
                                // create and add OSG geometry for the line sequence
                                AddRoadMarkGeom(vertices, indices, rm_group, *lane_roadmark, lane_roadmark->GetFade());
                                startpoint = q + 1;
                            }
                        }
                    }
                }
            }
        }

        return 0;
    }

    RoadGeom::RoadGeom(roadmanager::OpenDrive* odr,
                       osg::Node*              environment,
                       osg::Vec3d              origin,
                       bool                    generate_road_surface,
                       bool                    generate_road_objects,
                       std::string             exe_path,
                       bool                    optimize)
        : environment_(environment),
          optimize_(optimize)
    {
        if (!generate_road_surface && !generate_road_objects)
        {
            return;
        }

        exe_dir_    = DirNameOf(exe_path);
        odrManager_ = odr;
        root_       = new osg::Group;
        root_->setName("esmini_generated_road_model");

        if (generate_road_surface)
        {
            LOG_INFO("Generating a simplistic 3D model of the road network");

            osg::ref_ptr<osg::Group> r_group_ = new osg::Group;
            r_group_->setName("roads");
            osg::ref_ptr<osg::Group> rm_group_ = new osg::Group;
            rm_group_->setName("roadmarks");
            root_->addChild(rm_group_);
            root_->addChild(r_group_);

            if (!SE_Env::Inst().GetOptions().GetOptionSet("generate_without_textures"))
            {
                texture_map_[MaterialType::ASPHALT]  = ReadTexture("asphalt.jpg");
                texture_map_[MaterialType::GRASS]    = ReadTexture("grass.jpg");
                texture_map_[MaterialType::ROADMARK] = ReadTexture("roadmark.jpg", false);  // optional texture, not part of esmini
            }

            osg::ref_ptr<osg::Vec4Array> color_concrete     = new osg::Vec4Array;
            osg::ref_ptr<osg::Vec4Array> color_border_inner = new osg::Vec4Array;
            osg::ref_ptr<osg::Vec4Array> color_grass        = new osg::Vec4Array;

            if (auto it = texture_map_.find(MaterialType::ASPHALT); it != texture_map_.end() && it->second != nullptr)  // We have a texture
            {
                color_asphalt_->push_back(osg::Vec4(1.f, 1.f, 1.f, 1.0f));
            }
            else  // No texture, use default color
            {
                color_asphalt_->push_back(osg::Vec4(0.3f, 0.3f, 0.3f, 1.0f));
            }

            if (auto it = texture_map_.find(MaterialType::GRASS); it != texture_map_.end() && it->second != nullptr)  // We have a texture
            {
                color_grass->push_back(osg::Vec4(1.f, 1.f, 1.f, 1.0f));
            }
            else  // No texture, use default color
            {
                color_grass->push_back(osg::Vec4(0.25f, 0.5f, 0.35f, 1.0f));
            }

            color_concrete->push_back(osg::Vec4(0.61f, 0.61f, 0.61f, 1.0f));
            color_border_inner->push_back(osg::Vec4(0.45f, 0.45f, 0.45f, 1.0f));

            // algorithm:
            // for each road and lane section, consider all lanes except center lane (id 0):
            // - establish first point of each lane at s value = 0, set to current
            // - loop until reaching end of lane section:
            //   - for each lane:
            //     - find next OSI point along the lane, from the current section s value
            //       - register s value as lane current and as candidate section current
            //   - sort the list of section s value candidates
            //   - for each candidate:
            //     - for each lane:
            //       - calculate point at candidate s value
            //       - measure error from tangent of current section s-value point
            //       - if error is too large:
            //         - break
            //       - else, if error is OK:
            //         - register as new current section s value
            //     - if no OK point was found, pick the first candiate (lowest s-value)
            //   - establish points for all lanes at this s-value

            for (size_t i = 0; i < static_cast<unsigned int>(odr->GetNumOfRoads()); i++)
            {
                roadmanager::Road* road = odr->GetRoadByIdx(static_cast<int>(i));

                for (size_t j = 0; j < static_cast<unsigned int>(road->GetNumberOfLaneSections()); j++)
                {
                    roadmanager::LaneSection* lsec = road->GetLaneSectionByIdx(static_cast<int>(j));
                    if (lsec->GetNumberOfLanes() < 2)
                    {
                        // need at least reference lane plus another lane to form a road geometry
                        continue;
                    }

                    std::vector<int> all_lane_ids;
                    all_lane_ids.reserve(lsec->GetNumberOfLanes());

                    std::vector<int> lane_ids;  // all physical lane ids, except center lane which has no area
                    lane_ids.reserve(lsec->GetNumberOfLanes() - 1);
                    std::unordered_map<int, int> lane_idxs;  // store indexes for all lanes, except center lane (id 0)

                    // First make sure there are OSI points of the center lane
                    roadmanager::Lane* lane = lsec->GetLaneById(0);
                    if (lane->GetOSIPoints() == 0)
                    {
                        LOG_ERROR("Missing OSI points of centerlane road {} section {}", road->GetId(), j);
                        throw std::runtime_error("Missing OSI points");
                    }

                    // create a 2d list of positions for vertices, nr_of_s-values x nr_of_lanes
                    typedef struct
                    {
                        double x;
                        double y;
                        double z;
                        double h;
                        double p;
                        double r;
                        double slope;
                        double s;
                    } GeomPoint;

                    typedef struct
                    {
                        int    geom_point_index;
                        double friction;
                    } GeomStrip;  // could be multiple of these per lane

                    struct GeomCacheEntry
                    {
                        GeomPoint point;
                        double    friction = 1.0;
                    };

                    struct CandidatePos
                    {
                        double s;
                        double x;
                        double y;
                        double z;
                    };

                    std::vector<std::vector<std::vector<GeomPoint>>> geom_points_list;                          // two lists of points per lane
                    std::vector<std::vector<GeomStrip>>              geom_strips_list;                          // one list of strips info per lane
                    std::vector<int>                                 lane_osi_index(lsec->GetNumberOfLanes());  // current osi point per lane
                    std::vector<GeomCacheEntry>                      geom_cache(lsec->GetNumberOfLanes());      // one cache entry per lane
                    std::vector<CandidatePos>                        candidates_pos(lsec->GetNumberOfLanes());  // candidates for next current s-value
                    double                                           section_current_s = lsec->GetS();

                    roadmanager::Position pos;  // used for calculating points along the road
                    pos.SetSnapLaneTypes(-1);

                    // First populate s values of the material elements
                    //   - for each material a new friction segment is to be added
                    //   - loop over material friction segments, insert new vertices if needed
                    std::vector<double> friction_s_list;
                    for (size_t k = 0; k < static_cast<unsigned int>(lsec->GetNumberOfLanes()); k++)
                    {
                        lane = lsec->GetLaneByIdx(k);

                        all_lane_ids.push_back(lane->GetId());

                        if (lane->GetId() == 0)
                        {
                            // skip center lane
                            continue;
                        }

                        lane_idxs[lane->GetId()] = lane_idxs.size();
                        lane_ids.push_back(lane->GetId());

                        for (size_t l = 0; l < lane->GetNumberOfMaterials(); l++)
                        {
                            friction_s_list.push_back(lsec->GetS() + lane->GetMaterialByIdx(l)->s_offset);
                        }

                        // add lane height entries to the s-value list
                        for (size_t l = 0; l < lane->GetNumberOfHeights(); l++)
                        {
                            friction_s_list.push_back(lsec->GetS() + lane->GetHeightByIdx(l)->s_offset);
                        }
                    }

                    // sort friction s-values and remove duplicates
                    std::sort(friction_s_list.begin(), friction_s_list.end());
                    friction_s_list.erase(std::unique(friction_s_list.begin(), friction_s_list.end(), compare_s_values), friction_s_list.end());

                    // collect a list of s values where vertices are needed, considering all lanes
                    int                   friction_s_list_index = friction_s_list.size() > 0 ? 1 : -1;
                    bool                  done_section          = false;
                    roadmanager::Position pos2;

                    for (int counter = 0; !done_section; counter++)
                    {
                        if (counter == 0)
                        {
                            // First add s = start of lane section, to set start of mesh
                            done_section = false;
                            for (size_t k = 0; k < static_cast<unsigned int>(lsec->GetNumberOfLanes() - 1); k++)
                            {
                                lane_osi_index[k]   = 0;
                                candidates_pos[k].s = lsec->GetS();
                            }
                        }
                        else
                        {
                            // for each lane, find next s-value in and register it as candidate section current s-value
                            std::vector<double> s_list_sorted(all_lane_ids.size());
                            for (unsigned int k = 0; k < all_lane_ids.size(); k++)
                            {
                                lane                                            = lsec->GetLaneById(all_lane_ids[k]);
                                std::vector<roadmanager::PointStruct> osiPoints = lane->GetOSIPoints()->GetPoints();

                                for (size_t l = lane_osi_index[k]; l < osiPoints.size(); l++)
                                {
                                    if (l == osiPoints.size() - 1 || osiPoints[l].s > section_current_s + SMALL_NUMBER)
                                    {
                                        lane_osi_index[k]   = l;
                                        candidates_pos[k].s = osiPoints[l].s;
                                        s_list_sorted[k]    = osiPoints[l].s;

                                        // generate point at osi index s-value
                                        pos2.SetLaneBoundaryPos(road->GetId(), lane->GetId(), candidates_pos[k].s);
                                        candidates_pos[k].x = pos2.GetX();
                                        candidates_pos[k].y = pos2.GetY();
                                        candidates_pos[k].z = pos2.GetZ();

                                        break;
                                    }
                                }
                            }

                            // sort candidates
                            std::sort(s_list_sorted.begin(), s_list_sorted.end());
                            s_list_sorted.erase(std::unique(s_list_sorted.begin(), s_list_sorted.end(), compare_s_values), s_list_sorted.end());

                            // find highest s-value not exceeding the tolerated error, over all lanes
                            size_t k = 0;
                            for (; k < s_list_sorted.size(); k++)
                            {
                                size_t l = 0;
                                for (; l < all_lane_ids.size(); l++)
                                {
                                    lane = lsec->GetLaneById(all_lane_ids[l]);

                                    // generate point at pivot s-value
                                    pos.SetLaneBoundaryPos(road->GetId(), lane->GetId(), s_list_sorted[k]);

                                    // create a delta vector from real pos to cache/pivot point
                                    double diff[3], diff_tx[3];
                                    diff[0] = pos.GetX() - geom_cache[l].point.x;
                                    diff[1] = pos.GetY() - geom_cache[l].point.y;
                                    diff[2] = pos.GetZ() - geom_cache[l].point.z;

                                    // transform delta vector to road local coordinates, to get longitudinal and lateral error
                                    InverseRotateVec3d(geom_cache[l].point.h,
                                                       geom_cache[l].point.p,
                                                       geom_cache[l].point.r,
                                                       diff[0],
                                                       diff[1],
                                                       diff[2],
                                                       diff_tx[0],
                                                       diff_tx[1],
                                                       diff_tx[2]);

                                    double error_horizontal = abs(diff_tx[1]);
                                    double error_vertical   = abs(diff_tx[2]);

                                    if (error_horizontal > MAX_GEOM_ERROR_HORIZONTAL || error_vertical > MAX_GEOM_ERROR_VERTICAL)
                                    {
                                        break;
                                    }
                                }

                                if (l == all_lane_ids.size())
                                {
                                    // no error, register preliminary s value - if larger than current
                                    if (s_list_sorted[k] > section_current_s)
                                    {
                                        section_current_s = s_list_sorted[k];
                                    }
                                }
                                else
                                {
                                    // break occured, error too large, stop searching
                                    if (k == 0)
                                    {
                                        // can't skip first point
                                        section_current_s = s_list_sorted[k];
                                    }
                                    break;
                                }

                                // we have s-value of a OSI point, check if there is a new friction value before that
                                // also check for maximum length
                                double s_next_friction     = (friction_s_list_index > -1 && friction_s_list_index < friction_s_list.size())
                                                                 ? friction_s_list[friction_s_list_index]
                                                                 : lsec->GetS() + lsec->GetLength();
                                double s_next_geom_max_len = geom_cache[k].point.s + MAX_GEOM_LENGTH;

                                if (s_next_friction < section_current_s &&
                                    s_next_friction < s_next_geom_max_len + MIN_GEOM_LENGTH)  // add min geom len to avoid mini patches
                                {
                                    section_current_s = s_next_friction;
                                    friction_s_list_index++;
                                    break;
                                }
                                else if (s_next_geom_max_len < section_current_s - SMALL_NUMBER &&
                                         s_next_geom_max_len + MIN_GEOM_LENGTH < s_next_friction)  // add min geom len to avoid mini patches
                                {
                                    section_current_s = s_next_geom_max_len;
                                    break;
                                }
                            }
                        }

                        if (section_current_s > lsec->GetS() + lsec->GetLength() - SMALL_NUMBER)
                        {
                            done_section = true;
                        }

                        // s-value for next point established, create vertices for each lane
                        unsigned int geom_idx = 0;
                        for (size_t k = 0; k < all_lane_ids.size(); k++)
                        {
                            int lane_id  = all_lane_ids[k];
                            int lane_idx = lane_idxs[lane_id];

                            if (counter == 0 && lane_id != 0)
                            {
                                // add geometry and strip list for the lane
                                geom_points_list.push_back({{}, {}});
                                geom_strips_list.push_back({});
                            }

                            roadmanager::Lane::Material* mat = nullptr;
                            lane                             = lsec->GetLaneById(lane_id);
                            mat                              = lane->GetMaterialByS(section_current_s - lsec->GetS());
                            double friction                  = mat != nullptr ? mat->friction : FRICTION_DEFAULT;

                            if (lane_id != 0 && counter == 0 || !NEAR_NUMBERS(friction, geom_cache[k].friction))
                            {
                                // create initial strip or strip with new friction value
                                geom_strips_list[lane_idx].push_back({static_cast<int>(geom_points_list[lane_idx][0].size()), friction});
                            }

                            for (auto side : {0, 1})  // left, right
                            {
                                // Determine if we are on the "outer" edge relative to the road center
                                bool is_boundary = (lane_id >= 0) ? (side == 0) : (side == 1);

                                if (!is_boundary)
                                {
                                    double lane_width = lsec->GetWidth(section_current_s, lane_id);
                                    pos.SetLanePos(road->GetId(),
                                                   lane_id,
                                                   section_current_s,
                                                   -1 * SIGN(lane_id) * lane_width / 2);  // offset from lane center (half lane) to inner boundary
                                }
                                else
                                {
                                    pos.SetLaneBoundaryPos(road->GetId(), lane_id, section_current_s);
                                }

                                GeomPoint gp =
                                    {pos.GetX(), pos.GetY(), pos.GetZ(), pos.GetH(), pos.GetP(), pos.GetR(), pos.GetZRoadPrim(), pos.GetS()};

                                if (lane_id != 0)
                                {
                                    geom_points_list[geom_idx][side].push_back(gp);
                                }

                                if (is_boundary)
                                {
                                    geom_cache[k] = {gp, friction};
                                }

                                if (lane_id == 0)
                                {
                                    break;
                                }
                            }
                            if (lane_id != 0)
                            {
                                geom_idx++;
                            }
                        }
                    }

                    // Then create actual vertices and triangle strips for the lane section
                    // Each strip is made of two lanes, so we need to create a separate geometry for each pair of lanes
                    // Also within each lane, we need to create a separate geometry for each material segment
                    unsigned int nr_vertices =
                        static_cast<unsigned int>(2 * geom_points_list[0][0].size() * geom_strips_list.size());  // same nr vertices in all lanes
                    osg::ref_ptr<osg::Vec3Array> verticesAll  = new osg::Vec3Array(nr_vertices);
                    osg::ref_ptr<osg::Vec2Array> texcoordsAll = new osg::Vec2Array(nr_vertices);

                    // Potential optimization: Swap loops, creating all vertices for same s-value for each step
                    unsigned int vertex_counter = 0;
                    unsigned int vertex_idx     = 0;
                    double       texscale       = 1.0 / TEXTURE_SCALE;

                    for (size_t k = 0; k < geom_strips_list.size(); k++)  // loop over lanes
                    {
                        osg::ref_ptr<osg::Vec3Array>        verticesLocal;
                        osg::ref_ptr<osg::Vec2Array>        texcoordsLocal;
                        osg::ref_ptr<osg::Vec4Array>        colorLocal;
                        osg::ref_ptr<osg::DrawElementsUInt> indices;
                        lane = lsec->GetLaneById(lane_ids[k]);

                        for (size_t m = 0; m < geom_strips_list[k].size(); m++)  // loop over lane patches with constant friction
                        {
                            lane_friction_                                   = geom_strips_list[k][m].friction;
                            unsigned int                         gpi         = geom_strips_list[k][m].geom_point_index;
                            std::vector<std::vector<GeomPoint>>& geom_points = geom_points_list[k];
                            unsigned int                         n_points    = 0;

                            if (m < geom_strips_list[k].size() - 1)
                            {
                                n_points = geom_strips_list[k][m + 1].geom_point_index - gpi + 1;
                            }
                            else
                            {
                                n_points = geom_points[0].size() - gpi;
                            }

                            verticesLocal  = new osg::Vec3Array(n_points * 2);
                            indices        = new osg::DrawElementsUInt(GL_TRIANGLE_STRIP, n_points * 2);
                            texcoordsLocal = new osg::Vec2Array(n_points * 2);

                            unsigned int index_counter = 0;

                            int z_gap_found = 0;  // -1: left pointing wall, 1: right pointing wall, 0: no z difference => no wall
                            for (size_t l = 0; l < n_points; l++)
                            {
                                if (m == 0 || l > 0)
                                {
                                    // only create vertex when needed, reuse vertex of previous patch for first vertex of any additional patch
                                    for (auto side : {0, 1})  // left, right side of the lane - comply with GL_TRIANGLE_STRIP order
                                    {
                                        GeomPoint& gp = geom_points[side][gpi + l];

                                        // vertex
                                        (*verticesAll)[vertex_counter].set(static_cast<float>(gp.x - origin[0]),
                                                                           static_cast<float>(gp.y - origin[1]),
                                                                           static_cast<float>(gp.z));
                                        (*texcoordsAll)[vertex_counter].set(osg::Vec2(static_cast<float>(texscale * (gp.x - origin[0])),
                                                                                      static_cast<float>(texscale * (gp.y - origin[1]))));
                                        vertex_idx = vertex_counter++;
                                    }

                                    if (!z_gap_found && k > 0)  // look for z gap to the left neighbor lane (skip first)
                                    {
                                        double z_diff = geom_points[0][gpi + l].z - geom_points_list[k - 1][1][gpi + l].z;
                                        if (fabs(z_diff) > 1e-3)
                                        {
                                            z_gap_found = z_diff < 0 ? -1 : 1;
                                        }
                                    }
                                }

                                for (auto side : {0, 1})  // left, right side of the lane
                                {
                                    // Create indices for the lane strip, referring to the vertex list
                                    (*verticesLocal)[index_counter]  = (*verticesAll)[vertex_idx + side - 1];
                                    (*texcoordsLocal)[index_counter] = (*texcoordsAll)[vertex_idx + side - 1];
                                    (*indices)[index_counter++]      = index_counter;
                                }
                            }

                            // Create geometry for the strip made of this and previous lane
                            osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
                            geom->setUseDisplayList(true);
                            geom->setVertexArray(verticesLocal.get());
                            geom->addPrimitiveSet(indices.get());
                            geom->setTexCoordArray(0, texcoordsLocal.get());

                            MaterialType material_t;

                            if (lane->IsType(roadmanager::Lane::LaneType::LANE_TYPE_ANY_ROAD))
                            {
                                material_t = MaterialType::ASPHALT;
                                osg::ref_ptr<osg::Material> materialAsphalt_ =
                                    GetOrCreateMaterial("Asphalt", GetFrictionColor(lane_friction_), static_cast<uint8_t>(material_t), 1);

                                geom->getOrCreateStateSet()->setAttributeAndModes(materialAsphalt_.get());
                            }
                            else if (lane->IsType(roadmanager::Lane::LaneType::LANE_TYPE_BIKING) ||
                                     lane->IsType(roadmanager::Lane::LaneType::LANE_TYPE_SIDEWALK))
                            {
                                material_t = MaterialType::CONCRETE;
                                osg::ref_ptr<osg::Material> materialConcrete_ =
                                    GetOrCreateMaterial("Concrete", color_concrete->at(0), static_cast<uint8_t>(material_t));
                                geom->getOrCreateStateSet()->setAttributeAndModes(materialConcrete_.get());

                                // Use PolygonOffset feature to avoid z-fighting with road surface
                                geom->getOrCreateStateSet()->setAttributeAndModes(
                                    new osg::PolygonOffset(-POLYGON_OFFSET_SIDEWALK, -SIGN(POLYGON_OFFSET_SIDEWALK)));
                            }
                            // for border and grass materials, consider also lane position and neighbor lanes
                            // If lane is inner; either has neighbor lanes on both sides or is the first lane next to center:
                            // "none" type will get border material
                            // "border" type will get grass material
                            else if (((k > 0 && k + 1 < lane_ids.size()) || abs(lane->GetId()) == 1) &&
                                     (lane->IsType(roadmanager::Lane::LaneType::LANE_TYPE_BORDER) ||
                                      lane->IsType(roadmanager::Lane::LaneType::LANE_TYPE_NONE)))
                            {
                                material_t = MaterialType::BORDER;
                                osg::ref_ptr<osg::Material> materialBorderInner_ =
                                    GetOrCreateMaterial("Border", color_border_inner->at(0), static_cast<uint8_t>(material_t));
                                geom->getOrCreateStateSet()->setAttributeAndModes(materialBorderInner_.get());

                                // Use PolygonOffset feature to avoid z-fighting with road surface
                                geom->getOrCreateStateSet()->setAttributeAndModes(
                                    new osg::PolygonOffset(-POLYGON_OFFSET_BORDER, -SIGN(POLYGON_OFFSET_BORDER)));
                            }
                            else
                            {
                                material_t = MaterialType::GRASS;
                                osg::ref_ptr<osg::Material> materialGrass_ =
                                    GetOrCreateMaterial("Grass", color_grass->at(0), static_cast<uint8_t>(material_t));

                                geom->getOrCreateStateSet()->setAttributeAndModes(materialGrass_.get());

                                // Use PolygonOffset feature to avoid z-fighting with road surface
                                geom->getOrCreateStateSet()->setAttributeAndModes(
                                    new osg::PolygonOffset(-POLYGON_OFFSET_GRASS, -SIGN(POLYGON_OFFSET_GRASS)));
                            }
                            geom->setColorBinding(osg::Geometry::BIND_OVERALL);

                            // See if the material type has a texture associated with it, if so, apply it
                            auto texture_it = texture_map_.find(material_t);
                            if (texture_it != texture_map_.end())
                            {
                                geom->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture_it->second.get());
                            }

                            osg::ref_ptr<osg::Geode> geode = new osg::Geode;
                            geode->addDrawable(geom.get());

                            // osgUtil::Optimizer optimizer;
                            // optimizer.optimize(geode);
                            SetNodeName(*geode, prefix_road, road->GetId(), std::to_string(k) + "_" + std::to_string(m));
                            r_group_->addChild(geode);

                            if (z_gap_found != 0)
                            {
                                // create a patch to fill the verical gap using material of outer patch

                                osg::ref_ptr<osg::Geode> geode_to_clone;
                                if (z_gap_found == 1)
                                {
                                    // vertical patch is facing left, use material from current lane
                                    geode_to_clone = static_cast<osg::Geode*>(r_group_->getChild(r_group_->getNumChildren() - 1));
                                }
                                else
                                {
                                    // vertical patch is facing right, use material from previous (left) patch
                                    geode_to_clone = static_cast<osg::Geode*>(r_group_->getChild(r_group_->getNumChildren() - 2));
                                }

                                osg::ref_ptr<osg::Geode> geode_gap = static_cast<osg::Geode*>(geode_to_clone->clone(osg::CopyOp::DEEP_COPY_ALL));
                                if (geode_gap)
                                {
                                    osg::ref_ptr<osg::Vec3Array> vertices_gap = new osg::Vec3Array(n_points * 2);
                                    for (size_t l = 0; l < n_points; l++)
                                    {
                                        // vertex from previous patch
                                        (*vertices_gap)[l * 2 + 0].set(geom_points_list[k - 1][1][gpi + l].x - origin[0],
                                                                       geom_points_list[k - 1][1][gpi + l].y - origin[1],
                                                                       geom_points_list[k - 1][1][gpi + l].z);
                                        // vertex from current patch
                                        (*vertices_gap)[l * 2 + 1].set(osg::Vec3(geom_points_list[k][0][gpi + l].x - origin[0],
                                                                                 geom_points_list[k][0][gpi + l].y - origin[1],
                                                                                 geom_points_list[k][0][gpi + l].z));
                                    }

                                    osg::Geometry* geom_gap = geode_gap->getDrawable(0)->asGeometry();
                                    if (geom_gap)
                                    {
                                        geom_gap->setVertexArray(vertices_gap.get());
                                        r_group_->addChild(geode_gap);
                                        LOG_DEBUG("Added vertical gap patch for road {} section {} between lanes {} and {}",
                                                  road->GetId(),
                                                  j,
                                                  lane_ids[k],
                                                  lane_ids[k - 1]);
                                    }
                                    else
                                    {
                                        LOG_WARN("Failed to create geom for vertical gap patch at road {} section {} between lanes {} and {}",
                                                 road->GetId(),
                                                 j,
                                                 lane_ids[k],
                                                 lane_ids[k - 1]);
                                    }
                                }
                                else
                                {
                                    LOG_WARN("Failed to create geode for vertical gap patch for road {} section {} between lanes {} and {}",
                                             road->GetId(),
                                             j,
                                             lane_ids[k],
                                             lane_ids[k - 1]);
                                }
                            }
                        }
                    }

                    for (unsigned int l = 0; l < lsec->GetNumberOfLanes(); l++)
                    {
                        AddRoadMarks(lsec->GetLaneByIdx(l), rm_group_, origin);
                    }
                }
            }
            for (unsigned int i = 0; i < r_group_->getNumChildren(); i++)
            {
                // calculate normals for all lane strips
                osg::ref_ptr<osg::Geode> lane_patch_geode = static_cast<osg::Geode*>(r_group_->getChild(i));
                if (lane_patch_geode)
                {
                    osgUtil::SmoothingVisitor::smooth(*lane_patch_geode->getDrawable(0)->asGeometry(), 0.5);
                }
            }
        }

        if (generate_road_objects)
        {
            if (odrManager_->GetNumOfRoads() > 0 && CreateRoadSignsAndObjects(odrManager_, origin, generate_road_surface, root_, exe_path) != 0)
            {
                LOG_ERROR("Viewer::Viewer Failed to create road signs and objects!");
            }
        }

        std::string opt_groundplane = SE_Env::Inst().GetOptions().GetOptionValueByEnum(esmini_options::GROUND_PLANE);
        if ((opt_groundplane == "auto" && (!generate_road_surface && environment == nullptr)) || opt_groundplane == "on")
        {
            AddGroundSurface();
        }
    }

    int RoadGeom::CreateRoadSignsAndObjects(roadmanager::OpenDrive*  od,
                                            const osg::Vec3d&        origin,
                                            bool                     stand_in_model,
                                            osg::ref_ptr<osg::Group> parent,
                                            std::string              exe_path)
    {
        osg::ref_ptr<osg::Group>                     objGroup = new osg::Group;
        osg::ref_ptr<osg::PositionAttitudeTransform> tx       = nullptr;

        objGroup->setName("road_objects");

        roadmanager::Position pos;

        for (unsigned int r = 0; r < od->GetNumOfRoads(); r++)
        {
            roadmanager::Road* road = od->GetRoadByIdx(r);

            for (unsigned int s = 0; s < road->GetNumberOfSignals(); s++)
            {
                osg::ref_ptr<osg::Group> signGroup = new osg::Group;
                tx                                 = nullptr;
                roadmanager::Signal* signal        = road->GetSignal(s);
                SetNodeName(*signGroup, prefix_road_signal, s, signal->GetName());

                // create a bounding for the sign
                osg::ref_ptr<osg::PositionAttitudeTransform> tx_bb = new osg::PositionAttitudeTransform;
                signGroup->addChild(tx_bb);

                // avoid zero width, length and width - set to a minimum value of 0.05m
                osg::ref_ptr<osg::ShapeDrawable> shape =
                    new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0f, 0.0f, 0.5f * MAX(0.05f, static_cast<float>(signal->GetHeight()))),
                                                        MAX(0.05f, static_cast<float>(signal->GetDepth())),
                                                        MAX(0.05f, static_cast<float>(signal->GetWidth())),
                                                        MAX(0.05f, static_cast<float>(signal->GetHeight()))));

                shape->setColor(osg::Vec4(0.8f, 0.8f, 0.8f, 1.0f));
                tx_bb->addChild(shape);
                tx_bb->setPosition(osg::Vec3(static_cast<float>(signal->GetX() - origin[0]),
                                             static_cast<float>(signal->GetY() - origin[1]),
                                             static_cast<float>(signal->GetZ() + signal->GetZOffset())));
                tx_bb->setAttitude(osg::Quat(signal->GetH() + signal->GetHOffset(), osg::Vec3(0, 0, 1)));

                if (stand_in_model == true || !SE_Env::Inst().GetOptions().GetOptionSet("use_signs_in_external_model"))
                {
                    // Road sign filename is the combination of type_subtype_value
                    std::string filename = signal->GetCountry() + "_" + signal->GetType();
                    if (!(signal->GetSubType().empty() || signal->GetSubType() == "none" || signal->GetSubType() == "-1"))
                    {
                        filename += "_" + signal->GetSubType();
                    }

                    if (!(NEAR_NUMBERS(signal->GetValue(), -1.0) || signal->GetValueStr().empty()))
                    {
                        filename += "-" + signal->GetValueStr();
                    }

                    bool        found = false;
                    std::string located_file_path =
                        LocateFile(filename + ".osgb",
                                   {DirNameOf(odrManager_->GetOpenDriveFilename()) + "/../models", DirNameOf(exe_path) + "/../resources/models"},
                                   "Road signal 3D model",
                                   found);

                    if (found)
                    {
                        tx = LoadRoadFeature(road, located_file_path);
                    }

                    if (tx == nullptr)
                    {
                        // if file according to type, subtype and value could not be resolved, try from name
                        located_file_path =
                            LocateFile(signal->GetName() + ".osgb",
                                       {DirNameOf(odrManager_->GetOpenDriveFilename()) + "/../models", DirNameOf(exe_path) + "/../resources/models"},
                                       "Road signal 3D model",
                                       found);
                        if (found)
                        {
                            tx = LoadRoadFeature(road, signal->GetName() + ".osgb");
                        }
                    }

                    if (found)
                    {
                        signal->SetModel3DFullPath(located_file_path);
                    }

                    if (tx == nullptr && typeid(*signal) == typeid(roadmanager::TrafficLight))
                    {
                        std::string texture_file_name = "opendrive_" + signal->GetCombinedType() + ".png";

                        auto it = roadmanager::traffic_light_type_map.find(signal->GetCombinedType());

                        if (it != roadmanager::traffic_light_type_map.end())
                        {
                            roadmanager::TrafficLightInfo tl_info = it->second;

                            traffic_light_.emplace(
                                signal->GetId(),
                                TrafficLightModel(tl_info.nr_lamps,
                                                  LocateFile(texture_file_name,
                                                             {DirNameOf(odrManager_->GetOpenDriveFilename()) + "/../models/roads_signal_textures",
                                                              DirNameOf(exe_path) + "/../resources/models/roads_signal_textures"},
                                                             "Traffic light texture",
                                                             found)));
                            if (found)
                            {
                                tx = traffic_light_.at(signal->GetId()).GetTx();
                            }
                        }
                        else
                        {
                            LOG_ERROR("Unsupport traffic signal type {}, can't resolve corresponding texture file", signal->GetType());
                        }
                    }

                    if (tx == nullptr)
                    {
                        LOG_DEBUG("Failed to load signal {}.osgb / {}.osgb or create 3D model - using simple bounding box",
                                  FileNameOf(filename),
                                  signal->GetName());
                        osg::ref_ptr<osg::PositionAttitudeTransform> obj_standin =
                            dynamic_cast<osg::PositionAttitudeTransform*>(tx_bb->clone(osg::CopyOp::DEEP_COPY_ALL));
                        obj_standin->setNodeMask(NODE_MASK_SIGN);
                        signGroup->addChild(obj_standin);
                    }
                    else
                    {
                        tx->setPosition(osg::Vec3(static_cast<float>(signal->GetX() - origin[0]),
                                                  static_cast<float>(signal->GetY() - origin[1]),
                                                  static_cast<float>(signal->GetZ() + signal->GetZOffset())));
                        tx->setAttitude(osg::Quat(signal->GetH() + signal->GetHOffset(), osg::Vec3(0, 0, 1)));
                        tx->setNodeMask(NODE_MASK_SIGN);
                        signGroup->addChild(tx);
                    }
                }

                // set bounding box to wireframe mode
                osg::PolygonMode* polygonMode = new osg::PolygonMode;
                polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
                shape->getOrCreateStateSet()->setAttributeAndModes(polygonMode, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
                tx_bb->setNodeMask(NODE_MASK_ODR_FEATURES);

                objGroup->addChild(signGroup);
            }

            AddExpandedObjectsForRoad(road, origin, objGroup, exe_path);
        }

        if (optimize_)
        {
            // For some reason this operation ruins the positioning of road objects in exported model
            osgUtil::Optimizer optimizer;
            optimizer.optimize(objGroup, osgUtil::Optimizer::FLATTEN_STATIC_TRANSFORMS);
        }

        parent->addChild(objGroup);

        return 0;
    }

    osg::ref_ptr<osg::PositionAttitudeTransform> RoadGeom::LoadRoadFeature(roadmanager::Road* road, std::string file_path)
    {
        (void)road;
        osg::ref_ptr<osg::Node>                      node;
        osg::ref_ptr<osg::PositionAttitudeTransform> xform = 0;

        node = osgDB::readNodeFile(file_path);
        if (!node)
        {
            return 0;
        }

        xform = new osg::PositionAttitudeTransform;
        xform->addChild(node);

        return xform;
    }

    osg::ref_ptr<osg::Group> RoadGeom::CreateOutlineObject(roadmanager::Outline* outline, osg::Vec4 color, const osg::Vec3d& origin)
    {
        if (outline == 0)
        {
            return nullptr;
        }

        bool roof = outline->roof_ ? true : false;

        // nrPoints will be corners + 1 if the outline should be closed, reusing first corner as last
        int nrPoints = outline->closed_ ? static_cast<int>(outline->corner_.size()) + 1 : static_cast<int>(outline->corner_.size());

        osg::ref_ptr<osg::Group> group = new osg::Group();

        osg::ref_ptr<osg::Vec3Array> vertices_sides =
            new osg::Vec3Array(static_cast<unsigned int>(nrPoints) * 2);                                         // one set at bottom and one at top
        osg::ref_ptr<osg::Vec3Array> vertices_top    = new osg::Vec3Array(static_cast<unsigned int>(nrPoints));  // top
        osg::ref_ptr<osg::Vec3Array> vertices_bottom = new osg::Vec3Array(static_cast<unsigned int>(nrPoints));  // bottom

        osg::ref_ptr<osg::Vec2Array> tex_coords_sides  = new osg::Vec2Array(static_cast<unsigned int>(nrPoints) * 2);
        osg::ref_ptr<osg::Vec2Array> tex_coords_top    = new osg::Vec2Array(static_cast<unsigned int>(nrPoints));
        osg::ref_ptr<osg::Vec2Array> tex_coords_bottom = new osg::Vec2Array(static_cast<unsigned int>(nrPoints));

        // Set vertices
        float cumulative_side_dist = 0.0f;
        for (size_t i = 0; i < outline->corner_.size(); i++)
        {
            double                      x, y, z;
            roadmanager::OutlineCorner* corner = outline->corner_[i];
            corner->GetPos(x, y, z);
            (*vertices_sides)[i * 2 + 0].set(static_cast<float>(x - origin[0]),
                                             static_cast<float>(y - origin[1]),
                                             static_cast<float>(z + corner->GetHeight()));
            (*vertices_sides)[i * 2 + 1].set(static_cast<float>(x - origin[0]), static_cast<float>(y - origin[1]), static_cast<float>(z));

            if (i > 0)
            {
                double x1, y1, z1;
                outline->corner_[i - 1]->GetPos(x1, y1, z1);
                float dx = x1 - x;
                float dy = y1 - y;
                cumulative_side_dist += std::sqrt(dx * dx + dy * dy);
            }

            (*tex_coords_sides)[i * 2 + 0].set(cumulative_side_dist, corner->GetHeight());
            (*tex_coords_sides)[i * 2 + 1].set(cumulative_side_dist, 0.0f);

            // top and bottom shapes
            if (outline->GetCountourType() == roadmanager::Outline::ContourType::CONTOUR_TYPE_POLYGON)
            {
                (*vertices_top)[i].set(static_cast<float>(x - origin[0]),
                                       static_cast<float>(y - origin[1]),
                                       static_cast<float>(z + corner->GetHeight()));
                (*vertices_bottom)[outline->corner_.size() - 1 - i].set(static_cast<float>(x - origin[0]),
                                                                        static_cast<float>(y - origin[1]),
                                                                        static_cast<float>(z));

                double x2, y2, z2;
                outline->corner_[outline->corner_.size() - 1 - i]->GetPos(x2, y2, z2);
                float dx    = x2 - x;
                float dy    = y2 - y;
                float width = std::sqrt(dx * dx + dy * dy);

                (*tex_coords_top)[i].set(0.0, cumulative_side_dist);
                (*tex_coords_top)[outline->corner_.size() - 1 - i].set(width, cumulative_side_dist);
                (*tex_coords_bottom)[i].set(0.0, cumulative_side_dist);
                (*tex_coords_bottom)[outline->corner_.size() - 1 - i].set(width, cumulative_side_dist);
            }
        }

        if (outline->GetCountourType() == roadmanager::Outline::ContourType::CONTOUR_TYPE_QUAD_STRIP)
        {
            float cumulative_roof_dist = 0.0f;
            // rearrange vertices for quad strip
            for (size_t i = 0; i < outline->corner_.size(); i += 2)
            {
                unsigned right_index = outline->corner_.size() - 1 - (i / 2);
                unsigned left_index  = i / 2;

                double xl, yl, zl, xr, yr, zr;
                outline->corner_[left_index]->GetPos(xl, yl, zl);
                outline->corner_[right_index]->GetPos(xr, yr, zr);

                (*vertices_top)[i].set(static_cast<float>(xr - origin[0]),
                                       static_cast<float>(yr - origin[1]),
                                       static_cast<float>(zr + outline->corner_[right_index]->GetHeight()));
                (*vertices_top)[i + 1].set(static_cast<float>(xl - origin[0]),
                                           static_cast<float>(yl - origin[1]),
                                           static_cast<float>(zl + outline->corner_[right_index]->GetHeight()));
                (*vertices_bottom)[i].set(static_cast<float>(xl - origin[0]), static_cast<float>(yl - origin[1]), static_cast<float>(zl));
                (*vertices_bottom)[i + 1].set(static_cast<float>(xr - origin[0]), static_cast<float>(yr - origin[1]), static_cast<float>(zr));

                osg::Vec3f left(xl, yl, zl);
                osg::Vec3f right(xr, yr, zr);
                float      width = (right - left).length();

                if (i >= 2)
                {
                    unsigned left_index_prev  = (i - 1) / 2;
                    unsigned right_index_prev = outline->corner_.size() - 1 - ((i - 2) / 2);

                    double xl_prev, yl_prev, zl_prev, xr_prev, yr_prev, zr_prev;
                    outline->corner_[left_index_prev]->GetPos(xl_prev, yl_prev, zl_prev);
                    outline->corner_[right_index_prev]->GetPos(xr_prev, yr_prev, zr_prev);

                    osg::Vec3f left_prev(xl_prev, yl_prev, zl_prev);
                    osg::Vec3f right_prev(xr_prev, yr_prev, zr_prev);

                    osg::Vec3f center_prev = (left_prev + right_prev) * 0.5f;
                    osg::Vec3f center      = (left + right) * 0.5f;

                    cumulative_roof_dist += (center - center_prev).length();
                }

                (*tex_coords_top)[i].set(width, cumulative_roof_dist);
                (*tex_coords_top)[i + 1].set(0.0f, cumulative_roof_dist);
                (*tex_coords_bottom)[i].set(width, cumulative_roof_dist);
                (*tex_coords_bottom)[i + 1].set(0.0f, cumulative_roof_dist);
            }
        }

        // Close geometry
        if (outline->closed_)
        {
            cumulative_side_dist += ((*vertices_sides)[0] - (*vertices_sides)[2 * (nrPoints - 2)]).length();

            (*vertices_sides)[2 * static_cast<unsigned int>(nrPoints) - 2].set((*vertices_sides)[0]);
            (*vertices_sides)[2 * static_cast<unsigned int>(nrPoints) - 1].set((*vertices_sides)[1]);
            (*tex_coords_sides)[2 * static_cast<unsigned int>(nrPoints) - 2].set(cumulative_side_dist, (*tex_coords_sides)[0].y());
            (*tex_coords_sides)[2 * static_cast<unsigned int>(nrPoints) - 1].set(cumulative_side_dist, (*tex_coords_sides)[1].y());

            (*vertices_top)[static_cast<unsigned int>(nrPoints) - 1].set((*vertices_top)[0]);
            (*vertices_bottom)[static_cast<unsigned int>(nrPoints) - 1].set((*vertices_bottom)[0]);
            (*tex_coords_top)[static_cast<unsigned int>(nrPoints) - 1].set((*tex_coords_top)[0]);
            (*tex_coords_bottom)[static_cast<unsigned int>(nrPoints) - 1].set((*tex_coords_bottom)[0]);
        }

        // Finally create and add geometry
        osg::ref_ptr<osg::Geode>    geode  = new osg::Geode;
        osg::ref_ptr<osg::Geometry> geom[] = {new osg::Geometry, new osg::Geometry, new osg::Geometry};

        geom[0]->setVertexArray(vertices_sides.get());
        geom[0]->setTexCoordArray(0, tex_coords_sides.get());
        geom[0]->addPrimitiveSet(new osg::DrawArrays(GL_QUAD_STRIP, 0, 2 * nrPoints));

        if (roof)
        {
            geom[1]->setVertexArray(vertices_top.get());
            geom[1]->setTexCoordArray(0, tex_coords_top.get());
            geom[2]->setVertexArray(vertices_bottom.get());
            geom[2]->setTexCoordArray(0, tex_coords_bottom.get());
            if (outline->GetCountourType() == roadmanager::Outline::ContourType::CONTOUR_TYPE_POLYGON)
            {
                geom[1]->addPrimitiveSet(new osg::DrawArrays(GL_POLYGON, 0, nrPoints));
                osgUtil::Tessellator tessellator;
                tessellator.retessellatePolygons(*geom[1]);

                geom[2]->addPrimitiveSet(new osg::DrawArrays(GL_POLYGON, 0, nrPoints));
                tessellator.retessellatePolygons(*geom[2]);
            }
            else
            {
                geom[1]->addPrimitiveSet(new osg::DrawArrays(GL_QUAD_STRIP, 0, nrPoints - 1));
                geom[2]->addPrimitiveSet(new osg::DrawArrays(GL_QUAD_STRIP, 0, nrPoints - 1));
            }
        }

        int nrGeoms = roof ? 3 : 1;
        for (int i = 0; i < nrGeoms; i++)
        {
            osgUtil::SmoothingVisitor::smooth(*geom[i], 0.5);
            geom[i]->setDataVariance(osg::Object::STATIC);
            geom[i]->setUseDisplayList(true);
            geode->addDrawable(geom[i]);
        }

        osg::ref_ptr<osg::Material> material_ = new osg::Material;
        material_->setDiffuse(osg::Material::FRONT_AND_BACK, color);
        material_->setAmbient(osg::Material::FRONT_AND_BACK, color);
        geode->getOrCreateStateSet()->setAttributeAndModes(material_.get());
        geode->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);

        const bool transparent = (color.a() < 0.999f);
        if (transparent)
        {
            geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
            geode->getOrCreateStateSet()->setRenderBinDetails(20, "DepthSortedBin", osg::StateSet::RenderBinMode::OVERRIDE_RENDERBIN_DETAILS);
            geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
        }
        else
        {
            geode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::OPAQUE_BIN);
            geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::OFF);
        }

        group->addChild(geode);

        return group;
    }

    osg::ref_ptr<osg::Group> RoadGeom::CreateExpandedContinuousSegmentGeom(const std::array<roadmanager::Vec2, 4>& corners,
                                                                           double                                  z0,
                                                                           double                                  z1,
                                                                           double                                  heightStart,
                                                                           double                                  heightEnd,
                                                                           bool                                    emitStartCap,
                                                                           bool                                    emitEndCap,
                                                                           osg::Vec4                               color,
                                                                           const osg::Vec3d&                       origin)
    {
        // Corner layout: [0]=start/neg-t  [1]=start/pos-t  [2]=end/pos-t  [3]=end/neg-t
        const auto toX = [&](double v) { return static_cast<float>(v - origin[0]); };
        const auto toY = [&](double v) { return static_cast<float>(v - origin[1]); };

        const float c0x = toX(corners[0].x);
        const float c0y = toY(corners[0].y);
        const float c1x = toX(corners[1].x);
        const float c1y = toY(corners[1].y);
        const float c2x = toX(corners[2].x);
        const float c2y = toY(corners[2].y);
        const float c3x = toX(corners[3].x);
        const float c3y = toY(corners[3].y);

        const float b0 = static_cast<float>(z0);                // base Z at start edge
        const float b1 = static_cast<float>(z1);                // base Z at end edge
        const float t0 = b0 + static_cast<float>(heightStart);  // top  Z at start edge
        const float t1 = b1 + static_cast<float>(heightEnd);    // top  Z at end edge

        // Shared side vertices:
        // 0=c0 bottom, 1=c0 top, 2=c1 bottom, 3=c1 top, 4=c2 bottom, 5=c2 top, 6=c3 bottom, 7=c3 top
        osg::ref_ptr<osg::Vec3Array> verts_sides = new osg::Vec3Array(8);
        (*verts_sides)[0].set(c0x, c0y, b0);
        (*verts_sides)[1].set(c0x, c0y, t0);
        (*verts_sides)[2].set(c1x, c1y, b0);
        (*verts_sides)[3].set(c1x, c1y, t0);
        (*verts_sides)[4].set(c2x, c2y, b1);
        (*verts_sides)[5].set(c2x, c2y, t1);
        (*verts_sides)[6].set(c3x, c3y, b1);
        (*verts_sides)[7].set(c3x, c3y, t1);

        osg::ref_ptr<osg::DrawElementsUInt> side_indices = new osg::DrawElementsUInt(GL_QUADS);

        // Outer longitudinal walls (always visible from outside).
        // Right wall (negative t): c0 -> c3
        side_indices->push_back(0);
        side_indices->push_back(1);
        side_indices->push_back(7);
        side_indices->push_back(6);

        // Left wall (positive t): c2 -> c1
        side_indices->push_back(4);
        side_indices->push_back(5);
        side_indices->push_back(3);
        side_indices->push_back(2);

        // End caps are only needed on the outermost segments.
        if (emitStartCap)
        {
            side_indices->push_back(2);
            side_indices->push_back(3);
            side_indices->push_back(1);
            side_indices->push_back(0);
        }

        if (emitEndCap)
        {
            side_indices->push_back(6);
            side_indices->push_back(7);
            side_indices->push_back(5);
            side_indices->push_back(4);
        }

        // Top face QUAD CCW from above: c0, c3, c2, c1
        osg::ref_ptr<osg::Vec3Array> verts_top = new osg::Vec3Array(4);
        (*verts_top)[0].set(c0x, c0y, t0);
        (*verts_top)[1].set(c3x, c3y, t1);
        (*verts_top)[2].set(c2x, c2y, t1);
        (*verts_top)[3].set(c1x, c1y, t0);

        // Bottom face QUAD (opposite winding to top): c0, c1, c2, c3
        osg::ref_ptr<osg::Vec3Array> verts_bottom = new osg::Vec3Array(4);
        (*verts_bottom)[0].set(c0x, c0y, b0);
        (*verts_bottom)[1].set(c1x, c1y, b0);
        (*verts_bottom)[2].set(c2x, c2y, b1);
        (*verts_bottom)[3].set(c3x, c3y, b1);

        osg::ref_ptr<osg::Vec4Array> color_arr = new osg::Vec4Array(1);
        (*color_arr)[0]                        = color;

        osg::ref_ptr<osg::Geometry> geom_sides = new osg::Geometry;
        geom_sides->setVertexArray(verts_sides);
        geom_sides->addPrimitiveSet(side_indices);
        geom_sides->setColorArray(color_arr);
        geom_sides->setColorBinding(osg::Geometry::BIND_OVERALL);
        osgUtil::SmoothingVisitor::smooth(*geom_sides, 0.5);
        geom_sides->setDataVariance(osg::Object::STATIC);
        geom_sides->setUseDisplayList(true);

        osg::ref_ptr<osg::Geometry> geom_top = new osg::Geometry;
        geom_top->setVertexArray(verts_top);
        geom_top->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));
        geom_top->setColorArray(color_arr);
        geom_top->setColorBinding(osg::Geometry::BIND_OVERALL);
        osgUtil::SmoothingVisitor::smooth(*geom_top, 0.5);
        geom_top->setDataVariance(osg::Object::STATIC);
        geom_top->setUseDisplayList(true);

        osg::ref_ptr<osg::Geometry> geom_bottom = new osg::Geometry;
        geom_bottom->setVertexArray(verts_bottom);
        geom_bottom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));
        geom_bottom->setColorArray(color_arr);
        geom_bottom->setColorBinding(osg::Geometry::BIND_OVERALL);
        osgUtil::SmoothingVisitor::smooth(*geom_bottom, 0.5);
        geom_bottom->setDataVariance(osg::Object::STATIC);
        geom_bottom->setUseDisplayList(true);

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        geode->addDrawable(geom_sides);
        geode->addDrawable(geom_top);
        geode->addDrawable(geom_bottom);

        osg::ref_ptr<osg::Material> mat = new osg::Material;
        mat->setDiffuse(osg::Material::FRONT_AND_BACK, color);
        mat->setAmbient(osg::Material::FRONT_AND_BACK, color);
        geode->getOrCreateStateSet()->setAttributeAndModes(mat.get());
        geode->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

        osg::ref_ptr<osg::Group> seg_group = new osg::Group;
        seg_group->addChild(geode);
        return seg_group;
    }

    void RoadGeom::AddExpandedObjectsForRoad(roadmanager::Road*       road,
                                             const osg::Vec3d&        origin,
                                             osg::ref_ptr<osg::Group> parent,
                                             const std::string&       exe_path)
    {
        if (road == nullptr)
        {
            return;
        }

        // Build id→color and id→object lookups from legacy RMObject.
        std::unordered_map<id_t, osg::Vec4>                              color_map;
        std::unordered_map<id_t, roadmanager::RMObject*>                 object_map;
        std::unordered_map<id_t, const roadmanager::RMObjectDefinition*> def_map;
        for (unsigned int o = 0; o < road->GetNumberOfObjects(); ++o)
        {
            roadmanager::RMObject* obj = road->GetRoadObject(o);
            if (obj == nullptr)
            {
                continue;
            }
            osg::Vec4 c;
            for (int ci = 0; ci < 4; ++ci)
            {
                c[ci] = obj->GetColor()[ci];
            }
            color_map[obj->GetId()]  = c;
            object_map[obj->GetId()] = obj;
        }

        for (unsigned int od = 0; od < road->GetNumberOfObjectDefinitions(); ++od)
        {
            const roadmanager::RMObjectDefinition* def = road->GetObjectDefinition(od);
            if (def == nullptr)
            {
                continue;
            }
            def_map[def->id] = def;
        }

        auto get_object_node_prefix = [&](id_t source_object_id) -> const std::string&
        {
            auto it = object_map.find(source_object_id);
            if (it != object_map.end() && it->second != nullptr)
            {
                switch (it->second->GetTunnelComponentType())
                {
                    case roadmanager::RMObject::TunnelComponentType::TUNNEL_WALL:
                        return prefix_tunnel_wall;
                    case roadmanager::RMObject::TunnelComponentType::TUNNEL_ROOF:
                        return prefix_tunnel_roof;
                    default:
                        break;
                }
            }

            return prefix_road_object;
        };

        const std::vector<roadmanager::RMExpandedObject> expanded = roadmanager::ExpandRoadObjectDefinitions(*road);

        std::unordered_map<uint64_t, int> first_segment_index_by_repeat;
        std::unordered_map<uint64_t, int> last_segment_index_by_repeat;
        auto                              make_repeat_key = [](id_t source_object_id, int repeat_index) -> uint64_t
        { return (static_cast<uint64_t>(source_object_id) << 32) | static_cast<uint32_t>(repeat_index); };

        for (const auto& exp : expanded)
        {
            if (exp.kind != roadmanager::RMExpandedObject::Kind::CONTINUOUS_REPEAT_SEGMENT || exp.segmentIndex < 0)
            {
                continue;
            }

            const uint64_t key = make_repeat_key(exp.sourceObjectId, exp.repeatIndex);
            auto           itf = first_segment_index_by_repeat.find(key);
            auto           itl = last_segment_index_by_repeat.find(key);

            if (itf == first_segment_index_by_repeat.end() || exp.segmentIndex < itf->second)
            {
                first_segment_index_by_repeat[key] = exp.segmentIndex;
            }
            if (itl == last_segment_index_by_repeat.end() || exp.segmentIndex > itl->second)
            {
                last_segment_index_by_repeat[key] = exp.segmentIndex;
            }
        }

        roadmanager::Position pos;
        struct ExpandedRoadObjectModel
        {
            bool                                         attempted = false;
            std::string                                  full_path;
            osg::ref_ptr<osg::PositionAttitudeTransform> tx    = nullptr;
            double                                       dim_x = 0.0;
            double                                       dim_y = 0.0;
            double                                       dim_z = 0.0;
        };
        std::unordered_map<id_t, ExpandedRoadObjectModel> model_cache;

        auto get_model_for_source = [&](id_t source_object_id) -> ExpandedRoadObjectModel&
        {
            ExpandedRoadObjectModel& model = model_cache[source_object_id];
            if (model.attempted)
            {
                return model;
            }

            model.attempted = true;

            std::string source_name;
            auto        oit = object_map.find(source_object_id);
            if (oit != object_map.end() && oit->second != nullptr)
            {
                source_name = oit->second->GetName();
            }
            else
            {
                auto dit = def_map.find(source_object_id);
                if (dit != def_map.end() && dit->second != nullptr)
                {
                    source_name = dit->second->name;
                }
            }

            model.full_path = LocateRoadObjectModelFile(source_name, odrManager_->GetOpenDriveFilename(), exe_path);
            if (model.full_path.empty())
            {
                return model;
            }

            model.tx = LoadRoadFeature(road, model.full_path);
            if (model.tx == nullptr)
            {
                LOG_WARN("Failed to load road object model file: {} ({}). Creating fallback geometry.", FileNameOf(model.full_path), source_name);
                return model;
            }

            GetNodeDimensions(*model.tx, model.dim_x, model.dim_y, model.dim_z);
            if (oit != object_map.end() && oit->second != nullptr)
            {
                oit->second->SetModel3DFullPath(model.full_path);
            }

            return model;
        };

        auto resolve_outline_corner_index =
            [&](roadmanager::Outline* outline, const roadmanager::RMOutlineDefinition* outline_def, int ref) -> std::optional<size_t>
        {
            if (outline == nullptr)
            {
                return std::nullopt;
            }

            const size_t n = outline->corner_.size();
            if (n == 0)
            {
                return std::nullopt;
            }

            bool has_explicit_corner_ids = false;
            if (outline_def != nullptr)
            {
                for (const auto& corner_def : outline_def->corners)
                {
                    if (corner_def.id != ID_UNDEFINED)
                    {
                        has_explicit_corner_ids = true;
                        break;
                    }
                }

                if (has_explicit_corner_ids)
                {
                    if (ref < 0)
                    {
                        return std::nullopt;
                    }

                    const id_t ref_id = static_cast<id_t>(ref);
                    for (size_t i = 0; i < outline_def->corners.size() && i < n; ++i)
                    {
                        if (outline_def->corners[i].id == ref_id)
                        {
                            return i;
                        }
                    }
                    return std::nullopt;
                }
            }

            if (ref >= 0 && ref < static_cast<int>(n))
            {
                return static_cast<size_t>(ref);
            }
            if (ref > 0 && ref <= static_cast<int>(n))
            {
                return static_cast<size_t>(ref - 1);
            }
            return std::nullopt;
        };

        auto build_outline_marking_edges = [&](roadmanager::Outline*                         outline,
                                               const roadmanager::RMOutlineDefinition*       outline_def,
                                               const roadmanager::RMObjectMarkingDefinition& marking,
                                               double                                        z_offset) -> std::vector<MarkingEdge>
        {
            std::vector<MarkingEdge> edges;
            if (outline == nullptr || outline->corner_.size() < 2)
            {
                return edges;
            }

            if (marking.cornerReferences.size() >= 2)
            {
                for (size_t k = 0; k + 1 < marking.cornerReferences.size(); ++k)
                {
                    std::optional<size_t> i_opt = resolve_outline_corner_index(outline, outline_def, marking.cornerReferences[k]);
                    std::optional<size_t> j_opt = resolve_outline_corner_index(outline, outline_def, marking.cornerReferences[k + 1]);
                    if (!i_opt.has_value() || !j_opt.has_value())
                    {
                        continue;
                    }

                    const size_t i = i_opt.value();
                    const size_t j = j_opt.value();
                    if (i == j)
                    {
                        continue;
                    }

                    double x0, y0, z0, x1, y1, z1;
                    outline->corner_[i]->GetPos(x0, y0, z0);
                    outline->corner_[j]->GetPos(x1, y1, z1);

                    const double mark_z0 = z0 + z_offset + ROADMARK_Z_OFFSET;
                    const double mark_z1 = z1 + z_offset + ROADMARK_Z_OFFSET;
                    edges.push_back({osg::Vec3d(x0 - origin[0], y0 - origin[1], mark_z0), osg::Vec3d(x1 - origin[0], y1 - origin[1], mark_z1)});
                }

                return edges;
            }

            const size_t edge_count = outline->closed_ ? outline->corner_.size() : (outline->corner_.size() - 1);
            for (size_t i = 0; i < edge_count; ++i)
            {
                const bool include_edge =
                    marking.edgeReferences.empty() || std::any_of(marking.edgeReferences.begin(),
                                                                  marking.edgeReferences.end(),
                                                                  [i, edge_count](int ref) { return EdgeReferenceMatches(ref, i, edge_count); });
                if (!include_edge)
                {
                    continue;
                }

                const size_t j = (i + 1) % outline->corner_.size();
                double       x0, y0, z0, x1, y1, z1;
                outline->corner_[i]->GetPos(x0, y0, z0);
                outline->corner_[j]->GetPos(x1, y1, z1);

                const double mark_z0 = z0 + z_offset + ROADMARK_Z_OFFSET;
                const double mark_z1 = z1 + z_offset + ROADMARK_Z_OFFSET;
                edges.push_back({osg::Vec3d(x0 - origin[0], y0 - origin[1], mark_z0), osg::Vec3d(x1 - origin[0], y1 - origin[1], mark_z1)});
            }

            return edges;
        };

        auto build_outline_marking_corner_points = [&](roadmanager::Outline*                         outline,
                                                       const roadmanager::RMOutlineDefinition*       outline_def,
                                                       const roadmanager::RMObjectMarkingDefinition& marking,
                                                       double                                        z_offset) -> std::vector<osg::Vec3d>
        {
            std::vector<osg::Vec3d> points;
            if (outline == nullptr || outline->corner_.empty())
            {
                return points;
            }

            if (marking.cornerReferences.size() < 2)
            {
                return points;
            }

            for (int corner_ref : marking.cornerReferences)
            {
                std::optional<size_t> index = resolve_outline_corner_index(outline, outline_def, corner_ref);
                if (!index.has_value())
                {
                    continue;
                }

                double x, y, z;
                outline->corner_[index.value()]->GetPos(x, y, z);
                const double mark_z = z + z_offset + ROADMARK_Z_OFFSET;
                points.push_back(osg::Vec3d(x - origin[0], y - origin[1], mark_z));
            }

            return points;
        };

        auto build_box_marking_edges = [&](const roadmanager::RMExpandedObject&          exp,
                                           const roadmanager::RMObjectMarkingDefinition& marking,
                                           double                                        z_offset) -> std::vector<MarkingEdge>
        {
            std::vector<MarkingEdge> edges;

            const double w = exp.width.has_value() ? std::max(0.05, exp.width.value()) : 1.0;
            const double l = exp.length.has_value() ? std::max(0.05, exp.length.value()) : 1.0;

            pos.SetTrackPosMode(road->GetId(),
                                exp.s,
                                exp.t,
                                roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL |
                                    roadmanager::Position::PosMode::P_REL | roadmanager::Position::PosMode::R_REL);

            auto local_to_world = [&](double lx, double ly) -> osg::Vec3d
            {
                double rx = 0.0;
                double ry = 0.0;
                RotateVec2D(lx, ly, pos.GetH() + exp.hdg, rx, ry);
                return osg::Vec3d(pos.GetX() - origin[0] + rx, pos.GetY() - origin[1] + ry, pos.GetZ() + exp.zOffset + z_offset + ROADMARK_Z_OFFSET);
            };

            const osg::Vec3d c0 = local_to_world(0.5 * l, -0.5 * w);   // front-right
            const osg::Vec3d c1 = local_to_world(0.5 * l, 0.5 * w);    // front-left
            const osg::Vec3d c2 = local_to_world(-0.5 * l, 0.5 * w);   // rear-left
            const osg::Vec3d c3 = local_to_world(-0.5 * l, -0.5 * w);  // rear-right

            std::array<MarkingEdge, 4> box_edges = {{{c0, c1}, {c1, c2}, {c2, c3}, {c3, c0}}};

            if (!marking.edgeReferences.empty())
            {
                for (size_t i = 0; i < box_edges.size(); ++i)
                {
                    if (std::any_of(marking.edgeReferences.begin(),
                                    marking.edgeReferences.end(),
                                    [i](int ref) { return EdgeReferenceMatches(ref, i, 4); }))
                    {
                        edges.push_back(box_edges[i]);
                    }
                }
                return edges;
            }

            for (const auto& side : marking.sides)
            {
                if (side == roadmanager::RMObjectMarkingDefinition::Side::FRONT)
                    edges.push_back(box_edges[0]);
                else if (side == roadmanager::RMObjectMarkingDefinition::Side::LEFT)
                    edges.push_back(box_edges[1]);
                else if (side == roadmanager::RMObjectMarkingDefinition::Side::REAR)
                    edges.push_back(box_edges[2]);
                else if (side == roadmanager::RMObjectMarkingDefinition::Side::RIGHT)
                    edges.push_back(box_edges[3]);
            }

            return edges;
        };

        auto resolve_corner_reference_index = [&](const roadmanager::RMOutlineDefinition* outline_def, int ref, size_t n) -> std::optional<size_t>
        {
            if (n == 0)
            {
                return std::nullopt;
            }

            bool has_explicit_corner_ids = false;
            if (outline_def != nullptr)
            {
                for (const auto& corner_def : outline_def->corners)
                {
                    if (corner_def.id != ID_UNDEFINED)
                    {
                        has_explicit_corner_ids = true;
                        break;
                    }
                }

                if (has_explicit_corner_ids)
                {
                    if (ref < 0)
                    {
                        return std::nullopt;
                    }

                    const id_t ref_id = static_cast<id_t>(ref);
                    for (size_t i = 0; i < outline_def->corners.size() && i < n; ++i)
                    {
                        if (outline_def->corners[i].id == ref_id)
                        {
                            return i;
                        }
                    }
                    return std::nullopt;
                }
            }

            if (ref >= 0 && ref < static_cast<int>(n))
            {
                return static_cast<size_t>(ref);
            }
            if (ref > 0 && ref <= static_cast<int>(n))
            {
                return static_cast<size_t>(ref - 1);
            }
            return std::nullopt;
        };

        auto build_continuous_segment_marking_edges = [&](const roadmanager::RMExpandedObject&          exp,
                                                          const roadmanager::RMOutlineDefinition*       outline_def,
                                                          const roadmanager::RMObjectMarkingDefinition& marking,
                                                          double                                        z_offset) -> std::vector<MarkingEdge>
        {
            std::vector<MarkingEdge> edges;
            if (!exp.has_world_corners)
            {
                return edges;
            }

            pos.SetTrackPosMode(road->GetId(),
                                exp.s,
                                0.0,
                                roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL |
                                    roadmanager::Position::PosMode::P_REL | roadmanager::Position::PosMode::R_REL);
            const double z0 = pos.GetZ() + exp.zOffset + z_offset + ROADMARK_Z_OFFSET;

            pos.SetTrackPosMode(road->GetId(),
                                exp.sEnd,
                                0.0,
                                roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL |
                                    roadmanager::Position::PosMode::P_REL | roadmanager::Position::PosMode::R_REL);
            const double z1 = pos.GetZ() + exp.zOffsetEnd + z_offset + ROADMARK_Z_OFFSET;

            std::array<osg::Vec3d, 4> corners = {osg::Vec3d(exp.world_corners[0].x - origin[0], exp.world_corners[0].y - origin[1], z0),
                                                 osg::Vec3d(exp.world_corners[1].x - origin[0], exp.world_corners[1].y - origin[1], z0),
                                                 osg::Vec3d(exp.world_corners[2].x - origin[0], exp.world_corners[2].y - origin[1], z1),
                                                 osg::Vec3d(exp.world_corners[3].x - origin[0], exp.world_corners[3].y - origin[1], z1)};

            if (marking.cornerReferences.size() >= 2)
            {
                for (size_t k = 0; k + 1 < marking.cornerReferences.size(); ++k)
                {
                    std::optional<size_t> i_opt = resolve_corner_reference_index(outline_def, marking.cornerReferences[k], corners.size());
                    std::optional<size_t> j_opt = resolve_corner_reference_index(outline_def, marking.cornerReferences[k + 1], corners.size());
                    if (!i_opt.has_value() || !j_opt.has_value() || i_opt.value() == j_opt.value())
                    {
                        continue;
                    }

                    edges.push_back({corners[i_opt.value()], corners[j_opt.value()]});
                }
                return edges;
            }

            const std::array<MarkingEdge, 4> quad_edges = {
                {{corners[0], corners[1]}, {corners[1], corners[2]}, {corners[2], corners[3]}, {corners[3], corners[0]}}};

            if (!marking.edgeReferences.empty())
            {
                for (size_t i = 0; i < quad_edges.size(); ++i)
                {
                    if (std::any_of(marking.edgeReferences.begin(),
                                    marking.edgeReferences.end(),
                                    [i](int ref) { return EdgeReferenceMatches(ref, i, 4); }))
                    {
                        edges.push_back(quad_edges[i]);
                    }
                }
                return edges;
            }

            for (const auto& side : marking.sides)
            {
                if (side == roadmanager::RMObjectMarkingDefinition::Side::FRONT)
                    edges.push_back(quad_edges[2]);
                else if (side == roadmanager::RMObjectMarkingDefinition::Side::LEFT)
                    edges.push_back(quad_edges[1]);
                else if (side == roadmanager::RMObjectMarkingDefinition::Side::REAR)
                    edges.push_back(quad_edges[0]);
                else if (side == roadmanager::RMObjectMarkingDefinition::Side::RIGHT)
                    edges.push_back(quad_edges[3]);
            }

            return edges;
        };

        // Accumulation for continuous-repeat side/edgeRef markings.
        // Instead of per-segment rendering (which misses mitered joints and resets dash phase),
        // we collect corner data per segment and stitch into one polyline per side after the main loop.
        struct ContRepeatSideMarkKey
        {
            uint64_t repeat_key;     // make_repeat_key(sourceObjectId, repeatIndex)
            size_t   marking_index;  // index in def->markings
            bool     operator==(const ContRepeatSideMarkKey& o) const
            {
                return repeat_key == o.repeat_key && marking_index == o.marking_index;
            }
        };
        struct ContRepeatSideMarkKeyHash
        {
            size_t operator()(const ContRepeatSideMarkKey& k) const
            {
                size_t h = std::hash<uint64_t>{}(k.repeat_key);
                h ^= std::hash<size_t>{}(k.marking_index) * 2654435761u;
                return h;
            }
        };
        struct ContRepeatSideMarkAccum
        {
            // (segmentIndex, [c0,c1,c2,c3]) — corners at the marking z offset, unsorted
            std::vector<std::pair<int, std::array<osg::Vec3d, 4>>> seg_corners;
            const roadmanager::RMObjectMarkingDefinition*          marking_def = nullptr;
            osg::Vec4                                              color;
            id_t                                                   source_object_id = 0;
        };
        std::unordered_map<ContRepeatSideMarkKey, ContRepeatSideMarkAccum, ContRepeatSideMarkKeyHash> cont_side_mark_accum;

        struct ContRepeatGeomKey
        {
            uint64_t repeat_key;
            bool     operator==(const ContRepeatGeomKey& o) const
            {
                return repeat_key == o.repeat_key;
            }
        };
        struct ContRepeatGeomKeyHash
        {
            size_t operator()(const ContRepeatGeomKey& k) const
            {
                return std::hash<uint64_t>{}(k.repeat_key);
            }
        };
        struct ContRepeatGeomAccum
        {
            std::vector<roadmanager::RMExpandedObject> segments;
            id_t                                       source_object_id = 0;
            std::string                                node_prefix;
            osg::Vec4                                  color;
        };
        std::unordered_map<ContRepeatGeomKey, ContRepeatGeomAccum, ContRepeatGeomKeyHash> cont_repeat_geom_accum;

        auto add_markings_for_expanded = [&](const roadmanager::RMExpandedObject& exp, bool skip_cylinders)
        {
            auto dit = def_map.find(exp.sourceObjectId);
            if (dit == def_map.end() || dit->second == nullptr)
            {
                return;
            }
            const roadmanager::RMObjectDefinition* def = dit->second;
            if (def->markings.empty())
            {
                return;
            }

            const bool use_outline_edges = exp.kind == roadmanager::RMExpandedObject::Kind::OUTLINE_OBJECT;
            const bool use_continuous_segment_sides =
                exp.kind == roadmanager::RMExpandedObject::Kind::CONTINUOUS_REPEAT_SEGMENT && exp.has_world_corners;
            const bool use_continuous_segment_outline_refs = use_continuous_segment_sides && !def->outlines.empty();
            if (skip_cylinders && exp.radius.has_value() && !use_outline_edges)
            {
                return;
            }

            roadmanager::Outline*                   outline     = nullptr;
            const roadmanager::RMOutlineDefinition* outline_def = nullptr;
            if (use_outline_edges)
            {
                outline = BuildExpandedOutlineFromDefinition(*def, exp, exp.outlineIndex, road->GetId());
                if (exp.outlineIndex >= 0 && exp.outlineIndex < static_cast<int>(def->outlines.size()))
                {
                    outline_def = &def->outlines[exp.outlineIndex];
                }
            }
            else if (use_continuous_segment_outline_refs)
            {
                outline_def = &def->outlines[0];
            }

            for (size_t mi = 0; mi < def->markings.size(); ++mi)
            {
                const auto&     marking = def->markings[mi];
                const osg::Vec4 color   = ResolveMarkingColor(marking, osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
                const double    zofs    = marking.zOffset.value_or(0.0);

                // For non-outline bounding-box objects, side/edgeRef markings are handled as
                // ordered edge polylines so connected edges share a mitered corner.
                if (!use_outline_edges && !use_continuous_segment_sides && marking.cornerReferences.empty() &&
                    (!marking.sides.empty() || !marking.edgeReferences.empty()))
                {
                    auto same_optional_double = [](const std::optional<double>& a, const std::optional<double>& b) -> bool
                    {
                        if (a.has_value() != b.has_value())
                        {
                            return false;
                        }
                        if (!a.has_value())
                        {
                            return true;
                        }
                        return std::fabs(a.value() - b.value()) < SMALL_NUMBER;
                    };

                    auto same_color = [](const osg::Vec4& a, const osg::Vec4& b) -> bool
                    {
                        return std::fabs(a.r() - b.r()) < SMALL_NUMBER && std::fabs(a.g() - b.g()) < SMALL_NUMBER &&
                               std::fabs(a.b() - b.b()) < SMALL_NUMBER && std::fabs(a.a() - b.a()) < SMALL_NUMBER;
                    };

                    auto markings_are_chain_compatible = [&](const roadmanager::RMObjectMarkingDefinition& a,
                                                             const roadmanager::RMObjectMarkingDefinition& b,
                                                             const osg::Vec4&                              a_color,
                                                             const osg::Vec4&                              b_color) -> bool
                    {
                        return same_color(a_color, b_color) && same_optional_double(a.width, b.width) &&
                               same_optional_double(a.lineLength, b.lineLength) && same_optional_double(a.spaceLength, b.spaceLength) &&
                               same_optional_double(a.length, b.length) && same_optional_double(a.startOffset, b.startOffset) &&
                               same_optional_double(a.stopOffset, b.stopOffset) && same_optional_double(a.zOffset, b.zOffset) &&
                               a.placementMode == b.placementMode;
                    };

                    const double w = exp.width.has_value() ? std::max(0.05, exp.width.value()) : 1.0;
                    const double l = exp.length.has_value() ? std::max(0.05, exp.length.value()) : 1.0;

                    pos.SetTrackPosMode(road->GetId(),
                                        exp.s,
                                        exp.t,
                                        roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL |
                                            roadmanager::Position::PosMode::P_REL | roadmanager::Position::PosMode::R_REL);

                    auto local_to_world = [&](double lx, double ly) -> osg::Vec3d
                    {
                        double rx = 0.0;
                        double ry = 0.0;
                        RotateVec2D(lx, ly, pos.GetH() + exp.hdg, rx, ry);
                        return osg::Vec3d(pos.GetX() - origin[0] + rx,
                                          pos.GetY() - origin[1] + ry,
                                          pos.GetZ() + exp.zOffset + zofs + ROADMARK_Z_OFFSET);
                    };

                    // Corner order: 0=front-right, 1=front-left, 2=rear-left, 3=rear-right
                    const std::array<osg::Vec3d, 4> box_corners = {
                        local_to_world(0.5 * l, -0.5 * w),
                        local_to_world(0.5 * l, 0.5 * w),
                        local_to_world(-0.5 * l, 0.5 * w),
                        local_to_world(-0.5 * l, -0.5 * w),
                    };

                    auto build_box_polyline_for_edge = [&](int edge_idx) -> std::vector<osg::Vec3d>
                    {
                        if (edge_idx == 0)
                        {
                            return {box_corners[0], box_corners[1]};
                        }
                        if (edge_idx == 1)
                        {
                            return {box_corners[1], box_corners[2]};
                        }
                        if (edge_idx == 2)
                        {
                            return {box_corners[2], box_corners[3]};
                        }
                        if (edge_idx == 3)
                        {
                            return {box_corners[3], box_corners[0]};
                        }
                        return {};
                    };

                    auto emit_box_chain = [&](const std::vector<osg::Vec3d>& pts)
                    {
                        if (pts.size() < 2)
                        {
                            return;
                        }
                        osg::ref_ptr<osg::Group> mg = this->CreateObjectMarkingsGeomFromPolyline(pts, marking, color, false);
                        if (mg == nullptr)
                        {
                            return;
                        }
                        SetNodeName(*mg, prefix_road_object, exp.sourceObjectId, "expanded_marking");
                        osg::ref_ptr<osg::LOD> lod = new osg::LOD();
                        lod->addChild(mg);
                        lod->setRange(0, 0.0f, static_cast<float>(LOD_DIST_ROAD_FEATURES));
                        parent->addChild(lod);
                    };

                    auto dist2_xy = [](const osg::Vec3d& a, const osg::Vec3d& b) -> double
                    {
                        const double dx = a.x() - b.x();
                        const double dy = a.y() - b.y();
                        return dx * dx + dy * dy;
                    };

                    auto reverse_points = [](const std::vector<osg::Vec3d>& in) -> std::vector<osg::Vec3d>
                    { return std::vector<osg::Vec3d>(in.rbegin(), in.rend()); };

                    auto append_if_connected = [&](std::vector<osg::Vec3d>& chain, const std::vector<osg::Vec3d>& next_pts) -> bool
                    {
                        if (chain.empty() || next_pts.size() < 2)
                        {
                            return false;
                        }

                        const double eps2 = SMALL_NUMBER * SMALL_NUMBER;
                        if (dist2_xy(chain.back(), next_pts.front()) <= eps2)
                        {
                            chain.insert(chain.end(), next_pts.begin() + 1, next_pts.end());
                            return true;
                        }
                        if (dist2_xy(chain.back(), next_pts.back()) <= eps2)
                        {
                            for (auto it = next_pts.rbegin() + 1; it != next_pts.rend(); ++it)
                            {
                                chain.push_back(*it);
                            }
                            return true;
                        }
                        return false;
                    };

                    auto is_opposite_edge_pair = [](int a, int b) -> bool
                    {
                        const bool front_rear = (a == 0 && b == 2) || (a == 2 && b == 0);
                        const bool left_right = (a == 1 && b == 3) || (a == 3 && b == 1);
                        return front_rear || left_right;
                    };

                    auto side_to_edge_index = [](roadmanager::RMObjectMarkingDefinition::Side side) -> int
                    {
                        using Side = roadmanager::RMObjectMarkingDefinition::Side;
                        if (side == Side::FRONT)
                        {
                            return 0;
                        }
                        if (side == Side::LEFT)
                        {
                            return 1;
                        }
                        if (side == Side::REAR)
                        {
                            return 2;
                        }
                        if (side == Side::RIGHT)
                        {
                            return 3;
                        }
                        return -1;
                    };

                    std::vector<int>                              ordered_edges;
                    const roadmanager::RMObjectMarkingDefinition* chain_marking = &marking;
                    const osg::Vec4                               chain_color   = color;

                    for (size_t mj = mi; mj < def->markings.size(); ++mj)
                    {
                        const auto&     m2       = def->markings[mj];
                        const osg::Vec4 m2_color = ResolveMarkingColor(m2, osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));

                        // Chain only consecutive compatible side/edge-only markings.
                        if (m2.cornerReferences.size() >= 2 || (m2.sides.empty() && m2.edgeReferences.empty()) ||
                            !markings_are_chain_compatible(*chain_marking, m2, chain_color, m2_color))
                        {
                            break;
                        }

                        if (!m2.sides.empty())
                        {
                            for (const auto& side : m2.sides)
                            {
                                const int ei = side_to_edge_index(side);
                                if (ei >= 0)
                                {
                                    ordered_edges.push_back(ei);
                                }
                            }
                        }
                        else
                        {
                            for (int ref : m2.edgeReferences)
                            {
                                for (int ei = 0; ei < 4; ++ei)
                                {
                                    if (EdgeReferenceMatches(ref, static_cast<size_t>(ei), 4))
                                    {
                                        ordered_edges.push_back(ei);
                                        break;
                                    }
                                }
                            }
                        }

                        // Consume the marking entry; the outer loop continues at the first non-merged one.
                        mi = mj;
                    }

                    std::vector<osg::Vec3d> active_chain;
                    int                     prev_edge = -1;
                    for (int edge_idx : ordered_edges)
                    {
                        if (!active_chain.empty() && prev_edge >= 0 && is_opposite_edge_pair(prev_edge, edge_idx))
                        {
                            emit_box_chain(active_chain);
                            active_chain.clear();
                        }

                        std::vector<osg::Vec3d> pts = build_box_polyline_for_edge(edge_idx);
                        if (pts.size() < 2)
                        {
                            prev_edge = edge_idx;
                            continue;
                        }

                        if (active_chain.empty())
                        {
                            active_chain = pts;
                            prev_edge    = edge_idx;
                            continue;
                        }

                        if (!append_if_connected(active_chain, pts))
                        {
                            const std::vector<osg::Vec3d> pts_rev = reverse_points(pts);
                            if (!append_if_connected(active_chain, pts_rev))
                            {
                                emit_box_chain(active_chain);
                                active_chain = pts;
                            }
                        }
                        prev_edge = edge_idx;
                    }

                    if (!active_chain.empty())
                    {
                        emit_box_chain(active_chain);
                    }
                    continue;
                }

                // For continuous repeat segments with side/edgeRef markings, accumulate corner data
                // so we can stitch all segments into one polyline per side after the main loop.
                if (use_continuous_segment_sides && marking.cornerReferences.empty() && (!marking.sides.empty() || !marking.edgeReferences.empty()) &&
                    exp.segmentIndex >= 0)
                {
                    pos.SetTrackPosMode(road->GetId(),
                                        exp.s,
                                        0.0,
                                        roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL |
                                            roadmanager::Position::PosMode::P_REL | roadmanager::Position::PosMode::R_REL);
                    const double seg_z0 = pos.GetZ() + exp.zOffset + zofs + ROADMARK_Z_OFFSET;

                    pos.SetTrackPosMode(road->GetId(),
                                        exp.sEnd,
                                        0.0,
                                        roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL |
                                            roadmanager::Position::PosMode::P_REL | roadmanager::Position::PosMode::R_REL);
                    const double seg_z1 = pos.GetZ() + exp.zOffsetEnd + zofs + ROADMARK_Z_OFFSET;

                    const std::array<osg::Vec3d, 4> seg_c = {
                        osg::Vec3d(exp.world_corners[0].x - origin[0], exp.world_corners[0].y - origin[1], seg_z0),
                        osg::Vec3d(exp.world_corners[1].x - origin[0], exp.world_corners[1].y - origin[1], seg_z0),
                        osg::Vec3d(exp.world_corners[2].x - origin[0], exp.world_corners[2].y - origin[1], seg_z1),
                        osg::Vec3d(exp.world_corners[3].x - origin[0], exp.world_corners[3].y - origin[1], seg_z1),
                    };

                    const ContRepeatSideMarkKey accum_key{make_repeat_key(exp.sourceObjectId, exp.repeatIndex), mi};
                    ContRepeatSideMarkAccum&    accum_entry = cont_side_mark_accum[accum_key];
                    if (accum_entry.marking_def == nullptr)
                    {
                        accum_entry.marking_def      = &marking;
                        accum_entry.color            = color;
                        accum_entry.source_object_id = exp.sourceObjectId;
                    }
                    accum_entry.seg_corners.emplace_back(exp.segmentIndex, seg_c);
                    continue;  // deferred: rendered after the main loop as a stitched polyline
                }

                osg::ref_ptr<osg::Group> marking_group;
                if ((use_outline_edges || use_continuous_segment_outline_refs) && marking.cornerReferences.size() >= 2)
                {
                    std::vector<osg::Vec3d> points =
                        use_outline_edges ? build_outline_marking_corner_points(outline, outline_def, marking, zofs) : std::vector<osg::Vec3d>{};
                    if (use_continuous_segment_outline_refs)
                    {
                        std::vector<MarkingEdge> edges = build_continuous_segment_marking_edges(exp, outline_def, marking, zofs);
                        points.reserve(edges.size() + 1);
                        if (!edges.empty())
                        {
                            points.push_back(edges.front().a);
                            for (const auto& e : edges)
                            {
                                points.push_back(e.b);
                            }
                        }
                    }

                    const bool closed_loop = points.size() >= 3 && std::fabs(points.front().x() - points.back().x()) < SMALL_NUMBER &&
                                             std::fabs(points.front().y() - points.back().y()) < SMALL_NUMBER;
                    marking_group = this->CreateObjectMarkingsGeomFromPolyline(points, marking, color, closed_loop);
                }
                else
                {
                    std::vector<MarkingEdge> edges = use_outline_edges ? build_outline_marking_edges(outline, outline_def, marking, zofs)
                                                     : use_continuous_segment_outline_refs
                                                         ? build_continuous_segment_marking_edges(exp, outline_def, marking, zofs)
                                                         : build_box_marking_edges(exp, marking, zofs);
                    marking_group                  = this->CreateObjectMarkingsGeom(edges, marking, color);
                }
                if (marking_group != nullptr)
                {
                    SetNodeName(*marking_group, prefix_road_object, exp.sourceObjectId, "expanded_marking");
                    osg::ref_ptr<osg::LOD> lod = new osg::LOD();
                    lod->addChild(marking_group);
                    lod->setRange(0, 0.0f, static_cast<float>(LOD_DIST_ROAD_FEATURES));
                    parent->addChild(lod);
                }
            }

            if (outline != nullptr)
            {
                delete outline;
            }
        };

        std::unordered_set<id_t> expanded_source_ids;
        expanded_source_ids.reserve(expanded.size());
        for (const auto& exp : expanded)
        {
            expanded_source_ids.insert(exp.sourceObjectId);
        }

        for (const auto& exp : expanded)
        {
            const std::string& node_prefix = get_object_node_prefix(exp.sourceObjectId);
            osg::Vec4          color(0.8f, 0.8f, 0.8f, 1.0f);
            auto               cit = color_map.find(exp.sourceObjectId);
            if (cit != color_map.end())
            {
                color = cit->second;
            }

            bool has_object_markings = false;
            auto dit_markings        = def_map.find(exp.sourceObjectId);
            if (dit_markings != def_map.end() && dit_markings->second != nullptr)
            {
                has_object_markings = !dit_markings->second->markings.empty();
            }

            const double height = exp.height.has_value() ? std::max(0.05, exp.height.value()) : 1.0;

            if (exp.kind != roadmanager::RMExpandedObject::Kind::CONTINUOUS_REPEAT_SEGMENT)
            {
                ExpandedRoadObjectModel& model = get_model_for_source(exp.sourceObjectId);
                if (model.tx != nullptr)
                {
                    // Outline expansion emits one entry per outline; when a model exists we only want one visual per anchor.
                    if (exp.kind == roadmanager::RMExpandedObject::Kind::OUTLINE_OBJECT && exp.outlineIndex > 0)
                    {
                        continue;
                    }

                    pos.SetTrackPosMode(road->GetId(),
                                        exp.s,
                                        exp.t,
                                        roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL |
                                            roadmanager::Position::PosMode::P_REL | roadmanager::Position::PosMode::R_REL);

                    osg::ref_ptr<osg::PositionAttitudeTransform> clone =
                        dynamic_cast<osg::PositionAttitudeTransform*>(model.tx->clone(osg::CopyOp::DEEP_COPY_ALL));
                    if (clone != nullptr)
                    {
                        const double scale_x = (model.dim_x > SMALL_NUMBER && exp.length.has_value() && exp.length.value() > SMALL_NUMBER)
                                                   ? exp.length.value() / model.dim_x
                                                   : 1.0;
                        const double scale_y = (model.dim_y > SMALL_NUMBER && exp.width.has_value() && exp.width.value() > SMALL_NUMBER)
                                                   ? exp.width.value() / model.dim_y
                                                   : 1.0;
                        const double scale_z = (model.dim_z > SMALL_NUMBER && exp.height.has_value() && exp.height.value() > SMALL_NUMBER)
                                                   ? exp.height.value() / model.dim_z
                                                   : 1.0;
                        const double lod_x   = model.dim_x * scale_x;
                        const double lod_y   = model.dim_y * scale_y;

                        clone->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
                        clone->setScale(osg::Vec3(static_cast<float>(scale_x), static_cast<float>(scale_y), static_cast<float>(scale_z)));
                        clone->setPosition(osg::Vec3(static_cast<float>(pos.GetX() - origin[0]),
                                                     static_cast<float>(pos.GetY() - origin[1]),
                                                     static_cast<float>(pos.GetZ() + exp.zOffset)));
                        osg::Quat quatRoad(pos.GetR(), osg::X_AXIS, pos.GetP(), osg::Y_AXIS, pos.GetH() + exp.hdg, osg::Z_AXIS);
                        clone->setAttitude(quatRoad);
                        clone->setDataVariance(osg::Object::STATIC);
                        SetNodeName(*clone, node_prefix, exp.sourceObjectId, "expanded_model");

                        osg::ref_ptr<osg::LOD> lod = new osg::LOD();
                        lod->addChild(clone);
                        lod->setRange(0, 0.0f, static_cast<float>(LOD_DIST_ROAD_FEATURES + MAX(lod_x, lod_y)));
                        parent->addChild(lod);
                        add_markings_for_expanded(exp, false);
                        continue;
                    }
                }
            }

            if (has_object_markings)
            {
                // If markings are defined for this object definition, suppress primitive fill geometry
                // and render only the markings for this expanded instance.
                add_markings_for_expanded(exp, true);
                continue;
            }

            if (exp.kind == roadmanager::RMExpandedObject::Kind::CONTINUOUS_REPEAT_SEGMENT && exp.has_world_corners)
            {
                const ContRepeatGeomKey key{make_repeat_key(exp.sourceObjectId, exp.repeatIndex)};
                ContRepeatGeomAccum&    accum = cont_repeat_geom_accum[key];
                if (accum.segments.empty())
                {
                    accum.source_object_id = exp.sourceObjectId;
                    accum.node_prefix      = node_prefix;
                    accum.color            = color;
                }
                accum.segments.push_back(exp);
            }
            else if (exp.kind == roadmanager::RMExpandedObject::Kind::OUTLINE_OBJECT)
            {
                auto dit = def_map.find(exp.sourceObjectId);
                if (dit != def_map.end())
                {
                    roadmanager::Outline* outline = BuildExpandedOutlineFromDefinition(*dit->second, exp, exp.outlineIndex, road->GetId());
                    if (outline != nullptr)
                    {
                        osg::ref_ptr<osg::Group> ol_group = CreateOutlineObject(outline, color, origin);
                        delete outline;

                        if (ol_group != nullptr)
                        {
                            SetNodeName(*ol_group, node_prefix, exp.sourceObjectId, "expanded_outline_" + std::to_string(exp.outlineIndex));

                            osg::ComputeBoundsVisitor cbv;
                            ol_group->accept(cbv);
                            const osg::BoundingBox& bb = cbv.getBoundingBox();

                            osg::ref_ptr<osg::LOD> lod = new osg::LOD();
                            lod->addChild(ol_group);
                            lod->setRange(0, 0.0f, static_cast<float>(LOD_DIST_ROAD_FEATURES + MAX(bb.xMax() - bb.xMin(), bb.yMax() - bb.yMin())));
                            parent->addChild(lod);
                            add_markings_for_expanded(exp, false);
                        }
                    }
                }
                else
                {
                    // Fallback for legacy-only objects when no object definition is available.
                    auto oit = object_map.find(exp.sourceObjectId);
                    if (oit != object_map.end())
                    {
                        roadmanager::RMObject* src_obj    = oit->second;
                        const unsigned int     n_outlines = src_obj->GetNumberOfOutlines();
                        const unsigned int     oi         = static_cast<unsigned int>(exp.outlineIndex);
                        if (oi < n_outlines)
                        {
                            roadmanager::Outline*    outline  = src_obj->GetOutline(oi);
                            osg::ref_ptr<osg::Group> ol_group = CreateOutlineObject(outline, color, origin);
                            if (ol_group != nullptr)
                            {
                                SetNodeName(*ol_group, node_prefix, exp.sourceObjectId, "expanded_outline_" + std::to_string(exp.outlineIndex));

                                osg::ComputeBoundsVisitor cbv;
                                ol_group->accept(cbv);
                                const osg::BoundingBox& bb = cbv.getBoundingBox();

                                osg::ref_ptr<osg::LOD> lod = new osg::LOD();
                                lod->addChild(ol_group);
                                lod->setRange(0,
                                              0.0f,
                                              static_cast<float>(LOD_DIST_ROAD_FEATURES + MAX(bb.xMax() - bb.xMin(), bb.yMax() - bb.yMin())));
                                parent->addChild(lod);
                                add_markings_for_expanded(exp, false);
                            }
                        }
                    }
                }
            }
            else
            {
                // Single anchor or discrete repeat: render as cylinder if radius present, otherwise bounding-box.
                pos.SetTrackPosMode(road->GetId(),
                                    exp.s,
                                    exp.t,
                                    roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL |
                                        roadmanager::Position::PosMode::P_REL | roadmanager::Position::PosMode::R_REL);

                osg::ref_ptr<osg::PositionAttitudeTransform> tx = new osg::PositionAttitudeTransform;
                osg::ref_ptr<osg::ShapeDrawable>             shape;

                if (exp.radius.has_value())
                {
                    // Circular object: render as cylinder
                    osg::ref_ptr<osg::TessellationHints> th = new osg::TessellationHints();
                    th->setDetailRatio(0.5f);
                    const float radius = static_cast<float>(std::max(0.05, exp.radius.value()));
                    shape              = new osg::ShapeDrawable(
                        new osg::Cylinder(osg::Vec3(0.0f, 0.0f, 0.5f * static_cast<float>(height)), radius, static_cast<float>(height)),
                        th);
                    SetNodeName(*tx, node_prefix, exp.sourceObjectId, "expanded_cylinder");
                }
                else
                {
                    // Angular object: render as box
                    const double w = exp.width.has_value() ? std::max(0.05, exp.width.value()) : 1.0;
                    const double l = exp.length.has_value() ? std::max(0.05, exp.length.value()) : 1.0;
                    shape          = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0f, 0.0f, 0.5f * static_cast<float>(height)),
                                                                static_cast<float>(l),
                                                                static_cast<float>(w),
                                                                static_cast<float>(height)));
                    SetNodeName(*tx, node_prefix, exp.sourceObjectId, "expanded_bb");
                }

                shape->setColor(color);
                tx->addChild(shape);
                tx->setPosition(osg::Vec3(static_cast<float>(pos.GetX() - origin[0]),
                                          static_cast<float>(pos.GetY() - origin[1]),
                                          static_cast<float>(pos.GetZ() + exp.zOffset)));
                osg::Quat quatRoad(pos.GetR(), osg::X_AXIS, pos.GetP(), osg::Y_AXIS, pos.GetH() + exp.hdg, osg::Z_AXIS);
                tx->setAttitude(quatRoad);
                tx->setDataVariance(osg::Object::STATIC);

                osg::ref_ptr<osg::LOD> lod = new osg::LOD();
                lod->addChild(tx);
                lod->setRange(0, 0.0f, static_cast<float>(LOD_DIST_ROAD_FEATURES));
                parent->addChild(lod);
                add_markings_for_expanded(exp, true);
            }
        }

        // Flush accumulated continuous-repeat solid geometry as one connected mesh per repeat.
        // This enables smoothing over the full wall/top/bottom surfaces instead of per-segment.
        for (auto& [gk, gacc] : cont_repeat_geom_accum)
        {
            if (gacc.segments.empty())
            {
                continue;
            }

            std::sort(gacc.segments.begin(),
                      gacc.segments.end(),
                      [](const roadmanager::RMExpandedObject& a, const roadmanager::RMExpandedObject& b) { return a.segmentIndex < b.segmentIndex; });

            std::vector<osg::Vec3> right_bottom;
            std::vector<osg::Vec3> right_top;
            std::vector<osg::Vec3> left_bottom;
            std::vector<osg::Vec3> left_top;
            right_bottom.reserve(gacc.segments.size() + 1);
            right_top.reserve(gacc.segments.size() + 1);
            left_bottom.reserve(gacc.segments.size() + 1);
            left_top.reserve(gacc.segments.size() + 1);

            bool emit_start_cap = true;
            bool emit_end_cap   = true;

            for (size_t si = 0; si < gacc.segments.size(); ++si)
            {
                const roadmanager::RMExpandedObject& seg = gacc.segments[si];

                // Base Z at segment start and end (centerline t=0), then apply per-segment z offsets.
                pos.SetTrackPosMode(road->GetId(),
                                    seg.s,
                                    0.0,
                                    roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL |
                                        roadmanager::Position::PosMode::P_REL | roadmanager::Position::PosMode::R_REL);
                const float b0 = static_cast<float>(pos.GetZ() + seg.zOffset);

                pos.SetTrackPosMode(road->GetId(),
                                    seg.sEnd,
                                    0.0,
                                    roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL |
                                        roadmanager::Position::PosMode::P_REL | roadmanager::Position::PosMode::R_REL);
                const float b1 = static_cast<float>(pos.GetZ() + seg.zOffsetEnd);

                const float t0 = b0 + static_cast<float>(seg.height.has_value() ? std::max(0.05, seg.height.value()) : 1.0);
                const float t1 =
                    b1 + static_cast<float>(seg.heightEnd.has_value() ? std::max(0.05, seg.heightEnd.value())
                                                                      : (seg.height.has_value() ? std::max(0.05, seg.height.value()) : 1.0));

                const osg::Vec3 c0(static_cast<float>(seg.world_corners[0].x - origin[0]),
                                   static_cast<float>(seg.world_corners[0].y - origin[1]),
                                   b0);
                const osg::Vec3 c1(static_cast<float>(seg.world_corners[1].x - origin[0]),
                                   static_cast<float>(seg.world_corners[1].y - origin[1]),
                                   b0);
                const osg::Vec3 c2(static_cast<float>(seg.world_corners[2].x - origin[0]),
                                   static_cast<float>(seg.world_corners[2].y - origin[1]),
                                   b1);
                const osg::Vec3 c3(static_cast<float>(seg.world_corners[3].x - origin[0]),
                                   static_cast<float>(seg.world_corners[3].y - origin[1]),
                                   b1);

                const osg::Vec3 c0t(c0.x(), c0.y(), t0);
                const osg::Vec3 c1t(c1.x(), c1.y(), t0);
                const osg::Vec3 c2t(c2.x(), c2.y(), t1);
                const osg::Vec3 c3t(c3.x(), c3.y(), t1);

                if (si == 0)
                {
                    right_bottom.push_back(c0);
                    right_top.push_back(c0t);
                    left_bottom.push_back(c1);
                    left_top.push_back(c1t);

                    if (seg.segmentIndex >= 0)
                    {
                        auto itf = first_segment_index_by_repeat.find(gk.repeat_key);
                        if (itf != first_segment_index_by_repeat.end())
                        {
                            emit_start_cap = (seg.segmentIndex == itf->second);
                        }
                    }
                }

                right_bottom.push_back(c3);
                right_top.push_back(c3t);
                left_bottom.push_back(c2);
                left_top.push_back(c2t);

                if (si + 1 == gacc.segments.size() && seg.segmentIndex >= 0)
                {
                    auto itl = last_segment_index_by_repeat.find(gk.repeat_key);
                    if (itl != last_segment_index_by_repeat.end())
                    {
                        emit_end_cap = (seg.segmentIndex == itl->second);
                    }
                }
            }

            if (right_bottom.size() < 2 || left_bottom.size() < 2)
            {
                continue;
            }

            osg::ref_ptr<osg::Vec4Array> color_arr = new osg::Vec4Array(1);
            (*color_arr)[0]                        = gacc.color;

            auto make_geom_from_strip = [&](const std::vector<osg::Vec3>& a, const std::vector<osg::Vec3>& b) -> osg::ref_ptr<osg::Geometry>
            {
                if (a.size() != b.size() || a.size() < 2)
                {
                    return nullptr;
                }

                osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array;
                verts->reserve(2 * a.size());
                for (size_t i = 0; i < a.size(); ++i)
                {
                    verts->push_back(a[i]);
                    verts->push_back(b[i]);
                }

                osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
                geom->setVertexArray(verts);
                geom->addPrimitiveSet(new osg::DrawArrays(GL_QUAD_STRIP, 0, static_cast<GLsizei>(verts->size())));
                geom->setColorArray(color_arr);
                geom->setColorBinding(osg::Geometry::BIND_OVERALL);
                osgUtil::SmoothingVisitor::smooth(*geom, 0.5);
                geom->setDataVariance(osg::Object::STATIC);
                geom->setUseDisplayList(true);
                return geom;
            };

            auto make_geom_from_quad =
                [&](const osg::Vec3& p0, const osg::Vec3& p1, const osg::Vec3& p2, const osg::Vec3& p3) -> osg::ref_ptr<osg::Geometry>
            {
                osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array(4);
                (*verts)[0]                        = p0;
                (*verts)[1]                        = p1;
                (*verts)[2]                        = p2;
                (*verts)[3]                        = p3;

                osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
                geom->setVertexArray(verts);
                geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));
                geom->setColorArray(color_arr);
                geom->setColorBinding(osg::Geometry::BIND_OVERALL);
                osgUtil::SmoothingVisitor::smooth(*geom, 0.5);
                geom->setDataVariance(osg::Object::STATIC);
                geom->setUseDisplayList(true);
                return geom;
            };

            // Flip strip pair ordering to keep all surfaces CCW when viewed from the outside.
            osg::ref_ptr<osg::Geometry> geom_right  = make_geom_from_strip(right_top, right_bottom);
            osg::ref_ptr<osg::Geometry> geom_left   = make_geom_from_strip(left_bottom, left_top);
            osg::ref_ptr<osg::Geometry> geom_top    = make_geom_from_strip(left_top, right_top);
            osg::ref_ptr<osg::Geometry> geom_bottom = make_geom_from_strip(right_bottom, left_bottom);

            osg::ref_ptr<osg::Geometry> geom_start_cap;
            osg::ref_ptr<osg::Geometry> geom_end_cap;
            if (emit_start_cap)
            {
                // Reverse winding of start cap.
                geom_start_cap = make_geom_from_quad(left_bottom.front(), right_bottom.front(), right_top.front(), left_top.front());
            }
            if (emit_end_cap)
            {
                // Reverse winding of end cap.
                geom_end_cap = make_geom_from_quad(right_bottom.back(), left_bottom.back(), left_top.back(), right_top.back());
            }

            osg::ref_ptr<osg::Geode> geode = new osg::Geode;
            if (geom_right != nullptr)
            {
                geode->addDrawable(geom_right);
            }
            if (geom_left != nullptr)
            {
                geode->addDrawable(geom_left);
            }
            if (geom_top != nullptr)
            {
                geode->addDrawable(geom_top);
            }
            if (geom_bottom != nullptr)
            {
                geode->addDrawable(geom_bottom);
            }
            if (geom_start_cap != nullptr)
            {
                geode->addDrawable(geom_start_cap);
            }
            if (geom_end_cap != nullptr)
            {
                geode->addDrawable(geom_end_cap);
            }

            osg::ref_ptr<osg::Material> mat = new osg::Material;
            mat->setDiffuse(osg::Material::FRONT_AND_BACK, gacc.color);
            mat->setAmbient(osg::Material::FRONT_AND_BACK, gacc.color);
            geode->getOrCreateStateSet()->setAttributeAndModes(mat.get());
            geode->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);

            osg::ref_ptr<osg::Group> repeat_group = new osg::Group;
            repeat_group->addChild(geode);
            SetNodeName(*repeat_group, gacc.node_prefix, gacc.source_object_id, "expanded_repeat");

            osg::ComputeBoundsVisitor cbv;
            repeat_group->accept(cbv);
            const osg::BoundingBox& bb = cbv.getBoundingBox();

            osg::ref_ptr<osg::LOD> lod = new osg::LOD();
            lod->addChild(repeat_group);
            lod->setRange(0, 0.0f, static_cast<float>(LOD_DIST_ROAD_FEATURES + MAX(bb.xMax() - bb.xMin(), bb.yMax() - bb.yMin())));
            parent->addChild(lod);
        }

        // Flush accumulated continuous-repeat side/edgeRef markings as stitched polylines.
        // Each entry covers one (repeat, marking) pair; corners are sorted by segment index
        // and chained into per-side polylines so mitered joints and dash phase are preserved.
        for (auto& [accum_key, accum_entry] : cont_side_mark_accum)
        {
            if (accum_entry.seg_corners.empty() || accum_entry.marking_def == nullptr)
            {
                continue;
            }

            std::sort(accum_entry.seg_corners.begin(), accum_entry.seg_corners.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

            const roadmanager::RMObjectMarkingDefinition& m    = *accum_entry.marking_def;
            const osg::Vec4&                              mc   = accum_entry.color;
            const auto&                                   segs = accum_entry.seg_corners;

            auto emit_stitched = [&](const std::vector<osg::Vec3d>& pts)
            {
                if (pts.size() < 2)
                {
                    return;
                }
                osg::ref_ptr<osg::Group> mg = this->CreateObjectMarkingsGeomFromPolyline(pts, m, mc, false);
                if (mg == nullptr)
                {
                    return;
                }
                SetNodeName(*mg, prefix_road_object, accum_entry.source_object_id, "expanded_marking");
                osg::ref_ptr<osg::LOD> lod = new osg::LOD();
                lod->addChild(mg);
                lod->setRange(0, 0.0f, static_cast<float>(LOD_DIST_ROAD_FEATURES));
                parent->addChild(lod);
            };

            // Build the polyline for a given quad-edge index:
            //   edge 0 (FRONT)  — first segment's c0→c1  (transverse start cap)
            //   edge 1 (LEFT)   — c1→c2 chained forward across all segments
            //   edge 2 (REAR)   — last  segment's c2→c3 (transverse end cap)
            //   edge 3 (RIGHT)  — c0→c3 chained forward across all segments (canonical forward direction)
            auto build_polyline_for_edge = [&](int edge_idx) -> std::vector<osg::Vec3d>
            {
                std::vector<osg::Vec3d> pts;
                if (edge_idx == 1)  // LEFT: start-left → end-left per segment
                {
                    for (size_t i = 0; i < segs.size(); ++i)
                    {
                        if (i == 0)
                        {
                            pts.push_back(segs[i].second[1]);
                        }
                        pts.push_back(segs[i].second[2]);
                    }
                }
                else if (edge_idx == 3)  // RIGHT: start-right → end-right per segment (forward direction)
                {
                    for (size_t i = 0; i < segs.size(); ++i)
                    {
                        if (i == 0)
                        {
                            pts.push_back(segs[i].second[0]);
                        }
                        pts.push_back(segs[i].second[3]);
                    }
                }
                else if (edge_idx == 0 && !segs.empty())  // FRONT: first segment's start edge
                {
                    pts = {segs.front().second[0], segs.front().second[1]};
                }
                else if (edge_idx == 2 && !segs.empty())  // REAR: last segment's end edge
                {
                    pts = {segs.back().second[2], segs.back().second[3]};
                }
                return pts;
            };

            // Emit ordered side chains.
            // Consecutive non-opposite sides are stitched into one continuous polyline
            // (e.g. left + front), while opposite consecutive sides split into separate
            // markings (e.g. left + right).
            auto side_to_edge_index = [](roadmanager::RMObjectMarkingDefinition::Side side) -> int
            {
                using Side = roadmanager::RMObjectMarkingDefinition::Side;
                if (side == Side::FRONT)
                {
                    return 2;
                }
                if (side == Side::LEFT)
                {
                    return 1;
                }
                if (side == Side::REAR)
                {
                    return 0;
                }
                if (side == Side::RIGHT)
                {
                    return 3;
                }
                return -1;
            };

            auto is_opposite_edge_pair = [](int a, int b) -> bool
            {
                const bool front_rear = (a == 0 && b == 2) || (a == 2 && b == 0);
                const bool left_right = (a == 1 && b == 3) || (a == 3 && b == 1);
                return front_rear || left_right;
            };

            auto dist2_xy = [](const osg::Vec3d& a, const osg::Vec3d& b) -> double
            {
                const double dx = a.x() - b.x();
                const double dy = a.y() - b.y();
                return dx * dx + dy * dy;
            };

            auto reverse_points = [](const std::vector<osg::Vec3d>& in) -> std::vector<osg::Vec3d>
            {
                std::vector<osg::Vec3d> out(in.rbegin(), in.rend());
                return out;
            };

            auto append_if_connected = [&](std::vector<osg::Vec3d>& chain, const std::vector<osg::Vec3d>& next_pts) -> bool
            {
                if (chain.empty() || next_pts.size() < 2)
                {
                    return false;
                }

                const double eps2 = SMALL_NUMBER * SMALL_NUMBER;
                if (dist2_xy(chain.back(), next_pts.front()) <= eps2)
                {
                    chain.insert(chain.end(), next_pts.begin() + 1, next_pts.end());
                    return true;
                }

                if (dist2_xy(chain.back(), next_pts.back()) <= eps2)
                {
                    for (auto it = next_pts.rbegin() + 1; it != next_pts.rend(); ++it)
                    {
                        chain.push_back(*it);
                    }
                    return true;
                }

                return false;
            };

            if (!m.sides.empty())
            {
                std::vector<int> side_edges;
                side_edges.reserve(m.sides.size());
                for (const auto& side : m.sides)
                {
                    const int ei = side_to_edge_index(side);
                    if (ei >= 0)
                    {
                        side_edges.push_back(ei);
                    }
                }

                std::vector<osg::Vec3d> active_chain;
                int                     prev_edge = -1;

                for (size_t i = 0; i < side_edges.size(); ++i)
                {
                    const int edge_idx = side_edges[i];

                    // Split chains when consecutive sides are opposite.
                    if (!active_chain.empty() && prev_edge >= 0 && is_opposite_edge_pair(prev_edge, edge_idx))
                    {
                        emit_stitched(active_chain);
                        active_chain.clear();
                    }

                    std::vector<osg::Vec3d> pts = build_polyline_for_edge(edge_idx);
                    if (pts.size() < 2)
                    {
                        prev_edge = edge_idx;
                        continue;
                    }

                    if (active_chain.empty())
                    {
                        // If there is a next non-opposite side, choose orientation for the first
                        // segment that best prepares a contiguous join to that next side.
                        if (i + 1 < side_edges.size() && !is_opposite_edge_pair(edge_idx, side_edges[i + 1]))
                        {
                            const std::vector<osg::Vec3d> next_pts = build_polyline_for_edge(side_edges[i + 1]);
                            if (next_pts.size() >= 2)
                            {
                                const std::vector<osg::Vec3d> pts_rev = reverse_points(pts);
                                const double fwd_best = std::min(dist2_xy(pts.back(), next_pts.front()), dist2_xy(pts.back(), next_pts.back()));
                                const double rev_best =
                                    std::min(dist2_xy(pts_rev.back(), next_pts.front()), dist2_xy(pts_rev.back(), next_pts.back()));
                                if (rev_best + SMALL_NUMBER < fwd_best)
                                {
                                    pts = pts_rev;
                                }
                            }
                        }

                        active_chain = pts;
                        prev_edge    = edge_idx;
                        continue;
                    }

                    if (!append_if_connected(active_chain, pts))
                    {
                        // Try explicit reversal once before splitting.
                        const std::vector<osg::Vec3d> pts_rev = reverse_points(pts);
                        if (!append_if_connected(active_chain, pts_rev))
                        {
                            emit_stitched(active_chain);
                            active_chain = pts;
                        }
                    }

                    prev_edge = edge_idx;
                }

                if (!active_chain.empty())
                {
                    emit_stitched(active_chain);
                }
            }

            // Emit one polyline per edge reference
            for (size_t k = 0; k < 4; ++k)
            {
                if (std::any_of(m.edgeReferences.begin(),
                                m.edgeReferences.end(),
                                [k](int ref) { return EdgeReferenceMatches(ref, static_cast<size_t>(k), 4); }))
                {
                    emit_stitched(build_polyline_for_edge(static_cast<int>(k)));
                }
            }
        }

        // Compatibility fallback: render legacy objects that have no expanded-definition representation,
        // e.g. tunnel wall/roof components generated from <tunnel> elements.
        for (unsigned int o = 0; o < road->GetNumberOfObjects(); ++o)
        {
            roadmanager::RMObject* object = road->GetRoadObject(o);
            if (object == nullptr)
            {
                continue;
            }
            if (expanded_source_ids.find(object->GetId()) != expanded_source_ids.end())
            {
                // Warn if a tunnel component collides with an object ID (which suggests a true ID collision issue)
                if (object->GetTunnelComponentType() != roadmanager::RMObject::TunnelComponentType::NO_TUNNEL)
                {
                    LOG_WARN("Road {} tunnel component id {} already exists in expanded definitions. "
                             "Check your scenario for duplicate IDs between <tunnel> and <object> elements.",
                             road->GetId(),
                             object->GetId());
                }
                continue;
            }

            std::string obj_type = prefix_road_object;
            switch (object->GetTunnelComponentType())
            {
                case roadmanager::RMObject::TunnelComponentType::TUNNEL_WALL:
                    obj_type = prefix_tunnel_wall;
                    break;
                case roadmanager::RMObject::TunnelComponentType::TUNNEL_ROOF:
                    obj_type = prefix_tunnel_roof;
                    break;
                default:
                    break;
            }

            osg::Vec4 color;
            for (unsigned int c = 0; c < 4; ++c)
            {
                color[c] = object->GetColor()[c];
            }

            osg::ref_ptr<osg::PositionAttitudeTransform> tx = nullptr;
            std::string located_file_path = LocateRoadObjectModelFile(object->GetName(), odrManager_->GetOpenDriveFilename(), exe_path);
            if (!located_file_path.empty())
            {
                tx = LoadRoadFeature(road, located_file_path);
                if (tx != nullptr)
                {
                    object->SetModel3DFullPath(located_file_path);
                }
            }

            if (tx == nullptr && object->GetNumberOfOutlines() > 0)
            {
                for (unsigned int oi = 0; oi < object->GetNumberOfOutlines(); ++oi)
                {
                    roadmanager::Outline*    outline  = object->GetOutline(oi);
                    osg::ref_ptr<osg::Group> ol_group = CreateOutlineObject(outline, color, origin);
                    if (ol_group != nullptr)
                    {
                        SetNodeName(*ol_group, obj_type, object->GetId(), object->GetName() + "_legacy_" + std::to_string(oi));

                        osg::ComputeBoundsVisitor cbv;
                        ol_group->accept(cbv);
                        const osg::BoundingBox& bb = cbv.getBoundingBox();

                        osg::ref_ptr<osg::LOD> lod = new osg::LOD();
                        lod->addChild(ol_group);
                        lod->setRange(0, 0.0f, static_cast<float>(LOD_DIST_ROAD_FEATURES + MAX(bb.xMax() - bb.xMin(), bb.yMax() - bb.yMin())));
                        parent->addChild(lod);
                    }
                }
                continue;
            }

            if (tx == nullptr)
            {
                tx = new osg::PositionAttitudeTransform;
                osg::ref_ptr<osg::ShapeDrawable> shape =
                    new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0f, 0.0f, 0.5f * MAX(0.05f, static_cast<float>(object->GetHeight()))),
                                                        MAX(0.05f, static_cast<float>(object->GetLength())),
                                                        MAX(0.05f, static_cast<float>(object->GetWidth())),
                                                        MAX(0.05f, static_cast<float>(object->GetHeight()))));
                shape->setColor(color);
                tx->addChild(shape);
            }

            SetNodeName(*tx, obj_type, object->GetId(), object->GetName() + "_legacy");
            tx->setPosition(osg::Vec3(static_cast<float>(object->GetX() - origin[0]),
                                      static_cast<float>(object->GetY() - origin[1]),
                                      static_cast<float>(object->GetZ() + object->GetZOffset())));
            osg::Quat quatRoad(object->GetRoll(), osg::X_AXIS, object->GetPitch(), osg::Y_AXIS, object->GetH() + object->GetHOffset(), osg::Z_AXIS);
            tx->setAttitude(quatRoad);
            tx->setDataVariance(osg::Object::STATIC);

            osg::ComputeBoundsVisitor cbv;
            tx->accept(cbv);
            const osg::BoundingBox& bb = cbv.getBoundingBox();

            osg::ref_ptr<osg::LOD> lod = new osg::LOD();
            lod->addChild(tx);
            lod->setRange(0, 0.0f, static_cast<float>(LOD_DIST_ROAD_FEATURES + MAX(bb.xMax() - bb.xMin(), bb.yMax() - bb.yMin())));
            parent->addChild(lod);
        }
    }

    int RoadGeom::AddGroundSurface()
    {
        const double margin   = 1E4;
        const double z_offset = -1.0;
        // const osg::BoundingSphere bs = environment_->getBound();

        osg::ComputeBoundsVisitor cbv;
        osg::BoundingBox          bb;

        osg::Node* bb_node = environment_ != nullptr ? environment_ : root_->getNumChildren() > 0 ? root_->asNode() : nullptr;
        if (bb_node != nullptr)
        {
            bb_node->accept(cbv);
            bb = cbv.getBoundingBox();
        }
        else
        {
            bb.set(osg::Vec3d(0.0, 0.0, 0.0), osg::Vec3d(1e4, 1e4, 1e4));
        }

        osg::ref_ptr<osg::Geode>    ground = new osg::Geode;
        osg::ref_ptr<osg::Geometry> geom   = osg::createTexturedQuadGeometry(
            osg::Vec3(bb.xMin() - static_cast<float>(margin), bb.yMin() - static_cast<float>(margin), bb.zMin() + static_cast<float>(z_offset)),
            osg::Vec3(0.0f, 2.0f * static_cast<float>(margin) + (bb.yMax() - bb.yMin()), bb.zMin() + static_cast<float>(z_offset)),
            osg::Vec3(2.0f * static_cast<float>(margin) + (bb.xMax() - bb.xMin()), 0.0f, bb.zMin() + static_cast<float>(z_offset)));
        osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
        color->push_back(osg::Vec4(0.8f, 0.8f, 0.8f, 1.0f));
        geom->setColorArray(color.get());
        geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
        ground->addDrawable(geom);

        root_->addChild(ground.get());

        return 0;
    }

    void RoadGeom::SetNodeName(osg::Node& node, const std::string& prefix, id_t id, const std::string& label)
    {
        node.setName(prefix + std::to_string(id) + "_" + label);
    }

    int RoadGeom::SaveToFile(const std::string& filename)
    {
        osgDB::writeNodeFile(*root_, filename);
        return 0;
    }

    TrafficLightModel* RoadGeom::GetTrafficLightModel(int id)
    {
        auto it = traffic_light_.find(id);
        if (it != traffic_light_.end())
        {
            return &it->second;
        }
        return nullptr;
    }

}  // namespace roadgeom
