/*
 * Copyright (c) Deron (Delong Zhu)
   The Chinese University of Hong Kong
   Carnegie Mellon University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "vdb_edt/vdbmap.h"
#include <cmath>
// NOTE(ROS2): Needed for tf2::doTransform on sensor_msgs::msg::PointCloud2
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/create_timer_ros.h>

#include <unordered_map>
#include <algorithm>
#include <cstdint>

VDBMap::VDBMap()
    : L_FREE(-0.13), L_OCCU(+1.01), L_THRESH(0.0), L_MIN(-2.0), L_MAX(+3.5), VOX_SIZE(0.2),
      START_RANGE(0.0), SENSOR_RANGE(5.0), HIT_THICKNESS(1), VERSION(1),
      MAX_UPDATE_DIST(20.0), EDT_UPDATE_DURATION(0.5), VIS_UPDATE_DURATION(10.0),
      VIS_MAP_MINX(-200.0), VIS_MAP_MINY(-200.0), VIS_MAP_MINZ(-1.0),
      VIS_MAP_MAXX(+200.0), VIS_MAP_MAXY(+200.0), VIS_MAP_MAXZ(10.0), VIS_SLICE_LEVEL(2.0),
      msg_ready_(false), occu_update_count_(0), dist_update_count_(0), frontier_update_count_(0)
{
    node_handle_ = std::make_shared<rclcpp::Node>("vdb_map_node");
    node_name_ = get_node_name();
    initialize();
}

VDBMap::VDBMap(const rclcpp::Node::SharedPtr &external_node)
    : L_FREE(-0.13), L_OCCU(+1.01), L_THRESH(0.0), L_MIN(-2.0), L_MAX(+3.5), VOX_SIZE(0.2),
      START_RANGE(0.0), SENSOR_RANGE(5.0), HIT_THICKNESS(1), VERSION(1),
      MAX_UPDATE_DIST(20.0), EDT_UPDATE_DURATION(0.5), VIS_UPDATE_DURATION(10.0),
      VIS_MAP_MINX(-200.0), VIS_MAP_MINY(-200.0), VIS_MAP_MINZ(-1.0),
      VIS_MAP_MAXX(+200.0), VIS_MAP_MAXY(+200.0), VIS_MAP_MAXZ(10.0), VIS_SLICE_LEVEL(2.0),
      msg_ready_(false), occu_update_count_(0), dist_update_count_(0), frontier_update_count_(0)
{
    node_handle_ = external_node;
    node_name_ = get_node_name();
    initialize();
}

VDBMap::VDBMap(const rclcpp::Node::SharedPtr &external_node,
               const std::shared_ptr<tf2_ros::Buffer> &external_tf_buffer,
               const std::shared_ptr<tf2_ros::TransformListener> &external_tf_listener)
    : L_FREE(-0.13), L_OCCU(+1.01), L_THRESH(0.0), L_MIN(-2.0), L_MAX(+3.5), VOX_SIZE(0.2),
      START_RANGE(0.0), SENSOR_RANGE(5.0), HIT_THICKNESS(1), VERSION(1),
      MAX_UPDATE_DIST(20.0), EDT_UPDATE_DURATION(0.5), VIS_UPDATE_DURATION(10.0),
      VIS_MAP_MINX(-200.0), VIS_MAP_MINY(-200.0), VIS_MAP_MINZ(-1.0),
      VIS_MAP_MAXX(+200.0), VIS_MAP_MAXY(+200.0), VIS_MAP_MAXZ(10.0), VIS_SLICE_LEVEL(2.0),
      msg_ready_(false), occu_update_count_(0), dist_update_count_(0), frontier_update_count_(0)
{
    node_handle_ = external_node;
    tf_buffer_ = external_tf_buffer;
    tf_listener_ = external_tf_listener;
    node_name_ = get_node_name();
    initialize();
}

void VDBMap::initialize()
{
    RCLCPP_INFO(node_handle_->get_logger(), "Initializing vdb-edt.");
    setup_parameters();
    RCLCPP_INFO(node_handle_->get_logger(), "Finished param setup.");
    load_mapping_para();
    RCLCPP_INFO(node_handle_->get_logger(), "Finished loading mapping param.");

    last_timing_print_ = node_handle_->now();

    // TF2 buffer/listener
    if (!tf_buffer_)
    {
        RCLCPP_INFO(node_handle_->get_logger(), "Creating tf buffer for vdb-edt.");
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_handle_->get_clock());
        tf_buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(node_handle_->get_node_base_interface(),
                                                                                      node_handle_->get_node_timers_interface()));

        RCLCPP_INFO(node_handle_->get_logger(), "Creating tf listener for vdb-edt.");
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_handle_, false);
    }

    // Publishers
    occu_vis_pub_ = node_handle_->create_publisher<sensor_msgs::msg::PointCloud2>("/occ_grid", 5);
    slice_vis_pub_ = node_handle_->create_publisher<visualization_msgs::msg::Marker>("/dist_slice", 5);

    inflated_vis_pub_ = node_handle_->create_publisher<sensor_msgs::msg::PointCloud2>("/inflated_grid", 5);
    frontier_vis_pub = node_handle_->create_publisher<sensor_msgs::msg::PointCloud2>("/frontier_vis", 5);

    // OpenVDB init & grids
    openvdb::initialize();
    grid_logocc_ = openvdb::FloatGrid::create(0.0);
    this->set_voxel_size(*grid_logocc_, VOX_SIZE);

    if (enable_inflated_map_)
    {
        grid_inflated_ = openvdb::Int32Grid::create(0);
        grid_inflated_->setTransform(grid_logocc_->transform().copy());
        build_inflation_kernel();

        inflated_vis_pub_ = node_handle_->create_publisher<sensor_msgs::msg::PointCloud2>("/inflated_grid", 5);
        frontier_inflated_vis_pub_ = node_handle_->create_publisher<sensor_msgs::msg::PointCloud2>("/frontier_inflated_grid", 5);
    }

    if (enable_dist_map_)
    {
        max_coor_dist_ = static_cast<int>(MAX_UPDATE_DIST / VOX_SIZE);
        max_coor_sqdist_ = max_coor_dist_ * max_coor_dist_;
        grid_distance_ = std::make_shared<DynamicVDBEDT>(max_coor_dist_);
        grid_distance_->initialize(dist_map_, VOX_SIZE, VERSION);
        grid_distance_->setAccessor(dist_map_);

        update_edt_timer_ = node_handle_->create_wall_timer(std::chrono::duration<double>(EDT_UPDATE_DURATION),
                                                            std::bind(&VDBMap::update_edtmap, this));
    }

    if (enable_frontier_map_)
    {
        grid_frontier_ = openvdb::BoolGrid::create(false);
        grid_frontier_->setTransform(grid_logocc_->transform().copy());

        update_frontier_timer_ = node_handle_->create_wall_timer(std::chrono::duration<double>(FRONTIER_UPDATE_DURATION),
                                                                 std::bind(&VDBMap::update_frontier, this));
    }

    // Convert SENSOR_RANGE / START_RANGE from world (m) to index (voxels)
    {
        openvdb::Vec3d max_sense_dist(SENSOR_RANGE);
        openvdb::Vec3d sense_range_ijk = grid_logocc_->worldToIndex(max_sense_dist);
        SENSOR_RANGE = sense_range_ijk.x();

        openvdb::Vec3d min_sense_dist(START_RANGE);
        openvdb::Vec3d min_sense_ijk = grid_logocc_->worldToIndex(min_sense_dist);
        START_RANGE = min_sense_ijk.x();
    }

    // Visualization timer
    update_vis_timer_ = node_handle_->create_wall_timer(
        std::chrono::duration<double>(VIS_UPDATE_DURATION),
        std::bind(&VDBMap::visualize_maps, this));

    // General synced dataset via message_filters + tf2 MessageFilter
    cloud_sub_mf_.subscribe(node_handle_.get(), pcl_topic, rclcpp::SensorDataQoS().get_rmw_qos_profile());
    cloud_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
        cloud_sub_mf_, *tf_buffer_, worldframeId, 10, node_handle_);
    cloud_filter_->registerCallback(std::bind(&VDBMap::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(node_handle_->get_logger(), "[%s] VDBMap initialized.", node_name_.c_str());

    // Frontier manager
    if (enable_frontier_cluster_)
    {
        initialize_frontier_manager();
    }
}

VDBMap::~VDBMap()
{
    cloud_filter_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
}

void VDBMap::initialize_frontier_manager()
{
    frontier_manager_.initialize(grid_frontier_);
    frontier_cluster_pub_ = node_handle_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/frontier_clusters", 5);
    viewpoints_pub_ = node_handle_->create_publisher<geometry_msgs::msg::PoseArray>(
        "/frontier_viewpoints", 5);
}

//////////////////////////////////////////////////////////////////////////////////
// Node helpers

std::string VDBMap::get_node_name() const
{
    return node_handle_ ? node_handle_->get_name() : std::string("vdb_map_default");
}

rclcpp::Node::SharedPtr VDBMap::get_node_handle() const
{
    return node_handle_;
}

//////////////////////////////////////////////////////////////////////////////////
// Utilities

void VDBMap::set_voxel_size(openvdb::GridBase &grid, double vs)
{
    // const openvdb::math::Vec3d offset(vs / 2., vs / 2., vs / 2.);
    grid_transform_ = openvdb::math::Transform::createLinearTransform(vs);
    // tf->postTranslate(offset);
    grid.setTransform(grid_transform_);

    // openvdb::Vec3d xyz_0(0.0, 0.0, 0.0);
    // openvdb::Vec3d xyz_1(1.0, 1.0, 1.0);

    // openvdb::Vec3d ijk_0d = tf->worldToIndex(xyz_0);
    // openvdb::Vec3d ijk_1d = tf->worldToIndex(xyz_1);

    // openvdb::Vec3d ijk_0(0.0, 0.0, 0.0);
    // openvdb::Vec3d ijk_1(1.0, 1.0, 1.0);

    // openvdb::Vec3d xyz_0d = tf->indexToWorld(ijk_0);
    // openvdb::Vec3d xyz_1d = tf->indexToWorld(ijk_1);

    // RCLCPP_INFO_STREAM(node_handle_->get_logger(), "check xyz position: xyz = " << xyz_0 << ", mapping to ijk = " << ijk_0d);
    // RCLCPP_INFO_STREAM(node_handle_->get_logger(), "check xyz position: xyz = " << xyz_1 << ", mapping to ijk = " << ijk_1d);

    // RCLCPP_INFO_STREAM(node_handle_->get_logger(), "check ijk position: ijk = " << ijk_0 << ", mapping to xyz = " << xyz_0d);
    // RCLCPP_INFO_STREAM(node_handle_->get_logger(), "check ijk position: ijk = " << ijk_1 << ", mapping to xyz = " << xyz_1d);
}

openvdb::math::Transform::ConstPtr VDBMap::get_grid_transform() const
{
    return grid_transform_;
}

void VDBMap::setup_parameters()
{
    // ROS2: declare parameters with defaults
    node_handle_->declare_parameter<std::string>("pcl_topic", "/robot_1/sensors/ouster/point_cloud");
    node_handle_->declare_parameter<std::string>("world_frame_id", "map");
    node_handle_->declare_parameter<std::string>("robot_frame_id", "base_link");
    node_handle_->declare_parameter<std::string>("lidar_frame_id", "ouster");
    node_handle_->declare_parameter<std::string>("data_set", "shimizu");

    node_handle_->declare_parameter<double>("l_free", L_FREE);
    node_handle_->declare_parameter<double>("l_occu", L_OCCU);
    node_handle_->declare_parameter<double>("l_max", L_MAX);
    node_handle_->declare_parameter<double>("l_min", L_MIN);
    node_handle_->declare_parameter<double>("l_thresh", L_THRESH);
    node_handle_->declare_parameter<double>("vox_size", VOX_SIZE);

    node_handle_->declare_parameter<double>("start_range", START_RANGE);
    node_handle_->declare_parameter<double>("sensor_range", SENSOR_RANGE);

    node_handle_->declare_parameter<int>("vdbedt_version", VERSION);
    node_handle_->declare_parameter<double>("max_update_dist", MAX_UPDATE_DIST);
    node_handle_->declare_parameter<double>("edt_update_duration", EDT_UPDATE_DURATION);

    node_handle_->declare_parameter<int>("frontier_type", 0);
    node_handle_->declare_parameter<double>("frontier_update_duration", 2.0);

    node_handle_->declare_parameter<double>("vis_update_duration", VIS_UPDATE_DURATION);
    node_handle_->declare_parameter<double>("vis_slice_level", VIS_SLICE_LEVEL);
    node_handle_->declare_parameter<double>("vis_map_minx", VIS_MAP_MINX);
    node_handle_->declare_parameter<double>("vis_map_miny", VIS_MAP_MINY);
    node_handle_->declare_parameter<double>("vis_map_minz", VIS_MAP_MINZ);
    node_handle_->declare_parameter<double>("vis_map_maxx", VIS_MAP_MAXX);
    node_handle_->declare_parameter<double>("vis_map_maxy", VIS_MAP_MAXY);
    node_handle_->declare_parameter<double>("vis_map_maxz", VIS_MAP_MAXZ);

    node_handle_->declare_parameter<bool>("enable_dist_map", true);
    node_handle_->declare_parameter<bool>("enable_frontier_map", true);
    node_handle_->declare_parameter<bool>("enable_frontier_cluster", true);
    node_handle_->declare_parameter<bool>("enable_inflated_map", true);

    node_handle_->declare_parameter<double>("safe_robot_radius_xy", 0.5);
    node_handle_->declare_parameter<double>("safe_robot_height_z", 0.2);
}

bool VDBMap::load_mapping_para()
{
    auto log = node_handle_->get_logger();

    if (node_handle_->get_parameter("pcl_topic", pcl_topic))
    {
        RCLCPP_INFO(log, "Parameter pcl_topic set to: %s", pcl_topic.c_str());
    }
    else
    {
        RCLCPP_ERROR(log, "Please set input point cloud topic before running the node.");
        return false;
    }

    if (node_handle_->get_parameter("world_frame_id", worldframeId))
    {
        RCLCPP_INFO(log, "Parameter world_frame_id set to: %s", worldframeId.c_str());
    }
    else
    {
        RCLCPP_ERROR(log, "Please set input world frame id before running the node.");
        return false;
    }

    if (node_handle_->get_parameter("robot_frame_id", robotframeId))
    {
        RCLCPP_INFO(log, "Parameter robot_frame_id set to: %s", robotframeId.c_str());
    }
    else
    {
        RCLCPP_ERROR(log, "Please set input robot frame id before running the node.");
        return false;
    }

    std::string dataset_name;
    if (node_handle_->get_parameter("data_set", dataset_name))
    {
        RCLCPP_INFO(log, "config source: %s.", dataset_name.c_str());
    }

    auto report_double = [&](const char *name, double &var)
    {
        if (node_handle_->get_parameter(name, var))
        {
            RCLCPP_INFO(log, "Overriding Parameter %s to: %f", name, var);
        }
        else
        {
            RCLCPP_WARN(log, "Using the default %s: %f", name, var);
        }
    };
    auto report_int = [&](const char *name, int &var)
    {
        if (node_handle_->get_parameter(name, var))
        {
            RCLCPP_INFO(log, "Overriding Parameter %s to: %d", name, var);
        }
        else
        {
            RCLCPP_WARN(log, "Using the default %s: %d", name, var);
        }
    };
    auto report_bool = [&](const char *name, bool &var)
    {
        if (node_handle_->get_parameter(name, var))
        {
            RCLCPP_INFO(log, "Parameter %s set to: %s", name, var ? "true" : "false");
        }
        else
        {
            RCLCPP_WARN(log, "Using default %s: %s", name, var ? "true" : "false");
        }
    };

    report_int("frontier_type", frontier_type_);
    report_double("l_free", L_FREE);
    report_double("l_occu", L_OCCU);
    report_double("l_max", L_MAX);
    report_double("l_min", L_MIN);
    report_double("l_thresh", L_THRESH);
    report_double("vox_size", VOX_SIZE);
    report_double("start_range", START_RANGE);
    report_double("sensor_range", SENSOR_RANGE);
    report_int("vdbedt_version", VERSION);
    report_double("max_update_dist", MAX_UPDATE_DIST);
    report_double("edt_update_duration", EDT_UPDATE_DURATION);
    report_double("frontier_update_duration", FRONTIER_UPDATE_DURATION);
    report_double("vis_update_duration", VIS_UPDATE_DURATION);
    report_double("vis_slice_level", VIS_SLICE_LEVEL);
    report_double("vis_map_minx", VIS_MAP_MINX);
    report_double("vis_map_miny", VIS_MAP_MINY);
    report_double("vis_map_minz", VIS_MAP_MINZ);
    report_double("vis_map_maxx", VIS_MAP_MAXX);
    report_double("vis_map_maxy", VIS_MAP_MAXY);
    report_double("vis_map_maxz", VIS_MAP_MAXZ);
    report_double("safe_robot_radius_xy", safe_robot_radius_xy_);
    report_double("safe_robot_height_z", safe_robot_height_z_);
    report_bool("enable_dist_map", enable_dist_map_);
    report_bool("enable_inflated_map", enable_inflated_map_);
    report_bool("enable_frontier_map", enable_frontier_map_);
    report_bool("enable_frontier_cluster", enable_frontier_cluster_);
    report_bool("enable_inflated_map", enable_inflated_map_);

    if (!enable_frontier_map_)
    {
        RCLCPP_WARN(log, "Frontier clustering disabled because frontier is disabled");
        enable_frontier_cluster_ = false;
    }
    if (!enable_dist_map_)
    {
        RCLCPP_WARN(log, "Frontier clustering disabled because dist map is disabled");
        enable_frontier_cluster_ = false;
    }

    return true;
}

bool VDBMap::load_planning_para()
{
    std::cout << "planning parameters no set !" << std::endl;
    return true;
}

// Get lock and grid accessor (for outside batch query)
std::shared_mutex &VDBMap::get_map_mutex() const
{
    return map_mutex;
}

openvdb::Int32Grid::ConstAccessor VDBMap::get_inflated_accessor() const
{
    if (!grid_inflated_)
    {
        RCLCPP_ERROR(node_handle_->get_logger(), "Inflated grid is not initialized! Cannot get accessor.");
        throw std::runtime_error("Inflated grid is null.");
    }
    return grid_inflated_->getConstAccessor();
}

// Get value in grids
bool VDBMap::query_log_odds_at_world(const Eigen::Vector3d &p_world, float &logodds_out) const
{
    if (!grid_logocc_)
    {
        return false;
    }

    const openvdb::Vec3d w(p_world.x(), p_world.y(), p_world.z());
    const openvdb::Vec3d ijk_d = grid_logocc_->worldToIndex(w);
    const openvdb::Coord ijk = openvdb::Coord::round(ijk_d);

    return query_log_odds_at_index(ijk, logodds_out);
}

bool VDBMap::query_log_odds_at_index(const openvdb::Coord &ijk, float &logodds_out) const
{
    if (!grid_logocc_)
    {
        return false;
    }

    std::shared_lock<std::shared_mutex> rlk(map_mutex);

    openvdb::FloatGrid::ConstAccessor acc = grid_logocc_->getConstAccessor();
    return acc.probeValue(ijk, logodds_out);
}

bool VDBMap::query_sqdist_at_world(const Eigen::Vector3d &p_world, double &sqdist_out) const
{
    if (!grid_distance_)
    {
        return false;
    }

    const openvdb::Vec3d w(p_world.x(), p_world.y(), p_world.z());
    const openvdb::Vec3d ijk_d = dist_map_->worldToIndex(w);
    const openvdb::Coord ijk = openvdb::Coord::round(ijk_d);

    return query_sqdist_at_index(ijk, sqdist_out);
}

bool VDBMap::query_sqdist_at_index(const openvdb::Coord &ijk, double &sqdist_out) const
{
    if (!grid_distance_)
    {
        return false;
    }

    std::shared_lock<std::shared_mutex> rlk(map_mutex);

    double sq = grid_distance_->query_sq_distance(ijk);

    // unknown, no value at dist map
    if (sq < 0.0)
    {
        return false;
    }

    sqdist_out = sq;
    return true;
}

bool VDBMap::query_is_inflated_at_world(const Eigen::Vector3d &p_world, int &inflate_val) const
{
    if (!grid_inflated_)
    {
        return false;
    }

    const openvdb::Vec3d w(p_world.x(), p_world.y(), p_world.z());
    const openvdb::Vec3d ijk_d = grid_inflated_->worldToIndex(w);
    const openvdb::Coord ijk = openvdb::Coord::round(ijk_d);

    return query_is_inflated_at_index(ijk, inflate_val);
}

bool VDBMap::query_is_inflated_at_index(const openvdb::Coord &ijk, int &inflate_val) const
{
    if (!grid_inflated_)
    {
        return false;
    }

    std::shared_lock<std::shared_mutex> rlk(map_mutex);
    return grid_inflated_->getConstAccessor().probeValue(ijk, inflate_val);
}

// Query with a given accessor, MAKE SURE TO ADD A MUTEX WHEN CREATING THE ACCESSOR
// Logocc map
bool VDBMap::query_log_odds_at_world(const Eigen::Vector3d &p_world, float &logodds_out, openvdb::FloatGrid::ConstAccessor &acc) const
{
    if (!grid_logocc_)
    {
        return false;
    }

    const openvdb::Vec3d w(p_world.x(), p_world.y(), p_world.z());
    const openvdb::Vec3d ijk_d = grid_logocc_->worldToIndex(w);
    const openvdb::Coord ijk = openvdb::Coord::round(ijk_d);

    return query_log_odds_at_index(ijk, logodds_out, acc);
}

bool VDBMap::query_log_odds_at_index(const openvdb::Coord &ijk, float &logodds_out, openvdb::FloatGrid::ConstAccessor &acc) const
{
    return acc.probeValue(ijk, logodds_out);
}

// Inflated map
bool VDBMap::query_is_inflated_at_world(const Eigen::Vector3d &p_world, int &inflate_val, openvdb::Int32Grid::ConstAccessor &acc) const
{
    if (!grid_inflated_)
    {
        return false;
    }

    const openvdb::Vec3d w(p_world.x(), p_world.y(), p_world.z());
    const openvdb::Vec3d ijk_d = grid_inflated_->worldToIndex(w);
    const openvdb::Coord ijk = openvdb::Coord::round(ijk_d);

    return query_is_inflated_at_index(ijk, inflate_val, acc);
}

bool VDBMap::query_is_inflated_at_index(const openvdb::Coord &ijk, int &inflate_val, openvdb::Int32Grid::ConstAccessor &acc) const
{
    return acc.probeValue(ijk, inflate_val);
}

bool VDBMap::ray_esdf_clear_index(const openvdb::Coord &c0,
                                  const openvdb::Coord &c1,
                                  double min_clearance,
                                  openvdb::Coord &hit_point) const
{
    if (!grid_distance_)
    {
        return false;
    }
    const openvdb::Vec3d p0_ijk(c0.x(), c0.y(), c0.z());
    const openvdb::Vec3d p1_ijk(c1.x(), c1.y(), c1.z());
    openvdb::Vec3d dir = p1_ijk - p0_ijk;
    const double len = dir.length();

    if (len <= 1e-9)
    {
        double sq = grid_distance_->query_sq_distance(c0);
        return (sq >= min_clearance * min_clearance);
    }

    dir /= len;

    openvdb::math::Ray<double> ray(p0_ijk, dir);

    const double eps = 1e-6;
    openvdb::math::DDA<openvdb::math::Ray<double>, 0> dda(ray, 0.0, len + eps);
    std::shared_lock<std::shared_mutex> rlk(map_mutex);

    double min_sq_clearance = min_clearance * min_clearance;

    for (;;)
    {
        const openvdb::Coord vox = dda.voxel();

        double sq = grid_distance_->query_sq_distance(vox);

        if (sq < min_sq_clearance)
        {
            hit_point = vox;
            return false;
        }

        if (vox == c1)
        {
            break;
        }

        if (!(dda.time() < dda.maxTime()))
        {
            break;
        }
        dda.step();
    }
    return true;
}

bool VDBMap::ray_esdf_clear_index_optimistic(const openvdb::Coord &c0,
                                             const openvdb::Coord &c1,
                                             double min_clearance,
                                             openvdb::Coord &hit_point) const
{
    if (!grid_distance_)
    {
        return false;
    }
    const openvdb::Vec3d p0_ijk(c0.x(), c0.y(), c0.z());
    const openvdb::Vec3d p1_ijk(c1.x(), c1.y(), c1.z());
    openvdb::Vec3d dir = p1_ijk - p0_ijk;
    const double len = dir.length();

    if (len <= 1e-9)
    {
        double sq = grid_distance_->query_sq_distance(c0);
        return (sq >= min_clearance * min_clearance);
    }

    dir /= len;

    openvdb::math::Ray<double> ray(p0_ijk, dir);

    const double eps = 1e-6;
    openvdb::math::DDA<openvdb::math::Ray<double>, 0> dda(ray, 0.0, len + eps);
    std::shared_lock<std::shared_mutex> rlk(map_mutex);

    double min_sq_clearance = min_clearance * min_clearance;

    for (;;)
    {
        const openvdb::Coord vox = dda.voxel();

        double sq = grid_distance_->query_sq_distance(vox);

        if (sq >= 0 && sq < min_sq_clearance)
        {
            hit_point = vox;
            return false;
        }

        if (vox == c1)
        {
            break;
        }

        if (!(dda.time() < dda.maxTime()))
        {
            break;
        }
        dda.step();
    }
    return true;
}

bool VDBMap::ray_inflated_clear_index(const openvdb::Coord &c0,
                                      const openvdb::Coord &c1) const
{
    if (!grid_inflated_)
    {
        return false;
    }

    const int dx = c1.x() - c0.x();
    const int dy = c1.y() - c0.y();
    const int dz = c1.z() - c0.z();

    const int steps = std::max({std::abs(dx), std::abs(dy), std::abs(dz)});
    if (steps == 0)
    {
        int val = 0;
        bool active = query_is_inflated_at_index(c0, val);
        return !(active && (val % FRONTIER_INFLATION_DELTA) > 0);
    }

    const double inv = 1.0 / steps;
    const double sx = dx * inv;
    const double sy = dy * inv;
    const double sz = dz * inv;

    std::shared_lock<std::shared_mutex> rlk(map_mutex);
    openvdb::Int32Grid::ConstAccessor inf_acc = grid_inflated_->getConstAccessor();

    for (int i = 0; i <= steps; ++i)
    {
        const openvdb::Coord vox(
            c0.x() + static_cast<int>(std::round(sx * i)),
            c0.y() + static_cast<int>(std::round(sy * i)),
            c0.z() + static_cast<int>(std::round(sz * i)));

        int inflate_val = 0;
        bool is_inflated = query_is_inflated_at_index(vox, inflate_val, inf_acc);
        if (is_inflated && (inflate_val % FRONTIER_INFLATION_DELTA) > 0)
        {
            return false;
        }
    }
    return true;
}

bool VDBMap::ray_all_inflated_clear_index(const openvdb::Coord &c0,
                                          const openvdb::Coord &c1) const
{
    if (!grid_inflated_)
    {
        return false;
    }

    const int dx = c1.x() - c0.x();
    const int dy = c1.y() - c0.y();
    const int dz = c1.z() - c0.z();

    const int steps = std::max({std::abs(dx), std::abs(dy), std::abs(dz)});
    if (steps == 0)
    {
        int val = 0;
        bool active = query_is_inflated_at_index(c0, val);
        return !(active && val > 0);
    }

    const double inv = 1.0 / steps;
    const double sx = dx * inv;
    const double sy = dy * inv;
    const double sz = dz * inv;

    std::shared_lock<std::shared_mutex> rlk(map_mutex);
    openvdb::Int32Grid::ConstAccessor inf_acc = grid_inflated_->getConstAccessor();

    for (int i = 0; i <= steps; ++i)
    {
        const openvdb::Coord vox(
            c0.x() + static_cast<int>(std::round(sx * i)),
            c0.y() + static_cast<int>(std::round(sy * i)),
            c0.z() + static_cast<int>(std::round(sz * i)));

        int inflate_val = 0;
        bool is_inflated = query_is_inflated_at_index(vox, inflate_val, inf_acc);
        if (is_inflated && inflate_val > 0)
        {
            return false;
        }
    }
    return true;
}

// Inflation of occ voxel grid

void VDBMap::build_inflation_kernel()
{
    int rx = static_cast<int>(std::ceil(safe_robot_radius_xy_ / VOX_SIZE));
    int ry = rx;
    int rz = static_cast<int>(std::ceil(safe_robot_height_z_ / VOX_SIZE));

    inflation_kernel_.clear();

    for (int x = -rx; x <= rx; ++x)
    {
        for (int y = -ry; y <= ry; ++y)
        {
            for (int z = -rz; z <= rz; ++z)
            {
                if (x * x + y * y <= rx * rx && std::abs(z) <= rz)
                {
                    inflation_kernel_.push_back(openvdb::Coord(x, y, z));
                }
            }
        }
    }

    int frx = std::max(0, rx - 1);
    int fry = std::max(0, ry - 1);
    int frz = std::max(0, rz - 1);

    frontier_inflation_kernel_.clear();

    for (int x = -frx; x <= frx; ++x)
    {
        for (int y = -fry; y <= fry; ++y)
        {
            for (int z = -frz; z <= frz; ++z)
            {
                if (x * x + y * y <= frx * frx && std::abs(z) <= frz)
                {
                    frontier_inflation_kernel_.push_back(openvdb::Coord(x, y, z));
                }
            }
        }
    }
}

inline void VDBMap::apply_inflation(openvdb::Int32Grid::Accessor &inf_acc,
                                    const openvdb::Coord &center,
                                    int delta)
{
    for (const auto &offset : inflation_kernel_)
    {
        openvdb::Coord target = center + offset;
        int old_val = 0;
        inf_acc.probeValue(target, old_val);
        int new_val = old_val + delta;

        if (new_val > 0)
        {
            inf_acc.setValueOn(target, new_val);
        }
        else
        {
            inf_acc.setValueOff(target, 0); // 彻底释放内存
        }
    }
}

inline void VDBMap::apply_frontier_inflation(openvdb::Int32Grid::Accessor &inf_acc,
                                             const openvdb::Coord &center,
                                             int delta)
{
    for (const auto &offset : frontier_inflation_kernel_)
    {
        openvdb::Coord target = center + offset;
        int old_val = 0;
        inf_acc.probeValue(target, old_val);
        int new_val = old_val + delta;

        if (new_val > 0)
        {
            inf_acc.setValueOn(target, new_val);
        }
        else
        {
            inf_acc.setValueOff(target, 0);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// Callbacks

void VDBMap::cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc_msg)
{
    // TF2: MessageFilter ensures transform availability, but still catch exceptions
    geometry_msgs::msg::TransformStamped transform;
    try
    {
        transform = tf_buffer_->lookupTransform(worldframeId,
                                                pc_msg->header.frame_id,
                                                pc_msg->header.stamp);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(node_handle_->get_logger(), "TF lookup failed: %s", ex.what());
        return;
    }

    // Transform point cloud into world frame
    sensor_msgs::msg::PointCloud2 pc_world;
    try
    {
        tf2::doTransform(*pc_msg, pc_world, transform);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node_handle_->get_logger(), "doTransform failed: %s", e.what());
        return;
    }

    // Extract origin from transform
    origin_ = Eigen::Vector3d(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z);

    // Convert to PCL XYZ
    auto xyz = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(pc_world, *xyz);

    // update float occupancy map
    occu_update_count_++;
    // std::cout << "Running " << occu_update_count_ << " updates." << std::endl;

    timing::Timer update_OCC_timer("UpdateOccu");
    this->update_occmap(grid_logocc_, origin_, xyz);
    update_OCC_timer.Stop();

    // timing::Timing::Print(std::cout);
    const auto now = node_handle_->now();
    if ((now - last_timing_print_).seconds() >= 1.0)
    {
        RCLCPP_INFO(node_handle_->get_logger(), "%s",
                    timing::Timing::Print().c_str());
        last_timing_print_ = now;
    }
    msg_ready_ = true;
}

////////////////////////////////////////////////////////////////////////////////
// Visualization

void VDBMap::visualize_maps()
{
    // visualize the occupancy map
    if (occu_vis_pub_->get_subscription_count() > 0)
    {
        sensor_msgs::msg::PointCloud2 cloud_vis;
        this->grid_message(grid_logocc_, cloud_vis);
        occu_vis_pub_->publish(cloud_vis);
    }

    // visualize the inflated map (obstacle channel only)
    if (enable_inflated_map_ && grid_inflated_ && inflated_vis_pub_->get_subscription_count() > 0)
    {
        sensor_msgs::msg::PointCloud2 inflated_vis;
        this->inflated_grid_message(grid_inflated_, inflated_vis, InflationChannel::OCC_ONLY);
        inflated_vis_pub_->publish(inflated_vis);
    }

    // visualize the frontier inflated map (frontier channel only)
    if (enable_inflated_map_ && grid_inflated_ && frontier_inflated_vis_pub_ &&
        frontier_inflated_vis_pub_->get_subscription_count() > 0)
    {
        sensor_msgs::msg::PointCloud2 frontier_inflated_vis;
        this->inflated_grid_message(grid_inflated_, frontier_inflated_vis, InflationChannel::FRONTIER_ONLY);
        frontier_inflated_vis_pub_->publish(frontier_inflated_vis);
    }

    // visualize the distance map
    if (enable_dist_map_ && slice_vis_pub_->get_subscription_count() > 0)
    {
        visualization_msgs::msg::Marker slice_maker;
        auto vis_coor = static_cast<int>(2.0 / VOX_SIZE);
        this->get_slice_marker(slice_maker, 100, VIS_SLICE_LEVEL, vis_coor * vis_coor);
        slice_vis_pub_->publish(slice_maker);
    }
}

void VDBMap::grid_to_pcl(openvdb::FloatGrid::ConstPtr grid,
                         openvdb::FloatGrid::ValueType thresh,
                         std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &pc_out)
{
    pcl::PointXYZI point_xyzi;

    using value_type = openvdb::FloatGrid::ValueType;
    using itr_type = openvdb::FloatGrid::ValueOnCIter;

    const openvdb::math::Transform &grid_tf(grid->transform());

    for (itr_type itr = grid->cbeginValueOn(); itr.test(); ++itr)
    {
        if (!itr.isVoxelValue())
            continue;
        value_type val = itr.getValue();
        if (val < thresh)
            continue;

        openvdb::Coord ijk = itr.getCoord();
        openvdb::Vec3d p = grid_tf.indexToWorld(ijk);

        point_xyzi.x = static_cast<float>(p.x());
        point_xyzi.y = static_cast<float>(p.y());
        point_xyzi.z = static_cast<float>(p.z());
        point_xyzi.intensity = static_cast<float>(val);
        pc_out->points.push_back(point_xyzi);
    }
    pc_out->width = static_cast<uint32_t>(pc_out->points.size());
    pc_out->height = 1;
}

void VDBMap::grid_message(const openvdb::FloatGrid::Ptr &grid,
                          sensor_msgs::msg::PointCloud2 &disp_msg)
{
    std::shared_lock<std::shared_mutex> rlk(map_mutex);
    auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    this->grid_to_pcl(grid, static_cast<openvdb::FloatGrid::ValueType>(L_THRESH), pcl_cloud);
    pcl::toROSMsg(*pcl_cloud, disp_msg);
    disp_msg.header.frame_id = worldframeId;
    disp_msg.header.stamp = node_handle_->now();
}

////////////////////////////////////////////////////////////////////////////////
// EDT

void VDBMap::update_edtmap()
{
    if (!msg_ready_)
    {
        return;
    }

    // Lock
    std::unique_lock<std::shared_mutex> lk(map_mutex);

    // update float occupancy map
    dist_update_count_++;
    std::cout << "Running " << dist_update_count_ << " updates." << std::endl;
    timing::Timer update_DIST_timer("UpdateDIST");
    this->grid_distance_->update();
    update_DIST_timer.Stop();
    timing::Timing::Print(std::cout);
    std::cout << "changed: " << grid_distance_->sum_occ_changed
              << " raised: " << grid_distance_->sum_raised_num
              << " lowered: " << grid_distance_->sum_lowered_num << std::endl;
    msg_ready_ = false;
}

////////////////////////////////////////////////////////////////////////////////
// Occupancy update (ray casting)

void VDBMap::update_occmap(openvdb::FloatGrid::Ptr grid_map,
                           const Eigen::Vector3d &origin,
                           std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> xyz)
{
    // Accumulate log-odds deltas per voxel (no grid access here).
    std::unordered_map<openvdb::Coord, float, CoordHash> ll_delta;
    ll_delta.reserve(xyz->size() * 8);

    // potential new frontiers
    std::unordered_set<openvdb::Coord, CoordHash> patch_local;

    const openvdb::Vec3d origin_ijk =
        grid_map->worldToIndex(openvdb::Vec3d(origin.x(), origin.y(), origin.z()));

    for (const pcl::PointXYZ &pt : *xyz)
    {
        const openvdb::Vec3d p_ijk =
            grid_map->worldToIndex(openvdb::Vec3d(pt.x, pt.y, pt.z));
        openvdb::Vec3d dir = p_ijk - origin_ijk;
        const double range = dir.length();
        // if (range <= 1e-6)
        //     {continue;}
        if (range < START_RANGE)
        {
            continue;
        }
        dir.normalize();

        openvdb::math::Ray<double> ray(origin_ijk, dir);
        openvdb::math::DDA<openvdb::math::Ray<double>, 0> dda(
            ray, 0.0, std::min(SENSOR_RANGE, range));

        // Free space along the ray (exclude the first hit voxel).
        for (;;)
        {
            const openvdb::Coord cur = dda.voxel();
            // If stepping further would exceed/meet maxTime, `cur` is the hit voxel → don't add FREE.
            if (!(dda.time() < dda.maxTime()))
            {
                break;
            }
            ll_delta[cur] += static_cast<float>(L_FREE);
            dda.step();
        }

        // Hit (with thickness) only if not truncated by sensor range.
        if (range <= SENSOR_RANGE)
        {
            // first hit voxel
            {
                const openvdb::Coord hit = dda.voxel();
                ll_delta[hit] += static_cast<float>(L_OCCU);
            }
            // extra thickness: step then add, until maxTime
            for (int i = 1; i < HIT_THICKNESS; ++i)
            {
                if (!(dda.time() < dda.maxTime()))
                    break;
                dda.step();
                ll_delta[dda.voxel()] += static_cast<float>(L_OCCU);
            }
        }
    }

    // Batch apply under lock: write occupancy & notify EDT
    {
        std::unique_lock<std::shared_mutex> lk(map_mutex);

        auto acc = grid_map->getAccessor();

        // inflated map
        std::unique_ptr<openvdb::Int32Grid::Accessor> inf_acc_ptr;
        if (enable_inflated_map_)
        {
            inf_acc_ptr = std::make_unique<openvdb::Int32Grid::Accessor>(grid_inflated_->getAccessor());
        }

        const float occ_thresh = static_cast<float>(L_THRESH);

        for (const auto &kv : ll_delta)
        {
            const openvdb::Coord &ijk = kv.first;
            const float dL = kv.second;

            float ll_old = 0.0f;
            const bool known = acc.probeValue(ijk, ll_old);

            const float ll_new = std::clamp(ll_old + dL,
                                            static_cast<float>(L_MIN),
                                            static_cast<float>(L_MAX));

            if (!known)
            {
                // Unknown → write & init EDT cell
                acc.setValueOn(ijk, ll_new);
                if (enable_dist_map_)
                {
                    grid_distance_->dist_acc_->setValueOn(ijk);
                    if (ll_new >= occ_thresh)
                    {
                        openvdb::Coord c = ijk; // unknown -> occupied
                        grid_distance_->setObstacle(c);
                    }
                }

                // Inflated map
                if (enable_inflated_map_ && ll_new >= occ_thresh)
                {
                    apply_inflation(*inf_acc_ptr, ijk, +1);
                }
            }
            else
            {
                const bool wasOcc = (ll_old >= occ_thresh);
                const bool nowOcc = (ll_new >= occ_thresh);

                acc.setValueOn(ijk, ll_new);
                if (enable_dist_map_)
                {
                    if (!wasOcc && nowOcc)
                    {
                        openvdb::Coord c = ijk; // free -> occupied
                        grid_distance_->setObstacle(c);
                    }
                    else if (wasOcc && !nowOcc)
                    {
                        openvdb::Coord c = ijk; // occupied -> free
                        grid_distance_->removeObstacle(c);
                    }
                }

                if (enable_inflated_map_)
                {
                    if (!wasOcc && nowOcc)
                    {
                        apply_inflation(*inf_acc_ptr, ijk, +1);
                    }
                    else if (wasOcc && !nowOcc)
                    {
                        apply_inflation(*inf_acc_ptr, ijk, -1);
                    }
                }
            }
            if (enable_frontier_map_)
            {
                const bool nowFree = (ll_new < occ_thresh);
                const bool wasFree = (known && ll_old < occ_thresh);
                if (nowFree && !wasFree)
                {
                    patch_local.insert(ijk); // potential new frontier must be new free (unknown->free or occ->free)
                }
            }
        }
    } // map_mutex release

    if (!patch_local.empty())
    {
        std::lock_guard<std::mutex> slk(swap_lock);
        patch_write->insert(patch_local.begin(), patch_local.end());
    }
}

////////////////////////////////////////////////////////////////////////////////
// Frontier Extraction
void VDBMap::update_frontier()
{
    std::vector<openvdb::Coord> to_off;
    std::vector<openvdb::Coord> to_on;
    const float occ_thresh = static_cast<float>(L_THRESH);

    UpdatedArea snap;
    {
        std::lock_guard<std::mutex> g(swap_lock);
        std::swap(patch_write, patch_read);
        snap = std::move(*patch_read);
    }

    // Nothing to update
    if (snap.empty())
    {
        return;
    }

    frontier_update_count_++;
    std::cout << "Running " << frontier_update_count_ << " frontier updates." << std::endl;
    timing::Timer update_FRONTIER_timer("UpdateFrontier");

    auto c_it = snap.begin();
    openvdb::CoordBBox update_box(*c_it, *c_it);

    for (++c_it; c_it != snap.end(); ++c_it)
    {
        update_box.expand(*c_it);
    }

    using FrontierCheckFunc = bool (*)(const openvdb::FloatGrid::ConstAccessor &, const openvdb::Coord &, double);
    FrontierCheckFunc check_func = nullptr;

    switch (frontier_type_)
    {
    case 0:
    default:
        check_func = &VDBMap::check_frontier_6;
        break;
    case 1:
        check_func = &VDBMap::check_surface_frontier_6;
        break;
    case 2:
        check_func = &VDBMap::check_surface_frontier_26;
        break;
    }

    {
        std::shared_lock<std::shared_mutex> rlk(map_mutex);
        openvdb::FloatGrid::ConstAccessor occgrid_acc = grid_logocc_->getConstAccessor(); // read only

        // Check current frontiers: still frontier?
        update_box.expand(1);
        for (auto iter = grid_frontier_->cbeginValueOn(); iter; ++iter)
        {
            if (!iter.isVoxelValue() || !update_box.isInside(iter.getCoord()))
            {
                continue;
            }
            const openvdb::Coord ijk = iter.getCoord();
            if (!(*check_func)(occgrid_acc, ijk, L_THRESH))
            {
                to_off.push_back(ijk);
            }
        }

        // Check accumulated patch: new frontier?
        openvdb::BoolGrid::ConstAccessor frontier_read_acc = grid_frontier_->getConstAccessor();
        to_on.reserve(snap.size());
        for (const auto &ijk : snap)
        {
            if (frontier_read_acc.isValueOn(ijk))
            {
                // Avoid double inflation
                continue;
            }
            float v;
            if (!(occgrid_acc.probeValue(ijk, v) && v < occ_thresh))
            {
                continue;
            }
            if ((*check_func)(occgrid_acc, ijk, L_THRESH))
            {
                to_on.push_back(ijk);
            }
        }
    }

    {
        std::unique_lock<std::shared_mutex> lk(map_mutex);
        openvdb::BoolGrid::Accessor frontier_acc = grid_frontier_->getAccessor();

        if (enable_inflated_map_)
        {
            openvdb::Int32Grid::Accessor inf_acc = grid_inflated_->getAccessor();

            for (const auto &ijk : to_off)
            {
                frontier_acc.setValueOff(ijk, false);
                apply_frontier_inflation(inf_acc, ijk, -100000);
            }
            for (const auto &ijk : to_on)
            {
                frontier_acc.setValueOn(ijk, true);
                apply_frontier_inflation(inf_acc, ijk, +100000);
            }
        }
        else
        {
            for (const auto &ijk : to_off)
            {
                frontier_acc.setValueOff(ijk, false);
            }
            for (const auto &ijk : to_on)
            {
                frontier_acc.setValueOn(ijk, true);
            }
        }
    }

    openvdb::BoolGrid::ConstAccessor frontier_acc_read = grid_frontier_->getConstAccessor();
    openvdb::FloatGrid::ConstAccessor occgrid_acc_read = grid_logocc_->getConstAccessor();

    // Frontier Clustering
    if (enable_frontier_cluster_)
    { // RCLCPP_INFO(node_handle_->get_logger(), "Start clustering.");
        openvdb::CoordBBox expanded_box = frontier_manager_.expand_update_box(update_box, VOX_SIZE);
        frontier_manager_.reset_changed_clusters(expanded_box, frontier_acc_read);
        // RCLCPP_INFO(node_handle_->get_logger(), "Reset changed clusters.");
        frontier_manager_.frontier_clustering(grid_frontier_, expanded_box);
        // RCLCPP_INFO(node_handle_->get_logger(), "Frontier clustered.");
        frontier_manager_.compute_frontiers_to_visit(occgrid_acc_read, grid_distance_);
        // RCLCPP_INFO(node_handle_->get_logger(), "viewpoints generated.");
        frontier_manager_.update_frontier_cost_matrix();
        // RCLCPP_INFO(node_handle_->get_logger(), "Cost matrix updated.");
        vis_frontier_clusters();
        vis_frontier_viewpoints();
    }

    update_FRONTIER_timer.Stop();
    timing::Timing::Print(std::cout);

    vis_frontier();
}

////////////////////////////////////////////////////////////////////////////////
// Frontier Visualization
void VDBMap::vis_frontier()
{
    if (!frontier_vis_pub || frontier_vis_pub->get_subscription_count() == 0)
    {
        return;
    }

    std::shared_lock<std::shared_mutex> rlk(map_mutex);
    auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    this->frontier_to_pcl(grid_frontier_, pcl_cloud);

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*pcl_cloud, msg);
    msg.header.frame_id = worldframeId;
    msg.header.stamp = node_handle_->now();
    frontier_vis_pub->publish(msg);
}

void VDBMap::frontier_to_pcl(openvdb::BoolGrid::ConstPtr grid,
                             std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &pc_out)
{
    pcl::PointXYZ point_xyz;

    using itr_type = openvdb::BoolGrid::ValueOnCIter;

    const openvdb::math::Transform &grid_tf(grid->transform());

    for (itr_type itr = grid->cbeginValueOn(); itr.test(); ++itr)
    {
        if (!itr.isVoxelValue())
            continue;

        openvdb::Coord ijk = itr.getCoord();
        openvdb::Vec3d p = grid_tf.indexToWorld(ijk);

        point_xyz.x = static_cast<float>(p.x());
        point_xyz.y = static_cast<float>(p.y());
        point_xyz.z = static_cast<float>(p.z());
        pc_out->points.push_back(point_xyz);
    }
    pc_out->width = static_cast<uint32_t>(pc_out->points.size());
    pc_out->height = 1;
}

void VDBMap::vis_frontier_clusters()
{
    if (!frontier_cluster_pub_ || frontier_cluster_pub_->get_subscription_count() == 0)
    {
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
    frontier_manager_.clusters_to_pcl(cloud_xyzi);

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud_xyzi, msg);
    msg.header.frame_id = worldframeId; // 和你的 map/frame 对齐
    msg.header.stamp = node_handle_->now();

    frontier_cluster_pub_->publish(msg);
}

void VDBMap::vis_frontier_viewpoints()
{
    if (!viewpoints_pub_ || viewpoints_pub_->get_subscription_count() == 0)
    {
        return;
    }

    std::vector<Viewpoint> vps;
    frontier_manager_.export_viewpoints(vps);

    geometry_msgs::msg::PoseArray pa;
    pa.header.frame_id = worldframeId;
    pa.header.stamp = node_handle_->now();

    pa.poses.reserve(vps.size());

    for (const auto &vp : vps)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = vp.pos_.x();
        pose.position.y = vp.pos_.y();
        pose.position.z = vp.pos_.z();

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, vp.yaw_);

        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        pa.poses.push_back(pose);
    }

    viewpoints_pub_->publish(pa);
}


////////////////////////////////////////////////////////////////////////////////
// Slice visualization helpers

void VDBMap::get_slice_marker(visualization_msgs::msg::Marker &marker, int marker_id, double slice, double max_sqdist)
{
    marker.header.frame_id = worldframeId;
    marker.header.stamp = node_handle_->now();
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::MODIFY;
    marker.scale.x = VOX_SIZE;
    marker.scale.y = VOX_SIZE;
    marker.scale.z = VOX_SIZE;
    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;

    marker.points.clear();
    marker.colors.clear();

    // iterate the map
    std_msgs::msg::ColorRGBA c;
    openvdb::Vec3d map_min(VIS_MAP_MINX, VIS_MAP_MINY, VIS_MAP_MINZ);
    openvdb::Vec3d map_max(VIS_MAP_MAXX, VIS_MAP_MAXY, VIS_MAP_MAXZ);
    auto coor_min = dist_map_->worldToIndex(map_min);
    auto coor_max = dist_map_->worldToIndex(map_max);

    const int slice_idx = static_cast<int>(std::round(dist_map_->worldToIndex(openvdb::Vec3d(0.0, 0.0, slice)).z()));
    if (slice_idx < coor_min.z() || slice_idx > coor_max.z())
    // if (slice < coor_min.z() || slice > coor_max.z())
    {
        std::cout << "slice number is out of boundary!" << std::endl;
        return;
    }

    // int z = dist_map_->worldToIndex(openvdb::Vec3d(slice)).x();
    int z = slice_idx;
    std::shared_lock<std::shared_mutex> lk(map_mutex);
    for (int x = coor_min.x(); x <= coor_max.x(); ++x)
    {
        for (int y = coor_min.y(); y <= coor_max.y(); ++y)
        {
            openvdb::math::Coord target_cell(x, y, z);
            openvdb::math::Coord nearest_obst;
            auto cell_dist = grid_distance_->query_sq_distance(target_cell, nearest_obst);

            if (cell_dist < 0 || cell_dist >= max_coor_sqdist_)
            { // unknown cells
                continue;
            }
            auto world_xyz = dist_map_->indexToWorld(target_cell);

            geometry_msgs::msg::Point p;
            p.x = world_xyz.x();
            p.y = world_xyz.y();
            p.z = world_xyz.z();

            double norm = std::sqrt(static_cast<double>(cell_dist));
            double denom = std::sqrt(max_sqdist);
            c = rainbow_color_map((denom > 0.0) ? std::min(1.0, norm / denom) : 1.0);

            marker.points.push_back(p);
            marker.colors.push_back(c);

        } // end y loop
    } // end x loop
}

std_msgs::msg::ColorRGBA VDBMap::rainbow_color_map(double h)
{
    std_msgs::msg::ColorRGBA color;
    color.a = 1;
    // blend over HSV-values (more colors)

    double s = 1.0;
    double v = 1.0;

    h -= floor(h);
    h *= 6;
    int i;
    double m, n, f;

    i = static_cast<int>(floor(h));
    f = h - i;
    if (!(i & 1))
        f = 1 - f; // if i is even
    m = v * (1 - s);
    n = v * (1 - s * f);

    switch (i)
    {
    case 6:
    case 0:
        color.r = v;
        color.g = n;
        color.b = m;
        break;
    case 1:
        color.r = n;
        color.g = v;
        color.b = m;
        break;
    case 2:
        color.r = m;
        color.g = v;
        color.b = n;
        break;
    case 3:
        color.r = m;
        color.g = n;
        color.b = v;
        break;
    case 4:
        color.r = n;
        color.g = m;
        color.b = v;
        break;
    case 5:
        color.r = v;
        color.g = m;
        color.b = n;
        break;
    default:
        color.r = 1;
        color.g = 0.5;
        color.b = 0.5;
        break;
    }
    return color;
}

void VDBMap::inflated_grid_to_pcl(openvdb::Int32Grid::ConstPtr grid,
                                  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &pc_out,
                                  InflationChannel channel)
{
    pcl::PointXYZI point_xyzi;

    using value_type = openvdb::Int32Grid::ValueType;
    using itr_type = openvdb::Int32Grid::ValueOnCIter;

    const openvdb::math::Transform &grid_tf(grid->transform());

    for (itr_type itr = grid->cbeginValueOn(); itr.test(); ++itr)
    {
        if (!itr.isVoxelValue())
            continue;

        value_type val = itr.getValue();
        if (val <= 0)
            continue;

        int display_val = 0;
        switch (channel)
        {
        case InflationChannel::ALL:
            display_val = val;
            break;
        case InflationChannel::OCC_ONLY:
            display_val = val % FRONTIER_INFLATION_DELTA;
            break;
        case InflationChannel::FRONTIER_ONLY:
            display_val = val / FRONTIER_INFLATION_DELTA;
            break;
        }
        if (display_val <= 0)
            continue;

        openvdb::Coord ijk = itr.getCoord();
        openvdb::Vec3d p = grid_tf.indexToWorld(ijk);

        point_xyzi.x = static_cast<float>(p.x());
        point_xyzi.y = static_cast<float>(p.y());
        point_xyzi.z = static_cast<float>(p.z());
        point_xyzi.intensity = static_cast<float>(display_val);

        pc_out->points.push_back(point_xyzi);
    }
    pc_out->width = static_cast<uint32_t>(pc_out->points.size());
    pc_out->height = 1;
}

void VDBMap::inflated_grid_message(const openvdb::Int32Grid::Ptr &grid,
                                   sensor_msgs::msg::PointCloud2 &disp_msg,
                                   InflationChannel channel)
{
    std::shared_lock<std::shared_mutex> rlk(map_mutex);

    auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    this->inflated_grid_to_pcl(grid, pcl_cloud, channel);
    pcl::toROSMsg(*pcl_cloud, disp_msg);
    disp_msg.header.frame_id = worldframeId;
    disp_msg.header.stamp = node_handle_->now();
}