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
 *     * Redistributions of binary form must reproduce the above copyright
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

#ifndef _VDBMAP_H_
#define _VDBMAP_H_

#include <queue>
#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <limits>
#include <unordered_set>

// C++17 locks (replace boost::shared_mutex)
#include <shared_mutex>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

// NOTE(ROS2): doTransform for PointCloud2 lives in tf2_sensor_msgs (used in .cpp).
// #include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

// message_filters in ROS2
#include <message_filters/subscriber.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// OpenVDB
#include <openvdb/openvdb.h>
#include <openvdb/Grid.h>
#include <openvdb/math/DDA.h>
#include <openvdb/math/Ray.h>

#include "vdb_edt/dynamicVDBEDT.h"
#include "vdb_edt/timing.h"

#include "vdb_edt/frontier_cluster.h"

// Prefer constexpr over macro for compile-time constants.
static constexpr int kPoseQueueSize = 20;

class VDBMap
{

public:
    VDBMap();
    VDBMap(const rclcpp::Node::SharedPtr &external_node);
    VDBMap(const rclcpp::Node::SharedPtr &external_node,
           const std::shared_ptr<tf2_ros::Buffer> &external_tf_buffer,
           const std::shared_ptr<tf2_ros::TransformListener> &external_tf_listener);
    ~VDBMap();

    // Lookup logocc value: low freq
    bool query_log_odds_at_world(const Eigen::Vector3d &p_world, float &logodds_out) const;
    bool query_log_odds_at_index(const openvdb::Coord &ijk, float &logodds_out) const;
    // high freq
    bool query_log_odds_at_world(const Eigen::Vector3d &p_world, float &logodds_out, openvdb::FloatGrid::ConstAccessor &acc) const;
    bool query_log_odds_at_index(const openvdb::Coord &ijk, float &logodds_out, openvdb::FloatGrid::ConstAccessor &acc) const;

    // Lookup inflated value: low freq
    bool query_is_inflated_at_world(const Eigen::Vector3d &p_world, int &inflate_val) const;
    bool query_is_inflated_at_index(const openvdb::Coord &ijk, int &inflate_val) const;
    // high freq
    bool query_is_inflated_at_world(const Eigen::Vector3d &p_world, int &inflate_val, openvdb::Int32Grid::ConstAccessor &acc) const;
    bool query_is_inflated_at_index(const openvdb::Coord &ijk, int &inflate_val, openvdb::Int32Grid::ConstAccessor &acc) const;

    bool query_sqdist_at_world(const Eigen::Vector3d &p_world, double &sqdist_out) const;
    bool query_sqdist_at_index(const openvdb::Coord &ijk, double &sqdist_out) const;

    bool ray_esdf_clear_index(const openvdb::Coord &c0,
                              const openvdb::Coord &c1,
                              double min_clearance,
                              openvdb::Coord &hit_point) const;

    bool ray_esdf_clear_index_optimistic(const openvdb::Coord &c0,
                                         const openvdb::Coord &c1,
                                         double min_clearance,
                                         openvdb::Coord &hit_point) const;

    bool ray_inflated_clear_index(const openvdb::Coord &c0,
                                  const openvdb::Coord &c1) const;

    openvdb::math::Transform::ConstPtr get_grid_transform() const;

    static constexpr int FRONTIER_INFLATION_DELTA = 100000;

private:
    // General parameters
    std::string pcl_topic;
    std::string worldframeId;
    std::string robotframeId;
    std::string dataset;

    // Mapping parameters
    double L_FREE, L_OCCU, L_THRESH, L_MIN, L_MAX, VOX_SIZE;
    double START_RANGE, SENSOR_RANGE;
    int HIT_THICKNESS;

    // VDB map
    int VERSION;
    double MAX_UPDATE_DIST;
    double VIS_MAP_MINX, VIS_MAP_MINY, VIS_MAP_MINZ; // for visualization
    double VIS_MAP_MAXX, VIS_MAP_MAXY, VIS_MAP_MAXZ; // for visualization
    double EDT_UPDATE_DURATION;
    double VIS_UPDATE_DURATION;
    double FRONTIER_UPDATE_DURATION;
    double VIS_SLICE_LEVEL; // in meters

    std::string node_name_;
    // ROS2: keep a single rclcpp node. No private node handle is needed.
    rclcpp::Node::SharedPtr node_handle_;

    rclcpp::Time last_timing_print_;

    openvdb::math::Transform::Ptr grid_transform_;

    // Module switch
    bool enable_dist_map_;
    bool enable_frontier_map_;
    bool enable_frontier_cluster_;
    bool enable_inflated_map_;

public:
    std::string get_node_name() const;
    rclcpp::Node::SharedPtr get_node_handle() const;

    std::shared_mutex &get_map_mutex() const;

    openvdb::Int32Grid::ConstAccessor get_inflated_accessor() const;

    // read-write lock (C++17)
    using Lock = std::shared_mutex;
    using WriteLock = std::unique_lock<Lock>;
    using ReadLock = std::shared_lock<Lock>;
    Lock pose_queue_lock;

    // common tool functions
    void setup_parameters();
    bool load_mapping_para();
    bool load_planning_para();

    // for visulization
    rclcpp::TimerBase::SharedPtr update_vis_timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occu_vis_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr slice_vis_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inflated_vis_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frontier_inflated_vis_pub_;
    // ROS2: rclcpp::Timer uses callback without TimerEvent by default.
    void visualize_maps();

    // general dataset with tf and point cloud
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_mf_;
    std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> cloud_filter_;

    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc_msg);

    /*** specially designed for lady_and_cow dataset
         there is a sync problem in this dataset
    */
    bool msg_ready_;
    // ROS2: store numeric origin as Eigen::Vector3d.
    Eigen::Vector3d origin_;
    geometry_msgs::msg::TransformStamped latest_pose_;

    // Use ConstSharedPtr to avoid large copies and reduce memory footprint.
    std::queue<geometry_msgs::msg::TransformStamped::ConstSharedPtr> pose_queue_;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> cloud_queue_;

public:
    // a hash tool, for lower version of openvdb without coord hash
private:
    struct CoordHash
    {
        std::size_t operator()(const openvdb::Coord &c) const noexcept
        {
            std::uint64_t h = 0xCBF29CE484222325ull;
            auto mix = [&](int v)
            {
                std::uint64_t x = static_cast<std::uint32_t>(v);
                h ^= x + 0x9E3779B97F4A7C15ull + (h << 6) + (h >> 2);
            };
            mix(c.x());
            mix(c.y());
            mix(c.z());
            return static_cast<std::size_t>(h);
        }
    };

private: // occupancy map
    void initialize();
    mutable std::shared_mutex map_mutex;

    openvdb::FloatGrid::Ptr grid_logocc_;

    // major functions
    void set_voxel_size(openvdb::GridBase &grid, double vs);
    void update_occmap(openvdb::FloatGrid::Ptr grid_map,
                       const Eigen::Vector3d &origin,
                       std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> xyz);

    // visualization
    void grid_to_pcl(openvdb::FloatGrid::ConstPtr grid,
                     openvdb::FloatGrid::ValueType thresh,
                     std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &pc_out);
    void grid_message(const openvdb::FloatGrid::Ptr &grid,
                      sensor_msgs::msg::PointCloud2 &disp_msg);

    // Inflated Map
    openvdb::Int32Grid::Ptr grid_inflated_;
    std::vector<openvdb::Coord> inflation_kernel_;
    std::vector<openvdb::Coord> frontier_inflation_kernel_;

    void build_inflation_kernel();
    void apply_inflation(openvdb::Int32Grid::Accessor &inf_acc,
                         const openvdb::Coord &center,
                         int delta);
    void apply_frontier_inflation(openvdb::Int32Grid::Accessor &inf_acc,
                                  const openvdb::Coord &center,
                                  int delta);

    enum class InflationChannel { ALL, OCC_ONLY, FRONTIER_ONLY };

    void inflated_grid_to_pcl(openvdb::Int32Grid::ConstPtr grid,
                              std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> &pc_out,
                              InflationChannel channel = InflationChannel::ALL);
    void inflated_grid_message(const openvdb::Int32Grid::Ptr &grid,
                               sensor_msgs::msg::PointCloud2 &disp_msg,
                               InflationChannel channel = InflationChannel::ALL);

    double safe_robot_radius_xy_;
    double safe_robot_height_z_;

private: // distance map
    int max_coor_dist_;
    int max_coor_sqdist_;
    EDTGrid::Ptr dist_map_;
    std::shared_ptr<DynamicVDBEDT> grid_distance_;

    // functions for updating distance map
    rclcpp::TimerBase::SharedPtr update_edt_timer_;
    void update_edtmap(); // NOTE(ROS2): timer callback without event object.

    std_msgs::msg::ColorRGBA rainbow_color_map(double h);
    void get_slice_marker(visualization_msgs::msg::Marker &marker, int marker_id,
                          double slice, double max_sqdist);

public: // frontier map
    openvdb::BoolGrid::Ptr grid_frontier_;
    FrontierManager frontier_manager_;

private:
    void initialize_frontier_manager();
    rclcpp::TimerBase::SharedPtr update_frontier_timer_;
    void update_frontier();

    using UpdatedArea = std::unordered_set<openvdb::Coord, CoordHash>;
    std::unique_ptr<UpdatedArea> patch_write = std::make_unique<UpdatedArea>();
    std::unique_ptr<UpdatedArea> patch_read = std::make_unique<UpdatedArea>();
    std::mutex swap_lock;

    // visualization
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frontier_vis_pub;
    void vis_frontier();
    void frontier_to_pcl(openvdb::BoolGrid::ConstPtr grid,
                         std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &pc_out);

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frontier_cluster_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr viewpoints_pub_;
    void vis_frontier_clusters();
    void vis_frontier_viewpoints();

    // 0: check_frontier_6
    // 1: check_surface_frontier_6
    // 2: check_surface_frontier_26 (default)
    int frontier_type_;

    static inline bool check_frontier_6(const openvdb::FloatGrid::ConstAccessor &acc,
                                        const openvdb::Coord &ijk,
                                        const double threshold)
    {
        const float occ_thresh = static_cast<float>(threshold);
        float v = 0.f;
        if (!(acc.probeValue(ijk, v) && v < occ_thresh))
        {
            return false;
        }

        // 6-neighborhood offsets
        static const int d6[6][3] = {{+1, 0, 0}, {-1, 0, 0}, {0, +1, 0}, {0, -1, 0}, {0, 0, +1}, {0, 0, -1}};

        // if any 6-neighbor is unknown -> frontier
        for (int i = 0; i < 6; ++i)
        {
            const openvdb::Coord n = ijk.offsetBy(d6[i][0], d6[i][1], d6[i][2]);
            float nv;
            if (!acc.probeValue(n, nv))
            {
                return true; // unknown neighbor
            }
        }
        return false;
    }
    static inline bool check_surface_frontier_6(const openvdb::FloatGrid::ConstAccessor &acc,
                                                const openvdb::Coord &ijk,
                                                const double threshold)
    {
        const float occ_thresh = static_cast<float>(threshold);
        float v = 0.f;
        if (!(acc.probeValue(ijk, v) && v < occ_thresh))
        {
            return false;
        }

        // 6-neighborhood offsets
        static const int d6[6][3] = {{+1, 0, 0}, {-1, 0, 0}, {0, +1, 0}, {0, -1, 0}, {0, 0, +1}, {0, 0, -1}};

        bool has_unknown_neighbor = false;
        bool has_occ_neighbor = false;

        for (int i = 0; i < 6; ++i)
        {
            const openvdb::Coord n = ijk.offsetBy(d6[i][0], d6[i][1], d6[i][2]);
            float nv;
            if (!acc.probeValue(n, nv))
            {
                has_unknown_neighbor = true; // has unknown neighbor
            }
            else if (nv > occ_thresh)
            {
                has_occ_neighbor = true;
            }
            if (has_unknown_neighbor && has_occ_neighbor)
            {
                return true;
            }
        }
        return false;
    }

    static inline bool check_surface_frontier_26(const openvdb::FloatGrid::ConstAccessor &acc,
                                                 const openvdb::Coord &ijk,
                                                 const double threshold)
    {
        const float occ_thresh = static_cast<float>(threshold);
        float v = 0.f;
        if (!(acc.probeValue(ijk, v) && v < occ_thresh))
        {
            return false;
        }

        // 6-neighborhood offsets
        static const int d26[26][3] = {{-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1}, {-1, 0, -1}, {-1, 0, 0}, {-1, 0, 1}, {-1, 1, -1}, {-1, 1, 0}, {-1, 1, 1}, {0, -1, -1}, {0, -1, 0}, {0, -1, 1}, {0, 0, -1}, {0, 0, 1}, {0, 1, -1}, {0, 1, 0}, {0, 1, 1}, {1, -1, -1}, {1, -1, 0}, {1, -1, 1}, {1, 0, -1}, {1, 0, 0}, {1, 0, 1}, {1, 1, -1}, {1, 1, 0}, {1, 1, 1}};

        bool has_unknown_neighbor = false;
        bool has_occ_neighbor = false;

        for (int i = 0; i < 26; ++i)
        {
            const openvdb::Coord n = ijk.offsetBy(d26[i][0], d26[i][1], d26[i][2]);
            float nv;
            if (!acc.probeValue(n, nv))
            {
                has_unknown_neighbor = true; // has unknown neighbor
            }
            else if (nv > occ_thresh)
            {
                has_occ_neighbor = true;
            }
            if (has_unknown_neighbor && has_occ_neighbor)
            {
                return true;
            }
        }
        return false;
    }

private: // pose correction for lady and cow dataset
    int occu_update_count_;
    int dist_update_count_;
    int frontier_update_count_;
    Eigen::Matrix4d cur_transform_;
    Eigen::Matrix4d ref_transform_;
    Eigen::Matrix4d T_B_C_, T_D_B_;
};

#endif
