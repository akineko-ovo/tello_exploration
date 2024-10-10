#ifndef _MAP_ROS_H
#define _MAP_ROS_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <memory>
#include <random>

using std::default_random_engine;
using std::normal_distribution;
using std::shared_ptr;

namespace fast_planner
{
    class SDFMap;

    class MapROS
    {
    public:
        MapROS();
        ~MapROS();
        void setMap(SDFMap *map);
        void init();

    private:
        //   void depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
        //                          const nav_msgs::OdometryConstPtr& pose);
        void depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                               const geometry_msgs::PoseStampedConstPtr &pose);
        void updateESDFCallback(const ros::TimerEvent & /*event*/);
        void visCallback(const ros::TimerEvent & /*event*/);

        void publishMapAll();
        void publishMapLocal();
        void publishESDF();
        void publishUpdateRange();
        void publishUnknown();
        void publishDepth();

        void proessDepthImage();

        SDFMap *map_;
        // may use ExactTime?
        //   typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
        //       SyncPolicyImagePose;
        //   typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
            SyncPolicyImagePose;
        typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;

        ros::NodeHandle node_;
        shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
        // shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> pose_sub_;
        shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
        SynchronizerImagePose sync_image_pose_;

        ros::Publisher map_local_pub_, map_local_inflate_pub_, esdf_pub_, map_all_pub_, unknown_pub_,
            update_range_pub_, depth_pub_;
        ros::Timer esdf_timer_, vis_timer_;

        // params, depth projection
        Eigen::Matrix3d K_depth_;
        double cx_, cy_, fx_, fy_;
        double depth_filter_maxdist_, depth_filter_mindist_;
        int depth_filter_margin_;
        double k_depth_scaling_factor_;
        int skip_pixel_;
        string frame_id_;
        // msg publication
        double esdf_slice_height_;
        double visualization_truncate_height_, visualization_truncate_low_;
        bool show_esdf_time_, show_occ_time_;
        bool show_all_map_;

        // data
        // flags of map state
        bool local_updated_, esdf_need_update_;
        // input
        Eigen::Vector3d camera_pos_;
        Eigen::Quaterniond camera_q_;
        unique_ptr<cv::Mat> depth_image_;
        vector<Eigen::Vector3d> proj_points_;
        int proj_points_cnt_;
        double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
        // 更新次数
        int fuse_num_, esdf_num_;
        pcl::PointCloud<pcl::PointXYZ> point_cloud_;

        // 正态分布对象，均值为0，标准差为0.1
        normal_distribution<double> rand_noise_;
        // 随机数生成器
        default_random_engine eng_;

        ros::Time map_start_time_;

        friend SDFMap;
    };
}

#endif