#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <bspline_opt/bspline_optimizer.h>
#include <bspline/non_uniform_bspline.h>

#include <path_searching/astar2.h>
#include <path_searching/kinodynamic_astar.h>

#include <plan_env/edt_environment.h>

#include <active_perception/frontier_finder.h>

#include <plan_manage/plan_container.hpp>

#include <ros/ros.h>

namespace fast_planner {
// Fast Planner Manager
// Key algorithms of mapping and planning are called

class FastPlannerManager {
  // SECTION stable
public:
  FastPlannerManager();
  ~FastPlannerManager();

  /* main planning interface */
  bool kinodynamicReplan(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& start_vel,
                         const Eigen::Vector3d& start_acc, const Eigen::Vector3d& end_pt,
                         const Eigen::Vector3d& end_vel, const double& time_lb = -1);
  void planExploreTraj(const vector<Eigen::Vector3d>& tour, const Eigen::Vector3d& cur_vel,
                       const Eigen::Vector3d& cur_acc, const double& time_lb = -1);
                       
  void planYawExplore(const Eigen::Vector3d& start_yaw, const double& end_yaw, bool lookfwd,
                      const double& relax_time);

  void initPlanModules(ros::NodeHandle& nh);

  bool checkTrajCollision(double& distance);
  void calcNextYaw(const double& last_yaw, double& yaw);

  PlanParameters pp_;
  // @todo:us
  LocalTrajData local_data_;
  GlobalTrajData global_data_;
  MidPlanData plan_data_;
  EDTEnvironment::Ptr edt_environment_;
  unique_ptr<Astar> path_finder_;

private:
  /* main planning algorithms & modules */
  shared_ptr<SDFMap> sdf_map_;

  unique_ptr<KinodynamicAstar> kino_path_finder_;
  vector<BsplineOptimizer::Ptr> bspline_optimizers_;

  void updateTrajInfo();

public:
  typedef shared_ptr<FastPlannerManager> Ptr;

private:
  unique_ptr<FrontierFinder> frontier_finder_;
  unique_ptr<VisibilityUtil> visib_util_;

};
}  // namespace fast_planner

#endif