// #include <fstream>
#include <exploration_manager/fast_exploration_manager.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <lkh_tsp_solver/lkh_interface.h>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/graph_map.h>
#include <plan_env/edt_environment.h>
#include <active_perception/frontier_finder.h>
#include <plan_manage/planner_manager.h>

#include <exploration_manager/expl_data.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;

namespace fast_planner
{
    // SECTION interfaces for setup and query

    FastExplorationManager::FastExplorationManager()
    {
    }

    FastExplorationManager::~FastExplorationManager()
    {
        ViewNode::astar_.reset();
        ViewNode::caster_.reset();
        ViewNode::map_.reset();
    }

    void FastExplorationManager::initialize(ros::NodeHandle &nh)
    {
        planner_manager_.reset(new FastPlannerManager);
        planner_manager_->initPlanModules(nh);
        edt_environment_ = planner_manager_->edt_environment_;
        sdf_map_ = edt_environment_->sdf_map_;
        graph_map_.reset(new PlanGraph);
        frontier_finder_.reset(new FrontierFinder(graph_map_, edt_environment_, nh));
        // view_finder_.reset(new ViewFinder(edt_environment_, nh));

        ed_.reset(new ExplorationData);
        ep_.reset(new ExplorationParam);

        nh.param("exploration/refine_local", ep_->refine_local_, true);
        nh.param("exploration/refined_num", ep_->refined_num_, -1);
        nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
        nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
        nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
        nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
        nh.param("exploration/relax_time", ep_->relax_time_, 1.0);

        // 计算当前位姿到视点的cost，最大速度、加速度、角速度、角加速度、权重
        nh.param("exploration/vm", ViewNode::vm_, -1.0);
        nh.param("exploration/am", ViewNode::am_, -1.0);
        nh.param("exploration/yd", ViewNode::yd_, -1.0);
        nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
        nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

        ViewNode::astar_.reset(new Astar);
        ViewNode::astar_->init(nh, edt_environment_);
        ViewNode::map_ = sdf_map_;

        double resolution_ = sdf_map_->getResolution();
        Eigen::Vector3d origin, size;
        sdf_map_->getRegion(origin, size);
        // @todo:us 感觉也重复了
        ViewNode::caster_.reset(new RayCaster);
        ViewNode::caster_->setParams(resolution_, origin);

        planner_manager_->path_finder_->lambda_heu_ = 1.0;
        // planner_manager_->path_finder_->max_search_time_ = 0.05;
        planner_manager_->path_finder_->max_search_time_ = 1.0;

        // Initialize TSP par file
        ofstream par_file(ep_->tsp_dir_ + "/single.par"); // planner/utils/lkh_tsp_solver/resource
        par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single.tsp\n";
        par_file << "GAIN23 = NO\n";
        par_file << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/single.txt\n";
        par_file << "RUNS = 1\n";

        Point3D init_pos(0, 0, 1.0);
        graph_map_->addVertex(init_pos);
        graph_map_->updateKdtree();

        explore_start_time_ = ros::Time::now();
    }

    int FastExplorationManager::planExploreMotion(
        const Vector3d &pos, const Vector3d &vel, const Vector3d &acc, const Vector3d &yaw)
    {
        ros::Time t1 = ros::Time::now();
        auto t2 = t1;
        ed_->views_.clear();
        ed_->global_tour_.clear();
        ed_->graph_vertices_.clear();
        ed_->graph_edges_.clear();

        // Search frontiers and group them into clusters
        frontier_finder_->searchFrontiers(pos, yaw[0]);

        double frontier_time = (ros::Time::now() - t1).toSec();
        t1 = ros::Time::now();

        // Find viewpoints (x,y,z,yaw) for all frontier clusters and get visible ones' info
        frontier_finder_->computeFrontiersToVisit();
        frontier_finder_->getFrontiers(ed_->frontiers_);
        // frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
        frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);

        Vector3d next_pos;
        double next_yaw;
        int next_id = 0;

        double topo_time = 0, view_time = 0;

        if (ed_->frontiers_.empty())
        {
            ROS_WARN("No coverable frontier.");
            Vector3d init_pos(0, 0, 1);
            if ((pos - init_pos).norm() < 1.0)
            {
                return NO_FRONTIER;
            }
            else
            {
                next_pos = init_pos;
                next_yaw = 0;
            }
        }
        else
        {
            // 获取每个簇中最佳视点的pose, yaw, average_pose
            frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_, ed_->averages_);
            for (int i = 0; i < ed_->points_.size(); ++i)
                ed_->views_.push_back(ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

            view_time = (ros::Time::now() - t1).toSec();
            t1 = ros::Time::now();

            frontier_finder_->updateTopoGraph(pos);

            // 可视化
            for (const auto &node : graph_map_->getAllVertices())
            {
                ed_->graph_vertices_.emplace_back(Vector3d(node.x(), node.y(), node.z()));
            }

            for (int i = 0; i < graph_map_->getAllEdges().size(); ++i)
            {
                auto s_vp = graph_map_->getVertex(i);
                for (const auto &end_point : graph_map_->getAllEdges()[i])
                {
                    auto t_vp = graph_map_->getVertex(end_point);
                    vector<Vector3d> graph_edges;
                    graph_edges.emplace_back(Vector3d(s_vp.x(), s_vp.y(), s_vp.z()));
                    graph_edges.emplace_back(Vector3d(t_vp.x(), t_vp.y(), t_vp.z()));
                    ed_->graph_edges_.emplace_back(graph_edges);
                }
            }

            topo_time = (ros::Time::now() - t1).toSec();
            t1 = ros::Time::now();
            frontier_finder_->getClosestFrontier(pos, vel, yaw, next_id);
            // Do global and local tour planning and retrieve the next viewpoint

            // Only 1 destination, no need to find global tour through TSP
            ed_->global_tour_ = {pos, ed_->points_[next_id]};
            ed_->refined_tour_.clear();
            ed_->refined_views1_.clear();
            ed_->refined_views2_.clear();

            if (ep_->refine_local_)
            {
                // Find the min cost viewpoint for next frontier
                ed_->refined_ids_ = {next_id};
                ed_->unrefined_points_ = {ed_->points_[next_id]};
                ed_->n_points_.clear();
                vector<vector<double>> n_yaws;
                frontier_finder_->getViewpointsInfo(
                    pos, {next_id}, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

                double min_cost = 100000;
                int min_cost_id = -1;
                vector<Vector3d> tmp_path;
                for (int i = 0; i < ed_->n_points_[0].size(); ++i)
                {
                    auto tmp_cost = ViewNode::computeCost(
                        pos, ed_->n_points_[0][i], yaw[0], n_yaws[0][i], vel, yaw[1], tmp_path);
                    if (tmp_cost < min_cost)
                    {
                        min_cost = tmp_cost;
                        min_cost_id = i;
                    }
                }
                next_pos = ed_->n_points_[0][min_cost_id];
                next_yaw = n_yaws[0][min_cost_id];
                ed_->refined_points_ = {next_pos};
                ed_->refined_views_ = {next_pos + 2.0 * Vector3d(cos(next_yaw), sin(next_yaw), 0)};
            }
            else
            {
                next_pos = ed_->points_[0];
                next_yaw = ed_->yaws_[0];
            }
    }

    double path_plan_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();
    // Plan trajectory (position and yaw) to the next viewpoint

    // Compute time lower bound of yaw and use in trajectory generation
    double diff = fabs(next_yaw - yaw[0]);
    double time_lb = min(diff, 2 * M_PI - diff) / ViewNode::yd_;

    // Generate trajectory of x,y,z
    planner_manager_->path_finder_->reset();
    if (planner_manager_->path_finder_->search(pos, next_pos) != Astar::REACH_END)
    {
        ROS_ERROR("No path to next viewpoint");
        return FAIL;
    }
    // astar2的搜索分辨率与地图分辨率独立，一个网格可能出现多个点
    // path_next_goal：到next_pos的路径点
    ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
    shortenPath(ed_->path_next_goal_);

    const double radius_far = 5.0;
    const double radius_close = 1.5;
    const double len = Astar::pathLength(ed_->path_next_goal_);

    if (len < radius_close)
    {
        // Next viewpoint is very close, no need to search kinodynamic path, just use waypoints-based
        // optimization
        planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb);
        ed_->next_goal_ = next_pos;
    }
    else if (len > radius_far)
    {
        // Next viewpoint is far away, select intermediate goal on geometric path (this also deal with
        // dead end)
        // std::cout << "Far goal." << std::endl;
        double len2 = 0.0;
        vector<Eigen::Vector3d> truncated_path = {ed_->path_next_goal_.front()};
        for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i)
        {
            auto cur_pt = ed_->path_next_goal_[i];
            len2 += (cur_pt - truncated_path.back()).norm();
            truncated_path.push_back(cur_pt);
        }
        ed_->next_goal_ = truncated_path.back();
        planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb);
        // if (!planner_manager_->kinodynamicReplan(
        //         pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
        //   return FAIL;
        // ed_->kino_path_ = planner_manager_->kino_path_finder_->getKinoTraj(0.02);
    }
    else
    {
        // Search kino path to exactly next viewpoint and optimize
        // std::cout << "Mid goal" << std::endl;
        ed_->next_goal_ = next_pos;

        if (!planner_manager_->kinodynamicReplan(
                pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
            return FAIL;
    }

    // time_lb为角度/角速度的最短时间
    if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.1)
        ROS_ERROR("Lower bound not satified!");

    double traj_plan_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    planner_manager_->planYawExplore(yaw, next_yaw, true, ep_->relax_time_);

    double yaw_plan_time = (ros::Time::now() - t1).toSec();
    double total = (ros::Time::now() - t2).toSec();
    ROS_ERROR_COND(total > 0.1, "Total time too long!!!");

    double time_now = (ros::Time::now() - explore_start_time_).toSec();

    ofstream file("/home/ncadeaku/Softwares/ViewPlanning/FUEL_sim_grav/resources/explore.txt", ios::app);
    file << "time:" << time_now << ", frontier_time:" << frontier_time << ", topo_time:" << topo_time <<  ", view_time:" << view_time
         << ", path_plan_time:" << path_plan_time << ", total:" << total << std::endl;

    return SUCCEED;
}

// 根据输入的路径点，通过判断路径点之间的距离，将距离较近的路径点进行缩短，从而得到一个更为简化的路径。
void FastExplorationManager::shortenPath(vector<Vector3d> &path)
{
    if (path.empty())
    {
        ROS_ERROR("Empty path to shorten");
        return;
    }
    // Shorten the tour, only critical intermediate points are reserved.
    const double dist_thresh = 3.0;
    vector<Vector3d> short_tour = {path.front()};
    for (int i = 1; i < path.size() - 1; ++i)
    {
        if ((path[i] - short_tour.back()).norm() > dist_thresh)
            short_tour.push_back(path[i]);
        else
        {
            // Add waypoints to shorten path only to avoid collision
            ViewNode::caster_->input(short_tour.back(), path[i + 1]);
            Eigen::Vector3i idx;
            while (ViewNode::caster_->nextId(idx) && ros::ok())
            {
                if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
                    edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
                {
                    short_tour.push_back(path[i]);
                    break;
                }
            }
        }
    }
    if ((path.back() - short_tour.back()).norm() > 1e-3)
        short_tour.push_back(path.back());

    // @todo:us 为什么要保证至少3个点
    // Ensure at least three points in the path
    if (short_tour.size() == 2)
        short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
    path = short_tour;
}

void FastExplorationManager::findGlobalTour(const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_yaw, vector<int> &indices)
{
    auto t1 = ros::Time::now();

    // Get cost matrix for current state and clusters
    Eigen::MatrixXd cost_mat;
    frontier_finder_->updateFrontierCostMatrix();
    // 构建ATSP计算所需的CostMatrix
    frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
    const int dimension = cost_mat.rows();

    double mat_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    // Write params and cost matrix to problem file
    // @todo:us 写出去再处理，可以优化
    ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
    // Problem specification part, follow the format of TSPLIB

    string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
                       "\nEDGE_WEIGHT_TYPE : "
                       "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

    // string prob_spec = "NAME : single\nTYPE : TSP\nDIMENSION : " + to_string(dimension) +
    //     "\nEDGE_WEIGHT_TYPE : "
    //     "EXPLICIT\nEDGE_WEIGHT_FORMAT : LOWER_ROW\nEDGE_WEIGHT_SECTION\n";

    prob_file << prob_spec;
    // prob_file << "TYPE : TSP\n";
    // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
    // Problem data part
    const int scale = 100;
    if (false)
    {
        // Use symmetric TSP
        for (int i = 1; i < dimension; ++i)
        {
            for (int j = 0; j < i; ++j)
            {
                int int_cost = cost_mat(i, j) * scale;
                prob_file << int_cost << " ";
            }
            prob_file << "\n";
        }
    }
    else
    {
        // Use Asymmetric TSP
        for (int i = 0; i < dimension; ++i)
        {
            for (int j = 0; j < dimension; ++j)
            {
                int int_cost = cost_mat(i, j) * scale;
                prob_file << int_cost << " ";
            }
            prob_file << "\n";
        }
    }

    prob_file << "EOF";
    prob_file.close();

    // Call LKH TSP solver
    solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

    // Read optimal tour from the tour section of result file
    ifstream res_file(ep_->tsp_dir_ + "/single.txt");
    string res;
    while (getline(res_file, res))
    {
        // Go to tour section
        if (res.compare("TOUR_SECTION") == 0)
            break;
    }

    if (false)
    {
        // Read path for Symmetric TSP formulation
        getline(res_file, res); // Skip current pose
        getline(res_file, res);
        int id = stoi(res);
        bool rev = (id == dimension); // The next node is virutal depot?

        while (id != -1)
        {
            indices.push_back(id - 2);
            getline(res_file, res);
            id = stoi(res);
        }
        if (rev)
            reverse(indices.begin(), indices.end());
        indices.pop_back(); // Remove the depot
    }
    else
    {
        // Read path for ATSP formulation
        while (getline(res_file, res))
        {
            // Read indices of frontiers in optimal tour
            int id = stoi(res);
            if (id == 1) // Ignore the current state
                continue;
            if (id == -1)
                break;
            indices.push_back(id - 2); // Idx of solver-2 == Idx of frontier
        }
    }

    res_file.close();

    // Get the path of optimal tour from path matrix
    frontier_finder_->getPathForTour(cur_pos, indices, ed_->global_tour_);

    double tsp_time = (ros::Time::now() - t1).toSec();
    ROS_WARN("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);
}

// 调用Dijkstra算法，得到一条让各段路程 之和最低的路径
void FastExplorationManager::refineLocalTour(
    const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_yaw,
    const vector<vector<Vector3d>> &n_points, const vector<vector<double>> &n_yaws,
    vector<Vector3d> &refined_pts, vector<double> &refined_yaws)
{
    double create_time, search_time, parse_time;
    auto t1 = ros::Time::now();

    // Create graph for viewpoints selection
    GraphSearch<ViewNode> g_search;
    vector<ViewNode::Ptr> last_group, cur_group;

    // Add the current state
    ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
    first->vel_ = cur_vel;
    g_search.addNode(first);
    last_group.push_back(first);
    ViewNode::Ptr final_node;

    // Add viewpoints
    // std::cout << "Local tour graph: ";
    for (int i = 0; i < n_points.size(); ++i)
    {
        // Create nodes for viewpoints of one frontier
        for (int j = 0; j < n_points[i].size(); ++j)
        {
            ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
            g_search.addNode(node);
            // Connect a node to nodes in last group
            for (auto nd : last_group)
                g_search.addEdge(nd->id_, node->id_);
            cur_group.push_back(node);

            // Only keep the first viewpoint of the last local frontier
            if (i == n_points.size() - 1)
            {
                final_node = node;
                break;
            }
        }
        // Store nodes for this group for connecting edges
        // std::cout << cur_group.size() << ", ";
        last_group = cur_group;
        cur_group.clear();
    }
    // std::cout << "" << std::endl;
    create_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    // Search optimal sequence
    vector<ViewNode::Ptr> path;
    g_search.DijkstraSearch(first->id_, final_node->id_, path);

    search_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    // Return searched sequence
    for (int i = 1; i < path.size(); ++i)
    {
        refined_pts.push_back(path[i]->pos_);
        refined_yaws.push_back(path[i]->yaw_);
    }

    // Extract optimal local tour (for visualization)
    ed_->refined_tour_.clear();
    ed_->refined_tour_.push_back(cur_pos);
    ViewNode::astar_->lambda_heu_ = 1.0;
    ViewNode::astar_->setResolution(0.2);
    for (auto pt : refined_pts)
    {
        vector<Vector3d> path;
        if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
            ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
        else
            ed_->refined_tour_.push_back(pt);
    }
    ViewNode::astar_->lambda_heu_ = 10000;

    parse_time = (ros::Time::now() - t1).toSec();
    // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time, search_time, parse_time);
}

} // namespace fast_planner
