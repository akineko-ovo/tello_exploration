#include <active_perception/frontier_finder.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>
// #include <path_searching/astar2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plan_env/graph_map.h>
#include <plan_env/edt_environment.h>
#include <active_perception/perception_utils.h>
#include <active_perception/graph_node.h>

// use PCL region growing segmentation
// #include <pcl/point_types.h>
// #include <pcl/search/search.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Eigenvalues>

namespace fast_planner
{
    FrontierFinder::FrontierFinder(const shared_ptr<PlanGraph> &graph_map, const EDTEnvironment::Ptr &edt, ros::NodeHandle &nh)
    {
        this->graph_map_ = graph_map;
        this->edt_env_ = edt;
        int voxel_num = edt->sdf_map_->getVoxelNum();
        frontier_flag_ = vector<char>(voxel_num, 0);
        fill(frontier_flag_.begin(), frontier_flag_.end(), 0);
        // 簇中 grid 最少个数
        nh.param("frontier/cluster_min", cluster_min_, -1);
        // 簇在xy面上离簇的中心的最远距离，超过会被进一步划分
        nh.param("frontier/cluster_size_xy", cluster_size_xy_, -1.0);
        nh.param("frontier/cluster_size_z", cluster_size_z_, -1.0);
        nh.param("frontier/cluster_size", cluster_size_, -1.0);
        // @todo:us 视点和当前位姿的最小距离
        nh.param("frontier/min_candidate_dist", min_candidate_dist_, -1.0);
        // 生成视点周围离UNKONWN的最小安全距离
        nh.param("frontier/min_candidate_clearance", min_candidate_clearance_, -1.0);
        // 视点生成角度间隔，最小半径，最大半径，半径间隔数量
        nh.param("frontier/candidate_dphi", candidate_dphi_, -1.0);
        nh.param("frontier/candidate_dtheta", candidate_dtheta_, -1.0);
        nh.param("frontier/candidate_rmax", candidate_rmax_, -1.0);
        nh.param("frontier/candidate_rmin", candidate_rmin_, -1.0);
        nh.param("frontier/candidate_rnum", candidate_rnum_, -1);
        // @todo:us
        nh.param("frontier/down_sample", down_sample_, -1);
        // 生成视点观察到簇中cell的最少数量
        nh.param("frontier/min_visib_num", min_visib_num_, -1);
        // 生成视点观察到簇中cell的最小百分比，满足时视为完成覆盖
        nh.param("frontier/min_view_finish_fraction", min_view_finish_fraction_, -1.0);

        raycaster_.reset(new RayCaster);
        resolution_ = edt_env_->sdf_map_->getResolution();
        Eigen::Vector3d origin, size;
        edt_env_->sdf_map_->getRegion(origin, size);
        raycaster_->setParams(resolution_, origin);

        percep_utils_.reset(new PerceptionUtils(nh));
        percep_utils_->getClusterCenter(cluster_center_, cluster_size_);
    }

    FrontierFinder::~FrontierFinder()
    {
    }

    // results are saved in tmp_frontiers_
    void FrontierFinder::searchFrontiers(const Vector3d &pos, const double &yaw)
    {
        ros::Time t1 = ros::Time::now();
        tmp_frontiers_.clear();

        // Bounding box of updated region
        Vector3d update_min, update_max;
        edt_env_->sdf_map_->getUpdatedBox(update_min, update_max, true);

        const double connectable_range_ = 5.0;
        const double min_interval = 1.414;
        Point3D current_position(pos.x() + connectable_range_ / 2.0 * cos(yaw), pos.y() + connectable_range_ / 2.0 * sin(yaw), pos.z());

        // 获取当前位姿一定范围内的所有节点
        Point3DQueue neighbor_vertexs;
        std::vector<int> neighbor_vertex_ids = graph_map_->getNeighborVertexsIDs(graph_map_->kd_tree_, current_position,
                                                                                 connectable_range_,
                                                                                 neighbor_vertexs);

        // Removed changed frontiers in updated map
        auto resetFlag = [&](list<Frontier>::iterator &iter, list<Frontier> &frontiers)
        {
            Eigen::Vector3i idx;
            for (auto cell : iter->cells_)
            {
                edt_env_->sdf_map_->posToIndex(cell, idx);
                frontier_flag_[toadr(idx)] = 0;
            }
            // 返回指向被删除元素之后位置的迭代器
            iter = frontiers.erase(iter);
        };

        // std::cout << "Before remove: " << frontiers_.size() << std::endl;
        Point3DQueue new_vertices;
        std::vector<int> new_vertex_ids;

        removed_ids_.clear();
        int rmv_idx = 0;
        for (auto iter = frontiers_.begin(); iter != frontiers_.end();)
        {
            if (haveOverlap(iter->average_, update_min, update_max) &&
                isFrontierChanged(*iter))
            {
                resetFlag(iter, frontiers_);
                removed_ids_.push_back(rmv_idx);

                if (edt_env_->sdf_map_->getInflateOccupancy(iter->average_) == 1 || isNearUnknown(iter->average_))
                    continue;

                Point3D view(iter->average_.x(), iter->average_.y(), iter->average_.z());
                bool addable = true;

                for (int i = 0; i < neighbor_vertex_ids.size(); ++i)
                {
                    if (view.distance(neighbor_vertexs[i]) < min_interval && graph_map_->isVertexValide(neighbor_vertex_ids[i]))
                    {
                        Eigen::Vector3d end(neighbor_vertexs[i].x(), neighbor_vertexs[i].y(), neighbor_vertexs[i].z());
                        Eigen::Vector3i idx;
                        raycaster_->input(iter->average_, end);

                        bool isCollision = false;
                        while (raycaster_->nextId(idx))
                        {
                            // Hit obstacle, stop the ray
                            if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 || !edt_env_->sdf_map_->isInBox(idx) ||
                                edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
                            {
                                isCollision = true;
                                break;
                            }
                        }

                        if (!isCollision)
                        {
                            addable = false;
                            break;
                        }
                    }
                }

                if (addable)
                {
                    int graph_id = graph_map_->addVertex(view);
                    neighbor_vertexs.emplace_back(view);
                    neighbor_vertex_ids.emplace_back(graph_id);
                    new_vertices.emplace_back(view);
                    new_vertex_ids.emplace_back(graph_id);
                }
            }
            else
            {
                ++rmv_idx;
                ++iter;
            }
        }
        // std::cout << "After remove: " << frontiers_.size() << std::endl;
        // @todo:us dormant_frontiers 的作用
        for (auto iter = dormant_frontiers_.begin(); iter != dormant_frontiers_.end();)
        {
            if (haveOverlap(iter->average_, update_min, update_max) &&
                isFrontierChanged(*iter))
                resetFlag(iter, dormant_frontiers_);
            else
                ++iter;
        }

        for (int i = 0; i < new_vertex_ids.size(); ++i)
        {
            Point3D &point = new_vertices[i];
            int point_id = new_vertex_ids[i];

            Point3DQueue neighbors_vertexs;
            std::vector<int> neighbors_ids = graph_map_->getNeighborVertexsIDs(graph_map_->kd_tree_, point,
                                                                               connectable_range_,
                                                                               neighbors_vertexs);

            vector<Point3D> vertices;
            for (int i = 0; i < neighbors_ids.size(); ++i)
            {
                int id = neighbors_ids[i];
                if (point_id == id)
                    continue;

                Point3D vertex = neighbors_vertexs[i];
                // 仅当两点之间无障碍且未超过连接数上限时，建立双向连接
                Eigen::Vector3d end(vertex.x(), vertex.y(), vertex.z());
                Eigen::Vector3i idx;
                raycaster_->input(Eigen::Vector3d(point.x(), point.y(), point.z()), end);

                bool isCollision = false;
                while (raycaster_->nextId(idx))
                {
                    // Hit obstacle, stop the ray
                    bool unknown = edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN;
                    if (unknown || edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 || !edt_env_->sdf_map_->isInBox(idx))
                    {
                        isCollision = true;
                        break;
                    }
                }

                if (!isCollision)
                {
                    bool isNew = true;
                    for (int j = 0; j < vertices.size(); ++j)
                    {
                        auto dir0 = vertices[j] - point;
                        auto dir1 = vertex - point;
                        if (Point3D::dot(dir0, dir1) < cos(60 / 180 * M_PI))
                        {
                            if (dir1.norm() < dir0.norm())
                            {
                                vertices[j] = vertex;
                                isNew = false;
                                break;
                            }
                        }
                    }

                    if (isNew)
                    {
                        vertices.emplace_back(vertex);
                    }
                }
            }

            for (const auto &vertex : vertices)
            {
                int vertex_id = -1;
                graph_map_->getPointId(vertex, vertex_id);
                graph_map_->addTwoWayEdge(point_id, vertex_id);
            }
        }

        graph_map_->updateKdtree();
        ROS_WARN_THROTTLE(5.0, "Graph t: %lf", (ros::Time::now() - t1).toSec());

        // Search new frontier within box slightly inflated from updated box
        // @todo:us 为什么 z 轴只给0.5
        Vector3d search_min = update_min - Vector3d(1, 1, 0.5);
        Vector3d search_max = update_max + Vector3d(1, 1, 0.5);
        Vector3d box_min, box_max;
        edt_env_->sdf_map_->getBox(box_min, box_max);
        for (int k = 0; k < 3; ++k)
        {
            search_min[k] = max(search_min[k], box_min[k]);
            search_max[k] = min(search_max[k], box_max[k]);
        }

        Eigen::Vector3i min_id, max_id;
        edt_env_->sdf_map_->posToIndex(search_min, min_id);
        edt_env_->sdf_map_->posToIndex(search_max, max_id);

        cluster_center_cur_ = getClusterCenter(pos, yaw);

        for (int x = min_id(0); x <= max_id(0); ++x)
            for (int y = min_id(1); y <= max_id(1); ++y)
                for (int z = min_id(2); z <= max_id(2); ++z)
                {
                    // Scanning the updated region to find seeds of frontiers
                    Eigen::Vector3i cur(x, y, z);
                    if (frontier_flag_[toadr(cur)] == 0 && knownfree(cur) && isNeighborUnknown(cur))
                    {
                        // Expand from the seed cell to find a complete frontier cluster
                        expandFrontier(cur);
                    }
                }
        // splitLargeFrontiers(tmp_frontiers_);

        ROS_WARN_THROTTLE(5.0, "Frontier t: %lf", (ros::Time::now() - t1).toSec());
    }

    void FrontierFinder::expandFrontier(const Eigen::Vector3i &first)
    {
        vector<Frontier> frontiers;
        frontiers.resize(cluster_center_cur_.size());

        queue<Eigen::Vector3i> cell_queue;

        Vector3d pos;
        edt_env_->sdf_map_->indexToPos(first, pos);
        cell_queue.push(first);

        Eigen::Vector3d n;

        while (!cell_queue.empty())
        {
            auto cur = cell_queue.front();
            cell_queue.pop();
            auto nbrs = allNeighbors(cur);
            for (auto nbr : nbrs)
            {
                // Qualified cell should be inside bounding box and frontier cell not clustered
                int adr = toadr(nbr);
                if (frontier_flag_[adr] == 1 || !edt_env_->sdf_map_->isInBox(nbr) ||
                    !(knownfree(nbr) && isNeighborUnknown(nbr, n)))
                    continue;

                edt_env_->sdf_map_->indexToPos(nbr, pos);
                if (pos[2] < 0.4)
                    continue; // Remove noise close to ground

                int cluster_id = -1;
                double min_dist = INT_MAX;
                for (int cid = 0; cid < cluster_center_cur_.size(); ++cid)
                {
                    double dist = (pos - cluster_center_cur_[cid]).norm();
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        cluster_id = cid;
                    }
                }

                if (cluster_id == -1)
                    continue;
                // if ((pos.head<2>() - (frontiers[cluster_id].average_ / frontiers[cluster_id].cells_.size()).head<2>()).norm() > cluster_size_xy_) {
                //     continue;
                // }

                frontiers[cluster_id].cells_.emplace_back(pos);
                frontiers[cluster_id].normal_ += n;
                frontiers[cluster_id].average_ += pos;

                for (int i = 0; i < 3; ++i)
                {
                    frontiers[cluster_id].box_min_[i] = min(frontiers[cluster_id].box_min_[i], pos[i]);
                    frontiers[cluster_id].box_max_[i] = max(frontiers[cluster_id].box_max_[i], pos[i]);
                }

                cell_queue.push(nbr);
                frontier_flag_[adr] = 1;
            }
        }

        for (auto frontier : frontiers)
        {
            if (frontier.cells_.size() > cluster_min_)
            {
                // Compute detailed info
                frontier.normal_.normalize();
                frontier.average_ /= frontier.cells_.size();
                downsample(frontier.cells_, frontier.filtered_cells_);
                tmp_frontiers_.push_back(frontier);
            }
        }
    }

    vector<Vector3d> FrontierFinder::getClusterCenter(const Vector3d &cur_pos, const double &cur_yaw)
    {
        Eigen::Matrix3d R_wc;
        R_wc << cos(cur_yaw), -sin(cur_yaw), 0, sin(cur_yaw), cos(cur_yaw), 0, 0, 0, 1;

        Eigen::Vector3d t_wc = cur_pos;

        vector<Eigen::Vector3d> cluster_center;
        cluster_center.resize(cluster_center_.size());

        for (int i = 0; i < cluster_center_.size(); ++i)
        {
            auto center = R_wc * cluster_center_[i] + t_wc;
            cluster_center[i] = center;
        }

        return cluster_center;
    }

    void FrontierFinder::splitLargeFrontiers(list<Frontier> &frontiers)
    {
        list<Frontier> splits, tmps;
        for (auto it = frontiers.begin(); it != frontiers.end(); ++it)
        {
            // @todo:us 不需要 to be split vertically.
            // Check if each frontier needs to be split horizontally
            if (splitHorizontally(*it, splits))
            {
                tmps.insert(tmps.end(), splits.begin(), splits.end());
                splits.clear();
            }
            else
                tmps.push_back(*it);
        }
        frontiers = tmps;
    }

    bool FrontierFinder::splitHorizontally(const Frontier &frontier, list<Frontier> &splits)
    {
        // Split a frontier into small piece if it is too large
        auto mean = frontier.average_.head<2>();
        bool need_split = false;
        for (auto cell : frontier.filtered_cells_)
        {
            if ((cell.head<2>() - mean).norm() > cluster_size_xy_)
            {
                need_split = true;
                break;
            }
        }
        if (!need_split)
            return false;

        // Compute principal component
        // Covariance matrix of cells
        Eigen::Matrix2d cov;
        cov.setZero();
        for (auto cell : frontier.filtered_cells_)
        {
            Eigen::Vector2d diff = cell.head<2>() - mean;
            cov += diff * diff.transpose();
        }
        cov /= double(frontier.filtered_cells_.size());

        // Find eigenvector corresponds to maximal eigenvector
        Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
        auto values = es.eigenvalues().real();
        auto vectors = es.eigenvectors().real();
        int max_idx;
        double max_eigenvalue = -1000000;
        for (int i = 0; i < values.rows(); ++i)
        {
            if (values[i] > max_eigenvalue)
            {
                max_idx = i;
                max_eigenvalue = values[i];
            }
        }
        Eigen::Vector2d first_pc = vectors.col(max_idx);
        // std::cout << "max idx: " << max_idx << std::endl;
        // std::cout << "mean: " << mean.transpose() << ", first pc: " << first_pc.transpose() << std::endl;

        // Split the frontier into two groups along the first PC
        Frontier ftr1, ftr2;
        for (auto cell : frontier.cells_)
        {
            if ((cell.head<2>() - mean).dot(first_pc) >= 0)
                ftr1.cells_.push_back(cell);
            else
                ftr2.cells_.push_back(cell);
        }
        computeFrontierInfo(ftr1);
        computeFrontierInfo(ftr2);

        // Recursive call to split frontier that is still too large
        list<Frontier> splits2;
        if (splitHorizontally(ftr1, splits2))
        {
            splits.insert(splits.end(), splits2.begin(), splits2.end());
            splits2.clear();
        }
        else
            splits.push_back(ftr1);

        if (splitHorizontally(ftr2, splits2))
            splits.insert(splits.end(), splits2.begin(), splits2.end());
        else
            splits.push_back(ftr2);

        return true;
    }

    bool FrontierFinder::isInBoxes(
        const vector<pair<Vector3d, Vector3d>> &boxes, const Eigen::Vector3i &idx)
    {
        Vector3d pt;
        edt_env_->sdf_map_->indexToPos(idx, pt);
        for (auto box : boxes)
        {
            // Check if contained by a box
            bool inbox = true;
            for (int i = 0; i < 3; ++i)
            {
                inbox = inbox && pt[i] > box.first[i] && pt[i] < box.second[i];
                if (!inbox)
                    break;
            }
            if (inbox)
                return true;
        }
        return false;
    }

    void FrontierFinder::updateTopoGraph(const Vector3d &cur_pos)
    {
        const double connectable_range_ = 5.0;
        const double min_interval = 1.414;
        const int connectable_num_ = 5;
        Point3D current_position(cur_pos.x(), cur_pos.y(), cur_pos.z());

        // 获取当前位姿一定范围内的所有节点
        Point3DQueue neighbor_vertexs;
        std::vector<int> neighbor_vertex_ids = graph_map_->getNeighborVertexsIDs(graph_map_->kd_tree_, current_position,
                                                                                 connectable_range_,
                                                                                 neighbor_vertexs);

        Point3DQueue new_vertices;
        std::vector<int> new_vertex_ids;

        // 添加并获取当前位姿的节点id，在 min_interval 以内且不碰撞才不加入
        bool addable = true;
        int current_position_id;
        for (int i = 0; i < neighbor_vertex_ids.size(); ++i)
        {
            if (current_position.distance(neighbor_vertexs[i]) < min_interval && graph_map_->isVertexValide(neighbor_vertex_ids[i]))
            {
                Eigen::Vector3d end(neighbor_vertexs[i].x(), neighbor_vertexs[i].y(), neighbor_vertexs[i].z());
                Eigen::Vector3i idx;
                raycaster_->input(cur_pos, end);

                bool isCollision = false;
                while (raycaster_->nextId(idx))
                {
                    // Hit obstacle, stop the ray
                    if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 || !edt_env_->sdf_map_->isInBox(idx) ||
                        edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
                    {
                        isCollision = true;
                        break;
                    }
                }

                if (!isCollision)
                {
                    addable = false;
                    current_position_id = neighbor_vertex_ids[i];
                    break;
                }
            }
        }

        if (addable)
        {
            current_position_id = graph_map_->addVertex(current_position);
            neighbor_vertexs.emplace_back(current_position);
            new_vertices.emplace_back(current_position);
            new_vertex_ids.emplace_back(current_position_id);
        }

        current_position_id_ = current_position_id;

        // 遍历新增frontier并添加节点
        std::list<Frontier>::reverse_iterator rit = frontiers_.rbegin();
        for (int i = 0; i < new_frontiers_num_ && rit != frontiers_.rend(); ++i, ++rit)
        {
            Frontier &frontier = *rit;
            Viewpoint &repre_view = frontier.viewpoints_.front();
            Point3D view(repre_view.pos_.x(), repre_view.pos_.y(), repre_view.pos_.z());
            addable = true;

            for (int i = 0; i < neighbor_vertex_ids.size(); ++i)
            {
                if (view.distance(neighbor_vertexs[i]) < min_interval && graph_map_->isVertexValide(neighbor_vertex_ids[i]))
                {
                    Eigen::Vector3d end(neighbor_vertexs[i].x(), neighbor_vertexs[i].y(), neighbor_vertexs[i].z());
                    Eigen::Vector3i idx;
                    raycaster_->input(repre_view.pos_, end);

                    bool isCollision = false;
                    while (raycaster_->nextId(idx))
                    {
                        // Hit obstacle, stop the ray
                        if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 || !edt_env_->sdf_map_->isInBox(idx) ||
                            edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
                        {
                            isCollision = true;
                            break;
                        }
                    }

                    if (!isCollision)
                    {
                        frontier.graph_id_ = neighbor_vertex_ids[i];
                        addable = false;
                        break;
                    }
                }
            }

            if (addable)
            {
                frontier.graph_id_ = graph_map_->addVertex(view);
                neighbor_vertexs.emplace_back(view);
                neighbor_vertex_ids.emplace_back(frontier.graph_id_);
                new_vertices.emplace_back(view);
                new_vertex_ids.emplace_back(frontier.graph_id_);
            }
        }

        for (int i = 0; i < new_vertex_ids.size(); ++i)
        {
            Point3D &point = new_vertices[i];
            int point_id = new_vertex_ids[i];

            Point3DQueue neighbors_vertexs;
            std::vector<int> neighbors_ids = graph_map_->getNeighborVertexsIDs(graph_map_->kd_tree_, point,
                                                                               connectable_range_,
                                                                               neighbors_vertexs);

            vector<Point3D> vertices;
            vertices.clear();
            for (int i = 0; i < neighbors_ids.size(); ++i)
            {
                int id = neighbors_ids[i];
                if (point_id == id)
                    continue;

                Point3D vertex = neighbors_vertexs[i];
                // 仅当两点之间无障碍且未超过连接数上限时，建立双向连接
                Eigen::Vector3d end(vertex.x(), vertex.y(), vertex.z());

                bool isNew = true;
                int similar_id = -1;
                for (int j = 0; j < vertices.size(); ++j)
                {
                    auto dir0 = vertices[j] - point;
                    auto dir1 = vertex - point;
                    if (Point3D::dot(dir0, dir1) < cos(60 / 180 * M_PI))
                    {
                        if (dir1.norm() < dir0.norm())
                        {
                            isNew = false;
                        }
                        else
                        {
                            similar_id = j;
                        }
                        break;
                    }
                }

                if (!isNew)
                    continue;

                Eigen::Vector3i idx;
                raycaster_->input(Eigen::Vector3d(point.x(), point.y(), point.z()), end);

                bool isCollision = false;
                while (raycaster_->nextId(idx))
                {
                    // Hit obstacle, stop the ray
                    bool unknown = edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN;
                    if (unknown || edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 || !edt_env_->sdf_map_->isInBox(idx))
                    {
                        isCollision = true;
                        break;
                    }
                }

                if (!isCollision)
                {
                    if (similar_id != -1)
                    {
                        vertices[similar_id] = vertex;
                    }
                    else
                    {
                        vertices.emplace_back(vertex);
                    }
                }
            }

            for (const auto &vertex : vertices)
            {
                int vertex_id = -1;
                graph_map_->getPointId(vertex, vertex_id);
                graph_map_->addTwoWayEdge(point_id, vertex_id);
            }

            std::cout << "point: " << point.x() << ", " << point.y() << ", " << point.z() <<
            ", vertices.size(): " << vertices.size() << ", neighbors_ids.size(): " << neighbors_ids.size() << std::endl;
        }

        graph_map_->updateKdtree();
    }

    void FrontierFinder::getClosestFrontier(
        const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d cur_yaw,
        int &next_id)
    {
        Vector3d box_min, box_max;
        edt_env_->sdf_map_->getBox(box_min, box_max);
        Vector3d center = (box_max + box_min) / 2.0;
        Vector3d init_pos(0.0, 0.0, 1.0);

        double grav_cur;
        vector<Point3D> path;
        graph_map_->computeCost(0, current_position_id_, 0, 0, Point3D(0, 0, 0), path, grav_cur);
        if (grav_cur < 0.0)
        {
            vector<Vector3d> path;
            ViewNode::computeCost(init_pos, cur_pos, 0, 0, Vector3d::Zero(), 0, path, grav_cur);
        }

        double min_cost = (box_max(0) - box_min(0)) * (box_max(1) - box_min(1)) * (box_max(2) - box_min(2));
        for (auto ftr : frontiers_)
        {
            Viewpoint &vp = ftr.viewpoints_.front();
            double time = 0.0;
            double time_cost = graph_map_->computeCost(current_position_id_, ftr.graph_id_, cur_yaw[0], vp.yaw_, Point3D(cur_vel.x(), cur_vel.y(), cur_vel.z()), path, time);
            if (time < 0.0)
            {
                vector<Vector3d> path;
                time_cost = ViewNode::computeCost(cur_pos, vp.pos_, cur_yaw[0], vp.yaw_, cur_vel, cur_yaw[1], path, time);
                if (time < 0.0)
                {
                    std::cout << vp.pos_.transpose() << " fail! time: " << time << std::endl;
                    continue;
                }
            }

            if (time_cost > min_cost)
                continue;

            double grav = 0.0;
            double test = graph_map_->computeCost(0, ftr.graph_id_, 0, 0, Point3D(0, 0, 0), path, grav);
            if (grav < 0.0)
            {
                vector<Vector3d> path;
                ViewNode::computeCost(init_pos, vp.pos_, 0, 0, Vector3d::Zero(), 0, path, grav);
                if (grav < 0.0)
                {
                    std::cout << vp.pos_.transpose() << " fail! grav: " << grav << ", test: " << test << std::endl;
                    continue;
                }
            }

            double grav_cost = 2.0 * ftr.grav_cost_;
            if (grav_cur / grav > 1.2)
            {
                grav_cost += grav_cur;
            }
            else
            {
                grav_cost -= max(grav - grav_cur, 0.0);
            }

            double cost = time_cost + grav_cost;
            // std::cout << "pos: " << vp.pos_.transpose() << ", time_cost: " << time_cost << ", grav: " << grav_cur / grav << ", " <<  grav_cost << std::endl;

            if (cost < min_cost)
            {
                min_cost = cost;
                next_id = ftr.id_;
            }
        }
    }

    // 更新frontiers之间cost
    void FrontierFinder::updateFrontierCostMatrix()
    {
        // std::cout << "cost mat size before remove: " << std::endl;
        // for (auto ftr : frontiers_)
        //   std::cout << "(" << ftr.costs_.size() << "," << ftr.paths_.size() << "), ";
        // std::cout << "" << std::endl;

        // std::cout << "cost mat size remove: " << std::endl;
        if (!removed_ids_.empty())
        {
            // Delete path and cost for removed clusters
            for (auto it = frontiers_.begin(); it != first_new_ftr_; ++it)
            {
                auto cost_iter = it->costs_.begin();
                auto path_iter = it->paths_.begin();
                int iter_idx = 0;
                for (int i = 0; i < removed_ids_.size(); ++i)
                {
                    // Step iterator to the item to be removed
                    while (iter_idx < removed_ids_[i])
                    {
                        ++cost_iter;
                        ++path_iter;
                        ++iter_idx;
                    }
                    cost_iter = it->costs_.erase(cost_iter);
                    path_iter = it->paths_.erase(path_iter);
                }
                // std::cout << "(" << it->costs_.size() << "," << it->paths_.size() << "), ";
            }
            removed_ids_.clear();
        }
        // std::cout << "" << std::endl;

        auto updateCost = [](const list<Frontier>::iterator &it1, const list<Frontier>::iterator &it2)
        {
            // std::cout << "(" << it1->id_ << "," << it2->id_ << "), ";
            // Search path from old cluster's top viewpoint to new cluster'
            Viewpoint &vui = it1->viewpoints_.front();
            Viewpoint &vuj = it2->viewpoints_.front();
            vector<Vector3d> path_ij;
            double cost_ij = ViewNode::computeCost(
                vui.pos_, vuj.pos_, vui.yaw_, vuj.yaw_, Vector3d(0, 0, 0), 0, path_ij);
            // Insert item for both old and new clusters
            it1->costs_.push_back(cost_ij);
            it1->paths_.push_back(path_ij);
            reverse(path_ij.begin(), path_ij.end());
            it2->costs_.push_back(cost_ij);
            it2->paths_.push_back(path_ij);
        };

        // std::cout << "cost mat add: " << std::endl;
        // Compute path and cost between old and new clusters
        for (auto it1 = frontiers_.begin(); it1 != first_new_ftr_; ++it1)
            for (auto it2 = first_new_ftr_; it2 != frontiers_.end(); ++it2)
                updateCost(it1, it2);

        // Compute path and cost between new clusters
        for (auto it1 = first_new_ftr_; it1 != frontiers_.end(); ++it1)
            for (auto it2 = it1; it2 != frontiers_.end(); ++it2)
            {
                if (it1 == it2)
                {
                    // std::cout << "(" << it1->id_ << "," << it2->id_ << "), ";
                    it1->costs_.push_back(0);
                    it1->paths_.push_back({});
                }
                else
                    updateCost(it1, it2);
            }
        // std::cout << "" << std::endl;
        // std::cout << "cost mat size final: " << std::endl;
        // for (auto ftr : frontiers_)
        //   std::cout << "(" << ftr.costs_.size() << "," << ftr.paths_.size() << "), ";
        // std::cout << "" << std::endl;
    }

    void FrontierFinder::mergeFrontiers(Frontier &ftr1, const Frontier &ftr2)
    {
        // Merge ftr2 into ftr1
        ftr1.average_ =
            (ftr1.average_ * double(ftr1.cells_.size()) + ftr2.average_ * double(ftr2.cells_.size())) /
            (double(ftr1.cells_.size() + ftr2.cells_.size()));
        ftr1.cells_.insert(ftr1.cells_.end(), ftr2.cells_.begin(), ftr2.cells_.end());
        computeFrontierInfo(ftr1);
    }

    bool FrontierFinder::canBeMerged(const Frontier &ftr1, const Frontier &ftr2)
    {
        Vector3d merged_avg =
            (ftr1.average_ * double(ftr1.cells_.size()) + ftr2.average_ * double(ftr2.cells_.size())) /
            (double(ftr1.cells_.size() + ftr2.cells_.size()));
        // Check if it can merge two frontier without exceeding size limit
        for (auto c1 : ftr1.cells_)
        {
            auto diff = c1 - merged_avg;
            if (diff.head<2>().norm() > cluster_size_xy_ || diff[2] > cluster_size_z_)
                return false;
        }
        for (auto c2 : ftr2.cells_)
        {
            auto diff = c2 - merged_avg;
            if (diff.head<2>().norm() > cluster_size_xy_ || diff[2] > cluster_size_z_)
                return false;
        }
        return true;
    }

    bool FrontierFinder::haveOverlap(
        const Vector3d &min1, const Vector3d &max1, const Vector3d &min2, const Vector3d &max2)
    {
        // Check if two box have overlap part
        Vector3d bmin, bmax;
        for (int i = 0; i < 3; ++i)
        {
            bmin[i] = max(min1[i], min2[i]);
            bmax[i] = min(max1[i], max2[i]);
            if (bmin[i] > bmax[i] + 1e-3)
                return false;
        }
        return true;
    }

    bool FrontierFinder::haveOverlap(
        const Vector3d &average, const Vector3d &min, const Vector3d &max)
    {
        for (int i = 0; i < 3; ++i)
        {
            if (average[i] < min[i] - 1e-3 || average[i] > max[i] + 1e-3)
            {
                return false;
            }
        }
        return true;
    }

    bool FrontierFinder::isFrontierChanged(const Frontier &ft)
    {
        for (auto cell : ft.cells_)
        {
            Eigen::Vector3i idx;
            edt_env_->sdf_map_->posToIndex(cell, idx);
            // 不为 free 或 6邻域没有unkonwn
            if (!(knownfree(idx) && isNeighborUnknown(idx)))
                return true;
        }
        return false;
    }

    void FrontierFinder::computeFrontierInfo(Frontier &ftr)
    {
        // Compute average position and bounding box of cluster
        ftr.average_.setZero();
        ftr.box_max_ = ftr.cells_.front();
        ftr.box_min_ = ftr.cells_.front();
        for (auto cell : ftr.cells_)
        {
            ftr.average_ += cell;
            for (int i = 0; i < 3; ++i)
            {
                ftr.box_min_[i] = min(ftr.box_min_[i], cell[i]);
                ftr.box_max_[i] = max(ftr.box_max_[i], cell[i]);
            }
        }
        ftr.average_ /= double(ftr.cells_.size());

        // Compute downsampled cluster
        downsample(ftr.cells_, ftr.filtered_cells_);
    }

    // 对新增frontier信息进行更新
    void FrontierFinder::computeFrontiersToVisit()
    {
        Vector3d box_min, box_max;
        edt_env_->sdf_map_->getBox(box_min, box_max);
        Vector3d center = (box_max + box_min) / 2.0;

        first_new_ftr_ = frontiers_.end();
        int new_num = 0;
        int new_dormant_num = 0;
        // Try find viewpoints for each cluster and categorize them according to viewpoint number
        for (auto &tmp_ftr : tmp_frontiers_)
        {
            // Search viewpoints around frontier
            sampleViewpoints(tmp_ftr);
            if (!tmp_ftr.viewpoints_.empty())
            {
                ++new_num;
                list<Frontier>::iterator inserted = frontiers_.insert(frontiers_.end(), tmp_ftr);
                // Sort the viewpoints by coverage fraction, best view in front
                sort(
                    inserted->viewpoints_.begin(), inserted->viewpoints_.end(),
                    [](const Viewpoint &v1, const Viewpoint &v2)
                    { return v1.visib_num_ > v2.visib_num_; });

                Vector3d region;
                for (int i = 0; i < 2; ++i)
                {
                    region(i) = (inserted->average_(i) - center(i)) > 0 ? box_max(i) : box_min(i);
                    region(i) = (region(i) - inserted->average_(i)) / region(i);
                    inserted->grav_cost_ += region(i);
                }

                if (first_new_ftr_ == frontiers_.end())
                    first_new_ftr_ = inserted;
            }
            else
            {
                // Find no viewpoint, move cluster to dormant list
                dormant_frontiers_.push_back(tmp_ftr);
                ++new_dormant_num;
            }
        }
        new_frontiers_num_ = new_num;
        // Reset indices of frontiers
        int idx = 0;
        for (auto &ft : frontiers_)
        {
            ft.id_ = idx++;
        }
    }

    // 获取每个簇中最佳的viewpoints的pos,yaw和簇的平均中心
    void FrontierFinder::getTopViewpointsInfo(
        const Vector3d &cur_pos, vector<Eigen::Vector3d> &points, vector<double> &yaws,
        vector<Eigen::Vector3d> &averages)
    {
        points.clear();
        yaws.clear();
        averages.clear();
        for (auto frontier : frontiers_)
        {
            bool no_view = true;
            for (auto view : frontier.viewpoints_)
            {
                // @todo:us 为什么要足够远
                // Retrieve the first viewpoint that is far enough and has highest coverage
                if ((view.pos_ - cur_pos).norm() < min_candidate_dist_)
                    continue;
                points.push_back(view.pos_);
                yaws.push_back(view.yaw_);
                averages.push_back(frontier.average_);
                no_view = false;
                break;
            }
            if (no_view)
            {
                // All viewpoints are very close, just use the first one (with highest coverage).
                auto view = frontier.viewpoints_.front();
                points.push_back(view.pos_);
                yaws.push_back(view.yaw_);
                averages.push_back(frontier.average_);
            }
        }
    }

    // 根据给定的边界ids，获取每个前沿点的视点位置和偏航角信息
    // max_decay: 视点可见数的最大衰减因子，类型为double，用于筛选视点
    // view_num: 所需的视点数量，类型为int，表示每个前沿点返回的视点数量。
    void FrontierFinder::getViewpointsInfo(
        const Vector3d &cur_pos, const vector<int> &ids, const int &view_num, const double &max_decay,
        vector<vector<Eigen::Vector3d>> &points, vector<vector<double>> &yaws)
    {
        points.clear();
        yaws.clear();
        for (auto id : ids)
        {
            // Scan all frontiers to find one with the same id
            for (auto frontier : frontiers_)
            {
                if (frontier.id_ == id)
                {
                    // Get several top viewpoints that are far enough
                    vector<Eigen::Vector3d> pts;
                    vector<double> ys;
                    int visib_thresh = frontier.viewpoints_.front().visib_num_ * max_decay;
                    for (auto view : frontier.viewpoints_)
                    {
                        if (pts.size() >= view_num || view.visib_num_ <= visib_thresh)
                            break;
                        if ((view.pos_ - cur_pos).norm() < min_candidate_dist_)
                            continue;
                        pts.push_back(view.pos_);
                        ys.push_back(view.yaw_);
                    }
                    if (pts.empty())
                    {
                        // All viewpoints are very close, ignore the distance limit
                        for (auto view : frontier.viewpoints_)
                        {
                            if (pts.size() >= view_num || view.visib_num_ <= visib_thresh)
                                break;
                            pts.push_back(view.pos_);
                            ys.push_back(view.yaw_);
                        }
                    }
                    points.push_back(pts);
                    yaws.push_back(ys);
                }
            }
        }
    }

    void FrontierFinder::getFrontiers(vector<vector<Eigen::Vector3d>> &clusters)
    {
        clusters.clear();
        for (auto frontier : frontiers_)
            clusters.push_back(frontier.cells_);
        // clusters.push_back(frontier.filtered_cells_);
    }

    void FrontierFinder::getDormantFrontiers(vector<vector<Vector3d>> &clusters)
    {
        clusters.clear();
        for (auto ft : dormant_frontiers_)
            clusters.push_back(ft.cells_);
    }

    void FrontierFinder::getFrontierBoxes(vector<pair<Eigen::Vector3d, Eigen::Vector3d>> &boxes)
    {
        boxes.clear();
        for (auto frontier : frontiers_)
        {
            Vector3d center = (frontier.box_max_ + frontier.box_min_) * 0.5;
            Vector3d scale = frontier.box_max_ - frontier.box_min_;
            boxes.push_back(make_pair(center, scale));
        }
    }

    void FrontierFinder::getPathForTour(
        const Vector3d &pos, const vector<int> &frontier_ids, vector<Vector3d> &path)
    {
        // Make an frontier_indexer to access the frontier list easier
        vector<list<Frontier>::iterator> frontier_indexer;
        for (auto it = frontiers_.begin(); it != frontiers_.end(); ++it)
            frontier_indexer.push_back(it);

        // Compute the path from current pos to the first frontier
        vector<Vector3d> segment;
        ViewNode::searchPath(pos, frontier_indexer[frontier_ids[0]]->viewpoints_.front().pos_, segment);
        path.insert(path.end(), segment.begin(), segment.end());

        // Get paths of tour passing all clusters
        for (int i = 0; i < frontier_ids.size() - 1; ++i)
        {
            // Move to path to next cluster
            auto path_iter = frontier_indexer[frontier_ids[i]]->paths_.begin();
            int next_idx = frontier_ids[i + 1];
            for (int j = 0; j < next_idx; ++j)
                ++path_iter;
            path.insert(path.end(), path_iter->begin(), path_iter->end());
        }
    }

    void FrontierFinder::getFullCostMatrix(
        const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d cur_yaw,
        Eigen::MatrixXd &mat)
    {
        if (false)
        {
            // Use symmetric TSP formulation
            int dim = frontiers_.size() + 2;
            mat.resize(dim, dim); // current pose (0), sites, and virtual depot finally

            int i = 1, j = 1;
            for (auto ftr : frontiers_)
            {
                for (auto cs : ftr.costs_)
                    mat(i, j++) = cs;
                ++i;
                j = 1;
            }

            // Costs from current pose to sites
            for (auto ftr : frontiers_)
            {
                Viewpoint vj = ftr.viewpoints_.front();
                vector<Vector3d> path;
                mat(0, j) = mat(j, 0) =
                    ViewNode::computeCost(cur_pos, vj.pos_, cur_yaw[0], vj.yaw_, cur_vel, cur_yaw[1], path);
                ++j;
            }
            // Costs from depot to sites, the same large vaule
            for (j = 1; j < dim - 1; ++j)
            {
                mat(dim - 1, j) = mat(j, dim - 1) = 100;
            }
            // Zero cost to depot to ensure connection
            mat(0, dim - 1) = mat(dim - 1, 0) = -10000;
        }
        else
        {
            // Use Asymmetric TSP
            int dimen = frontiers_.size();
            mat.resize(dimen + 1, dimen + 1);
            // std::cout << "mat size: " << mat.rows() << ", " << mat.cols() << std::endl;
            // Fill block for clusters
            int i = 1, j = 1;
            for (auto ftr : frontiers_)
            {
                for (auto cs : ftr.costs_)
                {
                    // std::cout << "(" << i << ", " << j << ")"
                    // << ", ";
                    mat(i, j++) = cs;
                }
                ++i;
                j = 1;
            }
            // std::cout << "" << std::endl;

            // Fill block from current state to clusters
            mat.leftCols<1>().setZero();
            for (auto ftr : frontiers_)
            {
                // std::cout << "(0, " << j << ")"
                // << ", ";
                Viewpoint vj = ftr.viewpoints_.front();
                vector<Vector3d> path;
                mat(0, j++) =
                    ViewNode::computeCost(cur_pos, vj.pos_, cur_yaw[0], vj.yaw_, cur_vel, cur_yaw[1], path);
            }
            // std::cout << "" << std::endl;
        }
    }

    void FrontierFinder::findViewpoints(
        const Vector3d &sample, const Vector3d &ftr_avg, vector<Viewpoint> &vps)
    {
        if (!edt_env_->sdf_map_->isInBox(sample) ||
            edt_env_->sdf_map_->getInflateOccupancy(sample) == 1 || isNearUnknown(sample))
            return;

        double left_angle_, right_angle_, vertical_angle_, ray_length_;

        // Central yaw is determined by frontier's average position and sample
        auto dir = ftr_avg - sample;
        double hc = atan2(dir[1], dir[0]);

        vector<int> slice_gains;
        // Evaluate info gain of different slices
        for (double phi_h = -M_PI_2; phi_h <= M_PI_2 + 1e-3; phi_h += M_PI / 18)
        {
            // Compute gain of one slice
            int gain = 0;
            for (double phi_v = -vertical_angle_; phi_v <= vertical_angle_; phi_v += vertical_angle_ / 3)
            {
                // Find endpoint of a ray
                Vector3d end;
                end[0] = sample[0] + ray_length_ * cos(phi_v) * cos(hc + phi_h);
                end[1] = sample[1] + ray_length_ * cos(phi_v) * sin(hc + phi_h);
                end[2] = sample[2] + ray_length_ * sin(phi_v);

                // Do raycasting to check info gain
                Vector3i idx;
                raycaster_->input(sample, end);
                while (raycaster_->nextId(idx))
                {
                    // Hit obstacle, stop the ray
                    if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 || !edt_env_->sdf_map_->isInBox(idx))
                        break;
                    // Count number of unknown cells
                    if (edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
                        ++gain;
                }
            }
            slice_gains.push_back(gain);
        }

        // Sum up slices' gain to get different yaw's gain
        vector<pair<double, int>> yaw_gains;
        for (int i = 0; i < 6; ++i) // [-90,-10]-> [10,90], delta_yaw = 20, 6 groups
        {
            double yaw = hc - M_PI_2 + M_PI / 9.0 * i + right_angle_;
            int gain = 0;
            for (int j = 2 * i; j < 2 * i + 9; ++j) // 80 degree hFOV, 9 slices
                gain += slice_gains[j];
            yaw_gains.push_back(make_pair(yaw, gain));
        }

        // Get several yaws with highest gain
        vps.clear();
        sort(
            yaw_gains.begin(), yaw_gains.end(),
            [](const pair<double, int> &p1, const pair<double, int> &p2)
            {
                return p1.second > p2.second;
            });
        for (int i = 0; i < 3; ++i)
        {
            if (yaw_gains[i].second < min_visib_num_)
                break;
            Viewpoint vp = {sample, yaw_gains[i].first, yaw_gains[i].second};
            while (vp.yaw_ < -M_PI)
                vp.yaw_ += 2 * M_PI;
            while (vp.yaw_ > M_PI)
                vp.yaw_ -= 2 * M_PI;
            vps.push_back(vp);
        }
    }

    // Sample viewpoints around frontier's average position, check coverage to the frontier cells
    void FrontierFinder::sampleViewpoints(Frontier &frontier)
    {
        // Evaluate sample viewpoints on circles, find ones that cover most cells
        // const double theta_thres = 40 / 180 * M_PI;
        const double candidate_dtheta = 30.0 / 180.0 * M_PI;
        // for (double theta = -candidate_dtheta; theta <= candidate_dtheta; theta += candidate_dtheta) {
        for (double theta = 0.0; theta <= 1e-3; theta += candidate_dtheta) {
            for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi_)
            {
                Vector3d disp = {cos(theta) * cos(phi), cos(theta) * sin(phi), sin(theta)};
                if (acos(disp.dot(frontier.normal_)) < -0.5)
                    continue;
                for (double rc = candidate_rmin_, dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_; rc <= candidate_rmax_ + 1e-3; rc += dr)
                {
                    const Vector3d sample_pos = frontier.average_ + rc * disp;
                    if (sample_pos[2] < 0.4)
                        continue;

                    // Qualified viewpoint is in bounding box and in safe region
                    // @todo:us isNearUnknown() 为什么不考虑 NearOccupancy
                    if (!edt_env_->sdf_map_->isInBox(sample_pos) ||
                        edt_env_->sdf_map_->getInflateOccupancy(sample_pos) == 1 || isNearUnknown(sample_pos))
                        continue;

                    // Compute average yaw
                    auto &cells = frontier.filtered_cells_;
                    Eigen::Vector3d ref_dir = (cells.front() - sample_pos).normalized();
                    double avg_yaw = 0.0;
                    for (int i = 1; i < cells.size(); ++i)
                    {
                        Eigen::Vector3d dir = (cells[i] - sample_pos).normalized();
                        double yaw = acos(dir.dot(ref_dir));
                        if (ref_dir.cross(dir)[2] < 0)
                            yaw = -yaw;
                        avg_yaw += yaw;
                    }
                    avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);
                    wrapYaw(avg_yaw);
                    // Compute the fraction of covered and visible cells
                    int visib_num = countVisibleCells(sample_pos, avg_yaw, cells);
                    int visib_min = min(int(cells.size() / 4), min_visib_num_);
                    if (visib_num > visib_min)
                    {
                        Viewpoint vp = {sample_pos, avg_yaw, visib_num};
                        frontier.viewpoints_.push_back(vp);
                        // int gain = findMaxGainYaw(sample_pos, frontier, sample_yaw);
                    }
                }
            }
        }
    }

    bool FrontierFinder::isFrontierCovered()
    {
        Vector3d update_min, update_max;
        edt_env_->sdf_map_->getUpdatedBox(update_min, update_max);

        auto checkChanges = [&](const list<Frontier> &frontiers)
        {
            for (auto ftr : frontiers)
            {
                if (!haveOverlap(ftr.box_min_, ftr.box_max_, update_min, update_max))
                    continue;
                const int change_thresh = min_view_finish_fraction_ * ftr.cells_.size();
                int change_num = 0;
                for (auto cell : ftr.cells_)
                {
                    Eigen::Vector3i idx;
                    edt_env_->sdf_map_->posToIndex(cell, idx);
                    if (!(knownfree(idx) && isNeighborUnknown(idx)) && ++change_num >= change_thresh)
                        return true;
                }
            }
            return false;
        };

        if (checkChanges(frontiers_) || checkChanges(dormant_frontiers_))
            return true;

        return false;
    }

    bool FrontierFinder::isNearUnknown(const Eigen::Vector3d &pos)
    {
        const int vox_num = floor(min_candidate_clearance_ / resolution_);
        for (int x = -vox_num; x <= vox_num; ++x)
            for (int y = -vox_num; y <= vox_num; ++y)
                for (int z = -1; z <= 1; ++z)
                {
                    Eigen::Vector3d vox;
                    vox << pos[0] + x * resolution_, pos[1] + y * resolution_, pos[2] + z * resolution_;
                    if (edt_env_->sdf_map_->getOccupancy(vox) == SDFMap::UNKNOWN)
                        return true;
                }
        return false;
    }

    int FrontierFinder::countVisibleCells(
        const Eigen::Vector3d &pos, const double &yaw, const vector<Eigen::Vector3d> &cluster)
    {
        percep_utils_->setPose(pos, yaw);
        int visib_num = 0;
        Eigen::Vector3i idx;
        for (auto cell : cluster)
        {
            // Check if frontier cell is inside FOV
            if (!percep_utils_->insideFOV(cell))
                continue;

            // Check if frontier cell is visible (not occulded by obstacles)
            raycaster_->input(cell, pos);
            bool visib = true;
            while (raycaster_->nextId(idx))
            {
                if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 ||
                    edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
                {
                    visib = false;
                    break;
                }
            }
            if (visib)
                visib_num += 1;
        }
        return visib_num;
    }

    void FrontierFinder::downsample(
        const vector<Eigen::Vector3d> &cluster_in, vector<Eigen::Vector3d> &cluster_out)
    {
        // downsamping cluster
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudf(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto cell : cluster_in)
            cloud->points.emplace_back(cell[0], cell[1], cell[2]);

        const double leaf_size = edt_env_->sdf_map_->getResolution() * down_sample_;
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(leaf_size, leaf_size, leaf_size);
        sor.filter(*cloudf);

        cluster_out.clear();
        for (auto pt : cloudf->points)
            cluster_out.emplace_back(pt.x, pt.y, pt.z);
    }

    void FrontierFinder::wrapYaw(double &yaw)
    {
        while (yaw < -M_PI)
            yaw += 2 * M_PI;
        while (yaw > M_PI)
            yaw -= 2 * M_PI;
    }

    Eigen::Vector3i FrontierFinder::searchClearVoxel(const Eigen::Vector3i &pt)
    {
        queue<Eigen::Vector3i> init_que;
        vector<Eigen::Vector3i> nbrs;
        Eigen::Vector3i cur, start_idx;
        init_que.push(pt);
        // visited_flag_[toadr(pt)] = 1;

        while (!init_que.empty())
        {
            cur = init_que.front();
            init_que.pop();
            if (knownfree(cur))
            {
                start_idx = cur;
                break;
            }

            nbrs = sixNeighbors(cur);
            for (auto nbr : nbrs)
            {
                int adr = toadr(nbr);
                // if (visited_flag_[adr] == 0)
                // {
                //   init_que.push(nbr);
                //   visited_flag_[adr] = 1;
                // }
            }
        }
        return start_idx;
    }

    inline vector<Eigen::Vector3i> FrontierFinder::sixNeighbors(const Eigen::Vector3i &voxel)
    {
        vector<Eigen::Vector3i> neighbors(6);
        Eigen::Vector3i tmp;

        tmp = voxel - Eigen::Vector3i(1, 0, 0);
        neighbors[0] = tmp;
        tmp = voxel + Eigen::Vector3i(1, 0, 0);
        neighbors[1] = tmp;
        tmp = voxel - Eigen::Vector3i(0, 1, 0);
        neighbors[2] = tmp;
        tmp = voxel + Eigen::Vector3i(0, 1, 0);
        neighbors[3] = tmp;
        tmp = voxel - Eigen::Vector3i(0, 0, 1);
        neighbors[4] = tmp;
        tmp = voxel + Eigen::Vector3i(0, 0, 1);
        neighbors[5] = tmp;

        return neighbors;
    }

    inline vector<Eigen::Vector3i> FrontierFinder::tenNeighbors(const Eigen::Vector3i &voxel)
    {
        vector<Eigen::Vector3i> neighbors(10);
        Eigen::Vector3i tmp;
        int count = 0;

        for (int x = -1; x <= 1; ++x)
        {
            for (int y = -1; y <= 1; ++y)
            {
                if (x == 0 && y == 0)
                    continue;
                tmp = voxel + Eigen::Vector3i(x, y, 0);
                neighbors[count++] = tmp;
            }
        }
        neighbors[count++] = tmp - Eigen::Vector3i(0, 0, 1);
        neighbors[count++] = tmp + Eigen::Vector3i(0, 0, 1);
        return neighbors;
    }

    inline vector<Eigen::Vector3i> FrontierFinder::allNeighbors(const Eigen::Vector3i &voxel)
    {
        vector<Eigen::Vector3i> neighbors(26);
        Eigen::Vector3i tmp;
        int count = 0;
        for (int x = -1; x <= 1; ++x)
            for (int y = -1; y <= 1; ++y)
                for (int z = -1; z <= 1; ++z)
                {
                    if (x == 0 && y == 0 && z == 0)
                        continue;
                    tmp = voxel + Eigen::Vector3i(x, y, z);
                    neighbors[count++] = tmp;
                }
        return neighbors;
    }

    inline bool FrontierFinder::isNeighborUnknown(const Eigen::Vector3i &voxel)
    {
        // At least one neighbor is unknown
        auto nbrs = sixNeighbors(voxel);
        for (auto nbr : nbrs)
        {
            if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN)
                return true;
        }
        return false;
    }

    inline bool FrontierFinder::isNeighborUnknown(const Eigen::Vector3i &voxel, Eigen::Vector3d &normal)
    {
        normal = Eigen::Vector3d::Zero();
        Eigen::Vector3i nbr;

        nbr = voxel - Eigen::Vector3i(0, 0, 1);
        if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN)
            normal += Eigen::Vector3d(0, 0, 1);

        nbr = voxel + Eigen::Vector3i(0, 0, 1);
        if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN)
            normal -= Eigen::Vector3d(0, 0, 1);

        nbr = voxel - Eigen::Vector3i(1, 0, 0);
        if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN)
            normal += Eigen::Vector3d(1, 0, 0);

        nbr = voxel + Eigen::Vector3i(1, 0, 0);
        if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN)
            normal -= Eigen::Vector3d(1, 0, 0);

        nbr = voxel - Eigen::Vector3i(0, 1, 0);
        if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN)
            normal += Eigen::Vector3d(0, 1, 0);

        nbr = voxel + Eigen::Vector3i(0, 1, 0);
        if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN)
            normal -= Eigen::Vector3d(0, 1, 0);

        normal.normalize();

        return !normal.isZero();
    }

    inline int FrontierFinder::toadr(const Eigen::Vector3i &idx)
    {
        return edt_env_->sdf_map_->toAddress(idx);
    }

    inline bool FrontierFinder::knownfree(const Eigen::Vector3i &idx)
    {
        return edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::FREE;
    }

    inline bool FrontierFinder::inmap(const Eigen::Vector3i &idx)
    {
        return edt_env_->sdf_map_->isInMap(idx);
    }

} // namespace fast_planner
