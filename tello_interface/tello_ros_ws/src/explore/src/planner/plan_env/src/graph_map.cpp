#include "plan_env/graph_map.h"

namespace fast_planner
{
    PlanGraph::PlanGraph()
    {
        vertices_.clear();
        edges_.clear();
        distance_.clear();
        points_ids_.clear();
    }

    void PlanGraph::clearGraph()
    {
        vertices_.clear();
        edges_.clear();
        distance_.clear();
        points_ids_.clear();
    }

    // 此方法将一个新的顶点添加到图中，并为其创建空的连接和距离列表
    int PlanGraph::addVertex(const Point3D &point)
    {
        vertices_.push_back(point);
        points_.emplace_back(point.x(), point.y(), point.z());
        std::vector<int> connection;
        edges_.push_back(connection);
        std::vector<double> edges_distance;
        distance_.push_back(edges_distance);
        points_ids_[point] = vertices_.size() - 1;

        return vertices_.size() - 1;
    }

    // 为两个顶点之间添加一条有方向的边
    bool PlanGraph::addOneWayEdge(const int &a_id, const int &b_id, const double &distance)
    {
        if (vertexIndexInRange(a_id) && vertexIndexInRange((b_id)) && a_id != b_id && std::count(edges_[a_id].begin(), edges_[a_id].end(), b_id) == 0)
        {
            edges_[a_id].push_back(b_id);
            distance_[a_id].push_back(distance);
            return true;
        }
        else
        {
            return false;
        }
    }

    // 根据两顶点间的直线距离自动添加两条单向边，形成一条双向边
    bool PlanGraph::addTwoWayEdge(const int &a_id, const int &b_id)
    {
        double distance = vertices_[a_id].distance(vertices_[b_id]);
        if (addOneWayEdge(a_id, b_id, distance) && addOneWayEdge(b_id, a_id, distance))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    // 检查给定的三维点是否已经作为一个顶点存在于图中
    bool PlanGraph::isPoint3DExisted(const Point3D &point) const
    {
        if (points_ids_.count(point) != 0)
            return true;
        else
            return false;
    }

    // 获取指定索引的顶点
    Point3D PlanGraph::getVertex(const int &id) const
    {
        if (vertexIndexInRange(id))
            return vertices_[id];
        else
            return Point3D();
    }

    const std::vector<Point3D> &PlanGraph::getAllVertices() const
    {
        return vertices_;
    }

    const std::vector<std::vector<int>> &PlanGraph::getAllEdges() const
    {
        return edges_;
    }

    bool PlanGraph::isVertexValide(int point_id) const
    {
        return vertexIndexInRange(point_id) && edges_[point_id].size() > 0;
    }
    bool PlanGraph::getPointId(const Point3D &point, int &point_id)
    {
        if (points_ids_.count(point) != 0)
        {
            point_id = points_ids_[point];
            return true;
        }
        else
            return false;
    }

    // 更新内部KD树，以便进行快速最近邻搜索
    void PlanGraph::updateKdtree()
    {
        kd_tree_.setInputCloud(points_.makeShared());
    }

    // 查找与给定点最近的顶点的索引
    int PlanGraph::getNearestVertexId(pcl::KdTreeFLANN<pcl::PointXYZ> &kd_tree, const Point3D &point)
    {
        if (!points_.empty())
        {
            int k = 1;
            pcl::PointXYZ center(point.x(), point.y(), point.z());
            std::vector<int> neighbors_ids;
            std::vector<float> neighbors_distance;
            kd_tree.nearestKSearch(center, k, neighbors_ids, neighbors_distance);
            Point3D nearest_point(points_[neighbors_ids.front()].x, points_[neighbors_ids.front()].y,
                                  points_[neighbors_ids.front()].z);
            int nearest_point_id = -1;
            if (getPointId(nearest_point, nearest_point_id))
            {
                return nearest_point_id;
            }
            else
            {
                return -1;
            }
        }
        else
            return -1;
    }

    // 获取在给定范围内的所有邻居顶点的索引
    std::vector<int>
    PlanGraph::getNeighborVertexsIDs(pcl::KdTreeFLANN<pcl::PointXYZ> &kd_tree, const Point3D &point,
                                     const double &range, std::vector<Point3D> &neighbor_vertexs)
    {
        pcl::PointXYZ center(point.x(), point.y(), point.z());
        std::vector<int> neighbors_ids;
        std::vector<float> neighbors_distance;
        kd_tree.radiusSearch(center, range, neighbors_ids, neighbors_distance);

        std::vector<int> vertexs_ids;
        neighbor_vertexs.clear();
        int id = -1;
        Point3D neigbor_point;
        for (const auto &item : neighbors_ids)
        {
            neigbor_point.x() = points_[item].x;
            neigbor_point.y() = points_[item].y;
            neigbor_point.z() = points_[item].z;
            if (getPointId(neigbor_point, id))
            {
                vertexs_ids.push_back(id);
                neighbor_vertexs.push_back(neigbor_point);
            }
        }
        return vertexs_ids;
    }

    double PlanGraph::computeCost(const int &start_v_id, const int &end_v_id, const double &y1, const double &y2,
                                  const Point3D &v1, Point3DQueue &path)
    {

        const double vm_ = 2.0, w_dir_ = 1.5, yd_ = 60.0 * 3.1415926 / 180.0;
        double pos_cost = 0;

        if (!isVertexValide(start_v_id) || !isVertexValide(end_v_id))
            return 100000;

        if (start_v_id != end_v_id)
        {
            vector<int> waypoint_ids;
            getShortestPath(start_v_id, end_v_id, waypoint_ids, path);

            if (path.size() == 0)
            {
                std::cout << "A_star_search fail: start = " << start_v_id << ", end = " << end_v_id << std::endl;
                std::cout << "start.edges = " << edges_[start_v_id].size() << ", end.edges = " << edges_[end_v_id].size() << std::endl;
                return 10000;
            }

            for (int i = 1; i < path.size(); ++i)
            {
                pos_cost += (path[i] - path[i - 1]).norm();
            }
            pos_cost /= vm_;

            if (v1.norm() > 1e-3)
            {
                Point3D dir = (path.back() - path.front()).normalized();
                Point3D vdir = v1.normalized();
                double diff = acos(vdir.dot(dir));
                pos_cost += w_dir_ * diff;
            }
        }

        // Cost of yaw change
        double diff = fabs(y2 - y1);
        diff = min(diff, 2 * M_PI - diff);
        double yaw_cost = diff / yd_;
        return max(pos_cost, yaw_cost);
    }

    double PlanGraph::computeCost(const int &start_v_id, const int &end_v_id, const double &y1, const double &y2,
                                  const Point3D &v1, Point3DQueue &path, double &time)
    {

        const double vm_ = 2.0, w_dir_ = 1.5, yd_ = 60.0 * 3.1415926 / 180.0;

        time = -1.0;

        if (!isVertexValide(start_v_id) || !isVertexValide(end_v_id))
            return 100000;

        double pos_cost = 0;
        if (start_v_id != end_v_id)
        {
            vector<int> waypoint_ids;
            getShortestPath(start_v_id, end_v_id, waypoint_ids, path);

            if (path.size() == 0)
            {
                std::cout << "A_star_search fail: start = " << start_v_id << ", end = " << end_v_id << std::endl;
                std::cout << "start.edges = " << edges_[start_v_id].size() << ", end.edges = " << edges_[end_v_id].size() << std::endl;
                return 10000;
            }

            for (int i = 1; i < path.size(); ++i)
            {
                pos_cost += (path[i] - path[i - 1]).norm();
            }
            pos_cost /= vm_;
            time = pos_cost;

            if (v1.norm() > 1e-3)
            {
                Point3D dir = (path.back() - path.front()).normalized();
                Point3D vdir = v1.normalized();
                double diff = acos(vdir.dot(dir));
                pos_cost += w_dir_ * diff;
            }
        } else {
            time = 1e-5;
        }

        // Cost of yaw change
        double diff = fabs(y2 - y1);
        diff = min(diff, 2 * M_PI - diff);
        double yaw_cost = diff / yd_;
        time = max(time, yaw_cost);
        return max(pos_cost, yaw_cost);
    }

    double PlanGraph::computeGoCost(const int &des_v_id, const double &des_yaw)
    {
        const double vm_ = 1.0, w_dir_ = 2.0;

        if (!isVertexValide(des_v_id))
            return 0;

        vector<int> waypoint_ids;
        vector<Point3D> path;
        getShortestPath(des_v_id, 0, waypoint_ids, path);

        if (path.size() == 0)
        {
            std::cout << "A_star_search fail: start = " << des_v_id << ", end = " << 0 << std::endl;
            std::cout << "start.edges = " << edges_[des_v_id].size() << ", end.edges = " << edges_[0].size() << std::endl;
            return 0;
        }

        double go_cost = 0;

        for (int i = 1; i < path.size(); ++i)
        {
            go_cost += (path[i] - path[i - 1]).norm();
        }

        go_cost /= vm_;

        // Point3D vdir(cos(des_yaw), sin(des_yaw), 0.0);
        // Point3D dir = (vertices_[0] - vertices_[des_v_id]).normalized();

        // go_cost += w_dir_ * acos(dir.dot(vdir));

        return go_cost;
    }

    // 使用A*搜索算法寻找两个顶点之间的最短路径
    bool PlanGraph::getShortestPath(const int &start_v_id, const int &end_v_id, vector<int> &waypoint_ids,
                                    Point3DQueue &shortest_path) const
    {
        // std::cout << "start get shortest path in graph" << std::endl;
        waypoint_ids.clear();
        shortest_path.clear();
        if (vertexIndexInRange(start_v_id) && vertexIndexInRange(end_v_id))
        {
            if (start_v_id == end_v_id)
            {
                // std::cout << "start point = end point, return the start point" << std::endl;
                waypoint_ids.push_back(start_v_id);
                shortest_path.push_back(vertices_[start_v_id]);
                return false;
            }
            else
            {
                if (A_star_search(start_v_id, end_v_id, waypoint_ids))
                {
                    for (const auto &id : waypoint_ids)
                    {
                        shortest_path.push_back(vertices_[id]);
                    }
                    return true;
                }
                else
                {
                    return false;
                }
            }
        }
        else
        {
            // std::cout << "start or end point is out of range" << std::endl;
            return false;
        }
    }

    bool PlanGraph::A_star_search(const int &start_v_id, const int &end_v_id, vector<int> &waypoint_ids) const
    {
        // std::cout << "start A_star_search in graph" << std::endl;
        typedef std::pair<double, int> iPair;                                   // Vertices are represented by their index in the graph.vertices list
        std::priority_queue<iPair, std::vector<iPair>, std::greater<iPair>> pq; // Priority queue of vertices
        std::vector<double> dist(vertices_.size(), INFINITY);                   // Vector of distances
        std::vector<double> estimation(vertices_.size(), INFINITY);
        const int INF = 0x3f3f3f3f;                           // integer infinity
        std::vector<int> backpointers(vertices_.size(), INF); // Vector of backpointers
        std::vector<bool> in_pq(vertices_.size(), false);

        // std::cout << "searching.." << std::endl;

        // Add the start vertex
        dist[start_v_id] = 0;
        estimation[start_v_id] = vertices_[start_v_id].distance(vertices_[end_v_id]);
        pq.push(std::make_pair(estimation[start_v_id], start_v_id));
        in_pq[start_v_id] = true;

        int u;
        int v;
        while (!pq.empty())
        { // Loop until priority queue is empty
            u = pq.top().second;
            pq.pop(); // Pop the minimum distance vertex
            in_pq[u] = false;
            if (u == end_v_id)
            { // Early termination
                break;
            }
            for (int i = 0; i < edges_[u].size(); ++i)
            { // Get all adjacent vertices
                // Get vertex label and weight of current adjacent edge of u
                v = edges_[u][i];
                // If there is a shorter path to v through u
                if (dist[v] > dist[u] + distance_[u][i])
                {
                    // Updating distance of v
                    dist[v] = dist[u] + distance_[u][i];
                    estimation[v] = dist[v] + vertices_[v].distance(vertices_[end_v_id]);
                    backpointers[v] = u;
                    if (!in_pq[v])
                    {
                        pq.push(std::make_pair(estimation[v], v));
                        in_pq[v] = true;
                    }
                }
            }
        }

        // std::cout << "back pointer to path.." << std::endl;
        // Backtrack to find path
        waypoint_ids.clear();
        int current = end_v_id;
        if (backpointers[current] == INF)
        { // no path found
            // std::cout << "no path found, return empty path" << std::endl;
            return false;
        }
        else
        {
            // path found
            while (current != INF)
            {
                waypoint_ids.push_back(current);
                current = backpointers[current];
            }
            // Reverse the path (constructing it this way since vector is more efficient
            // at push_back than insert[0])
            std::reverse(waypoint_ids.begin(), waypoint_ids.end());
            // std::cout << "reverse a path. " << std::endl;
            // std::cout << "A_star search finish." << std::endl;
            return true;
        }
    }
}