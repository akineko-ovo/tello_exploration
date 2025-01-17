#ifndef _GRAPH_MAP_H_
#define _GRAPH_MAP_H_

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <queue>

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <unordered_map>

#include "plan_env/kdtree.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace fast_planner
{

    class Point3D
    {

    public:
        Point3D() : data_{0.0, 0.0, 0.0} {}

        Point3D(double x, double y, double z) : data_{x, y, z} {}

        Point3D(Point3D const &other) : data_{other.data_[0], other.data_[1], other.data_[2]} {}

        Point3D &operator=(Point3D const &other)
        {
            data_[0] = other.data_[0];
            data_[1] = other.data_[1];
            data_[2] = other.data_[2];
            return *this;
        }

        Point3D cross(Point3D const &other) const { return cross(*this, other); }

        static Point3D cross(Point3D const &first, Point3D const &second)
        {
            return Point3D(
                (first.data_[1] * second.data_[2]) - (first.data_[2] * second.data_[1]),
                (first.data_[2] * second.data_[0]) - (first.data_[0] * second.data_[2]),
                (first.data_[0] * second.data_[1]) - (first.data_[1] * second.data_[0]));
        }

        double dot(Point3D const &other) const { return dot(*this, other); }

        static double dot(Point3D const &first, Point3D const &second)
        {
            return (first.data_[0] * second.data_[0]) + (first.data_[1] * second.data_[1]) +
                   (first.data_[2] * second.data_[2]);
        }

        double &operator()(size_t idx) { return data_[idx]; }

        double const &operator()(size_t idx) const { return data_[idx]; }

        double &operator[](size_t idx) { return data_[idx]; }

        double const &operator[](size_t idx) const { return data_[idx]; }

        double &x() { return data_[0]; }

        double const &x() const { return data_[0]; }

        double &y() { return data_[1]; }

        double const &y() const { return data_[1]; }

        double &z() { return data_[2]; }

        double const &z() const { return data_[2]; }

        double &roll() { return data_[0]; }

        double const &roll() const { return data_[0]; }

        double &pitch() { return data_[1]; }

        double const &pitch() const { return data_[1]; }

        double &yaw() { return data_[2]; }

        double const &yaw() const { return data_[2]; }

        Point3D operator-() const { return Point3D(-data_[0], -data_[1], -data_[2]); }

        Point3D operator-(Point3D const &other) const
        {
            return Point3D(data_[0] - other.data_[0], data_[1] - other.data_[1],
                           data_[2] - other.data_[2]);
        }

        Point3D operator-(double value) const
        {
            return Point3D(data_[0] - value, data_[1] - value, data_[2] - value);
        }

        Point3D operator+(Point3D const &other) const
        {
            return Point3D(data_[0] + other.data_[0], data_[1] + other.data_[1],
                           data_[2] + other.data_[2]);
        }

        Point3D operator+(double value) const
        {
            return Point3D(data_[0] + value, data_[1] + value, data_[2] + value);
        }

        Point3D operator*(Point3D const &other) const
        {
            return Point3D(data_[0] * other.data_[0], data_[1] * other.data_[1],
                           data_[2] * other.data_[2]);
        }

        Point3D operator*(double value) const
        {
            return Point3D(data_[0] * value, data_[1] * value, data_[2] * value);
        }

        Point3D operator/(Point3D const &other) const
        {
            return Point3D(data_[0] / other.data_[0], data_[1] / other.data_[1],
                           data_[2] / other.data_[2]);
        }

        Point3D operator/(double value) const
        {
            return Point3D(data_[0] / value, data_[1] / value, data_[2] / value);
        }

        void operator-=(Point3D const &other)
        {
            data_[0] -= other.data_[0];
            data_[1] -= other.data_[1];
            data_[2] -= other.data_[2];
        }

        void operator+=(Point3D const &other)
        {
            data_[0] += other.data_[0];
            data_[1] += other.data_[1];
            data_[2] += other.data_[2];
        }

        void operator*=(Point3D const &other)
        {
            data_[0] *= other.data_[0];
            data_[1] *= other.data_[1];
            data_[2] *= other.data_[2];
        }

        void operator/=(Point3D const &other)
        {
            data_[0] /= other.data_[0];
            data_[1] /= other.data_[1];
            data_[2] /= other.data_[2];
        }

        void operator-=(double value)
        {
            data_[0] -= value;
            data_[1] -= value;
            data_[2] -= value;
        }

        void operator+=(double value)
        {
            data_[0] += value;
            data_[1] += value;
            data_[2] += value;
        }

        void operator*=(double value)
        {
            data_[0] *= value;
            data_[1] *= value;
            data_[2] *= value;
        }

        void operator/=(double value)
        {
            data_[0] /= value;
            data_[1] /= value;
            data_[2] /= value;
        }

        bool operator==(Point3D const &other) const
        {
            return data_[0] == other.data_[0] && data_[1] == other.data_[1] &&
                   data_[2] == other.data_[2];
        }

        bool operator!=(Point3D const &other) const
        {
            return data_[0] != other.data_[0] || data_[1] != other.data_[1] ||
                   data_[2] != other.data_[2];
        }

        double norm() const { return std::sqrt(squaredNorm()); }

        double squaredNorm() const
        {
            return (data_[0] * data_[0]) + (data_[1] * data_[1]) + (data_[2] * data_[2]);
        }

        Point3D &normalize()
        {
            *this /= norm();
            return *this;
        }

        Point3D normalized() const
        {
            Point3D temp(*this);
            return temp.normalize();
        }

        double angleTo(Point3D const &other) const
        {
            return std::acos(dot(other) / (norm() * other.norm()));
        }

        double distance(Point3D const &other) const
        {
            double x = data_[0] - other.data_[0];
            double y = data_[1] - other.data_[1];
            double z = data_[2] - other.data_[2];
            return sqrt((x * x) + (y * y) + (z * z));
        }

        double distanceXY(Point3D const &other) const
        {
            double x = data_[0] - other.data_[0];
            double y = data_[1] - other.data_[1];
            return sqrt((x * x) + (y * y));
        }

        size_t size() const { return 3; }

        double min() const { return std::min(std::min(data_[0], data_[1]), data_[2]); }

        double max() const { return std::max(std::max(data_[0], data_[1]), data_[2]); }

        size_t minElementIndex() const
        {
            if (data_[0] <= data_[1])
            {
                return data_[0] <= data_[2] ? 0 : 2;
            }
            else
            {
                return data_[1] <= data_[2] ? 1 : 2;
            }
        }

        size_t maxElementIndex() const
        {
            if (data_[0] >= data_[1])
            {
                return data_[0] >= data_[2] ? 0 : 2;
            }
            else
            {
                return data_[1] >= data_[2] ? 1 : 2;
            }
        }

        Point3D &ceil()
        {
            for (int i = 0; i < 3; ++i)
            {
                data_[i] = std::ceil(data_[i]);
            }
            return *this;
        }

        Point3D ceil() const
        {
            return Point3D(std::ceil(data_[0]), std::ceil(data_[1]), std::ceil(data_[2]));
        }

        Point3D &floor()
        {
            for (int i = 0; i < 3; ++i)
            {
                data_[i] = std::floor(data_[i]);
            }
            return *this;
        }

        Point3D floor() const
        {
            return Point3D(std::floor(data_[0]), std::floor(data_[1]), std::floor(data_[2]));
        }

        Point3D &trunc()
        {
            for (int i = 0; i < 3; ++i)
            {
                data_[i] = std::trunc(data_[i]);
            }
            return *this;
        }

        Point3D trunc() const
        {
            return Point3D(std::trunc(data_[0]), std::trunc(data_[1]), std::trunc(data_[2]));
        }

        Point3D &round()
        {
            for (int i = 0; i < 3; ++i)
            {
                data_[i] = std::round(data_[i]);
            }
            return *this;
        }

        Point3D round() const
        {
            return Point3D(std::round(data_[0]), std::round(data_[1]), std::round(data_[2]));
        }

    protected:
        double data_[3];
    };

    struct Point3DHash
    {
        std::size_t operator()(const Point3D &key) const
        {
            int x_int = std::round(key.x() * 1000);
            int y_int = std::round(key.y() * 1000);
            int z_int = std::round(key.z() * 1000);
            std::string x = std::to_string((int)(x_int));
            std::string y = std::to_string((int)(y_int));
            std::string z = std::to_string((int)(z_int));

            using std::hash;
            using std::size_t;

            return ((hash<std::string>()(x) ^ (hash<std::string>()(y) << 1)) >> 1) ^ (hash<std::string>()(z) << 1);
        }
    };

    struct Equal
    {
        bool operator()(const Point3D &lhs, const Point3D &rhs) const
        {
            return std::round(lhs.x() * 1000) == std::round(rhs.x() * 1000) && std::round(lhs.y() * 1000) == std::round(rhs.y() * 1000) && std::round(lhs.z() * 1000) == std::round(rhs.z() * 1000);
        }
    };

    using Point3DSet = std::unordered_set<Point3D, Point3DHash, Equal>;
    template <typename T>
    using Point3DMap = std::unordered_map<Point3D, T, Point3DHash, Equal>;
    using Point3DQueue = std::vector<Point3D>;

    using namespace std;

    class PlanGraph
    {

    public:
        typedef shared_ptr<PlanGraph> Ptr;

        explicit PlanGraph();

        void clearGraph();

        // add and remove elements;
        int addVertex(const Point3D &point);

        bool addOneWayEdge(const int &a_id, const int &b_id, const double &distance);

        bool addTwoWayEdge(const int &a_id, const int &b_id);

        bool isPoint3DExisted(const Point3D &point) const;

        // extraction
        Point3D getVertex(const int &id) const;

        const std::vector<Point3D> &getAllVertices() const;

        const std::vector<std::vector<int>> &getAllEdges() const;

        bool isVertexValide(int point_id) const;

        bool getPointId(const Point3D &point, int &point_id);

        void updateKdtree();

        int getNearestVertexId(pcl::KdTreeFLANN<pcl::PointXYZ> &kd_tree,
                               const Point3D &point);

        std::vector<int> getNeighborVertexsIDs(pcl::KdTreeFLANN<pcl::PointXYZ> &kd_tree, const Point3D &point,
                                               const double &range, std::vector<Point3D> &neighbor_vertexs);

        double computeCost(const int &start_v_id, const int &end_v_id, const double &y1, const double &y2,
                           const Point3D &v1, Point3DQueue &path);

        double computeCost(const int &start_v_id, const int &end_v_id, const double &y1, const double &y2,
                           const Point3D &v1, Point3DQueue &path, double &time);

        double computeGoCost(const int &des_v_id, const double &des_yaw);

        bool getShortestPath(const int &start_v_id, const int &end_v_id, std::vector<int> &waypoint_ids,
                             Point3DQueue &shortest_path) const;

        bool A_star_search(const int &start_v_id, const int &end_v_id, std::vector<int> &waypoint_ids) const;

        pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_;

    private:
        bool vertexIndexInRange(const int &index) const
        {
            return index >= 0 && index < vertices_.size();
        }

        std::vector<Point3D> vertices_;       // vertex  positions
        std::vector<std::vector<int>> edges_; //  edges.
        // Distances between two vertexs corresponding to edges.
        std::vector<std::vector<double>> distance_;
        Point3DMap<int> points_ids_; // store the ids of points.

        pcl::PointCloud<pcl::PointXYZ> points_;
    };
}

#endif