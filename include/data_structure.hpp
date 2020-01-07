#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <memory>
#include <tuple>

const double color_base = 180;
const double color_scale = color_base/3.14159;
struct Polygon;
struct Vertice{
    Eigen::Vector3d point;
    bool is_not_extrem;
    Vertice(double x, double y, double z):point(x, y, z){}
};

struct Edge{
    using PtrVertice = Vertice*;
    using PtrPolygon = Polygon*;
    static cv::Vec3b color;
    double x;
    int y;
    double z;
    double dx;
    int dy;
    int id;
    double dz_x;
    double dz_y;
    PtrPolygon polygon;
    bool flag = true;
    bool extrem_process = false;
    PtrVertice v1, v2;
    Edge(PtrVertice v1_, PtrVertice v2_, PtrPolygon polygon);
    void plot(cv::Mat & img);
};

struct Polygon{
    using PtrVertice = Vertice*;
    using PtrEdge = Edge*;
    using PtrPolygon = Polygon*;
    static int count;
    std::vector<PtrVertice> vn;
    std::vector<PtrEdge> edges;
    // PtrEdge edge1, edge2, edge3;
    int id;
    double a, b, c, d;
    cv::Vec3b color;
    // double max_y_, min_y_;
    int dy;
    int max_y, min_y;
    std::tuple<int, int, int> caculate_polygon();
    void caculate_edge();
    void caculate_normal();
    double caculate_intersection(PtrPolygon polygon, double x_l, double x_r, double y);
    double caculate_depth(double x, double y);
    Polygon(std::vector<PtrVertice> vn);
};

struct ActiveEdge{
    using PtrEdge = Edge*;
    using PtrPolygon = Polygon*;
    double x_l, x_r;
    double z_l, dz_x;
    PtrPolygon polygon;
    ActiveEdge(PtrEdge e1, PtrEdge e2, PtrPolygon polygon);
}; 

struct ActiveSingleEdge{
    using PtrEdge = Edge*;
    using PtrPolygon = Polygon*;
    double x;
    int y;
    double dx;
    int dy;
    double dz_x;
    double dz_y;
    double z;
    PtrPolygon polygon;
    ActiveSingleEdge();
    ActiveSingleEdge(PtrEdge e1, PtrPolygon & polygon);
};