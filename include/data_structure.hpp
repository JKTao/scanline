#pragma once

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <memory>
#include <tuple>

const double color_base = 180;
const double color_scale = color_base/3.14159;
struct Vertice{
    Eigen::Vector3d point;
    int index;
    Vertice(double x, double y, double z):point(x, y, z){}
};

struct Edge{
    using PtrVertice = Vertice*;
    static cv::Scalar color;
    int x;
    int y;
    double dx;
    int dy;
    int id;
    bool flag = true;
    PtrVertice v1, v2;
    Edge(PtrVertice v1_, PtrVertice v2_, int id):v1(v1_), v2(v2_), id(id){
        if(v1->point[1] < v2->point[1]){
            std::swap(v1, v2);
        }
        if(v1->point[1] == v2->point[1]){
            flag = false;
        }else{
            dx = -(v1->point[0] - v2->point[0]) / (v1->point[1] - v2->point[1]);
        }
        dy = v1->point[1] - v2->point[1];
        y = v1->point[1];
        x = v1->point[0];
    }
    void plot(cv::Mat & img){
        // cout << "DEBUG" << img.size() << v1->point[0] << " " << v1->point[1] << endl;
        cv::line(img, cv::Point2d(v1->point[0], v1->point[1]), cv::Point2d(v2->point[0], v2->point[1]), color, 1);
    }
};

struct Polygon{
    using PtrVertice = Vertice*;
    using PtrEdge = Edge*;
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
    std::tuple<int, int, int> caculate_polygon(){
        auto [min_y_element, max_y_element] = minmax_element(vn.begin(), vn.end(), [](const PtrVertice & v1, PtrVertice &v2){return v1->point[1] < v2->point[1];});
        std::tie(min_y, max_y) = std::make_tuple((*min_y_element)->point[1], (*max_y_element)->point[1]);
        dy = max_y - min_y;
        return std::make_tuple(max_y, min_y, dy);
    }
    void caculate_edge(){
        //TODO: What if the edge is parallel to x axis?
        int i;
        for(i = 0; i < vn.size() - 1; i++){
            auto edge = new Edge(vn[i], vn[i+1], id);
            edges.emplace_back(edge);
        }
        auto edge = new Edge(vn[i], vn[0], id);
        edges.emplace_back(edge);
    }
    void caculate_normal(){
        Eigen::Vector3d e1 = vn[0]->point - vn[1]->point, e2 = vn[1]->point - vn[2]->point;
        Eigen::Vector3d normal = e1.cross(e2).normalized();
        std::tie(a, b, c, d) = std::make_tuple(normal[0], normal[1], normal[2], -normal.adjoint() * vn[0]->point);
        double theta = acos(abs(normal[2]));
        int color_ = int(color_scale * theta + 254 - color_base);
        color = cv::Vec3b(color_, color_, color_);
    }

    Polygon(std::vector<PtrVertice> vn):id(count++){
        this->vn = vn;
    }
};

struct ActiveEdge{
    using PtrEdge = Edge*;
    using PtrPolygon = Polygon*;
    double x_l, dx_l;
    double x_r, dx_r;
    int dy_l, dy_r;
    int y;

    double z_l, dz_x, dz_y;
    PtrPolygon polygon;
    ActiveEdge(PtrEdge e1, PtrEdge e2, PtrPolygon & polygon){
        if(e1->x > e2->x || (e1->x == e2->x && e1->dx > e2->dx)){
            std::swap(e1, e2);
        }
        std::tie(dx_l, dx_r, dy_l, dy_r, this->polygon) = std::make_tuple(e1->dx, e2->dx, e1->dy, e2->dy, polygon);
        //TODO: recaculate z_l
        x_l = e1->x;
        x_r = e2->x;
        dz_x = -polygon->a / polygon->c;
        dz_y = polygon->b / polygon->c;
        // z_l = e_l->v1->point[2] + dz_y * diff_y + dz_x * dx_l * diff_y;
        y = e1->y;
        z_l = -(polygon->a * x_l + polygon->b * y + polygon->d) / polygon->c;
    }
}; 