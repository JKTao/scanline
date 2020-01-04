#pragma once

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <memory>
#include <tuple>

struct Vertice{
    Eigen::Vector3d point;
    int index;
    Vertice(double x, double y, double z):point(x, y, z){}
};

struct Edge{
    using PtrVertice = std::shared_ptr<Vertice>;
    static cv::Scalar color;
    double x;
    double dx;
    int dy;
    int id;
    PtrVertice v1, v2;
    Edge(PtrVertice v1, PtrVertice v2, int id):v1(v1), v2(v2), x(v1->point[0]), id(id){
        if(v1->point[1] == v2->point[1]){
            dx = 0;
        }else{
            dx = -(v1->point[0] - v2->point[0]) / (v1->point[1] - v2->point[1]);
        }
        dy = v1->point[1] - v2->point[1];
    }
    void plot(cv::Mat & img){
        // cout << "DEBUG" << img.size() << v1->point[0] << " " << v1->point[1] << endl;
        cv::line(img, cv::Point2d(v1->point[0], v1->point[1]), cv::Point2d(v2->point[0], v2->point[1]), color, 1);
    }
};

struct Polygon{
    using PtrVertice = std::shared_ptr<Vertice>;
    using PtrEdge = std::shared_ptr<Edge>;
    static int count;
    PtrVertice v[3];
    PtrEdge edge1, edge2, edge3;
    int id;
    double a, b, c, d;
    cv::Vec3b color;
    // double max_y_, min_y_;
    int dy;
    double max_y, min_y;
    std::tuple<int, int, int> caculate_polygon(){
        std::sort(begin(v), end(v), [](const PtrVertice & v1, const PtrVertice & v2){return v1->point[1] > v2->point[1];} );
        max_y = v[0]->point[1];
        min_y = v[2]->point[1];
        dy = max_y - min_y;
        return std::make_tuple(int(max_y + 0.5), int(min_y + 0.5), dy);
    }
    std::tuple<int, int, int, PtrEdge, PtrEdge, PtrEdge> caculate_edge(){
        //TODO: What if the edge is parallel to x axis?
        Eigen::Vector3d e1 = v[0]->point - v[1]->point, e2 = v[1]->point - v[2]->point;
        Eigen::Vector3d normal = e1.cross(e2).normalized();
        // cout << "DEBUG " << e1 << "and " << e2 << endl;
        std::tie(a, b, c, d) = std::make_tuple(normal[0], normal[1], normal[2], -normal.adjoint() * v[0]->point);

        edge1 = std::make_shared<Edge>(v[0], v[2], id);
        edge2 = std::make_shared<Edge>(v[0], v[1], id);
        edge3 = std::make_shared<Edge>(v[1], v[2], id);
        //divide the point to edge3
        edge2->dy--;

        double theta = acos(abs(normal[2]));
        int color_ = int(200 * theta + 55);
        color = cv::Vec3b(color_, color_, color_);
        return std::make_tuple(v[0]->point[1], v[1]->point[1], v[2]->point[1], edge1, edge2, edge3);
    }
    Polygon(PtrVertice v1, PtrVertice v2, PtrVertice v3):v{v1, v2, v3}, id(count++){
    }
};

struct ActiveEdge{
    using PtrEdge = std::shared_ptr<Edge>;
    using PtrPolygon = std::shared_ptr<Polygon>;
    double x_l, dx_l;
    double x_r, dx_r;
    int dy_l, dy_r;

    double z_l, dz_x, dz_y;
    PtrPolygon polygon;
    ActiveEdge(PtrEdge & e1, PtrEdge & e2, PtrPolygon & polygon){
        PtrEdge e_l = e2, e_r = e1;
        if(e1->x + e1->dx < e1->x + e2->dx){
            e_l = e1;
            e_r = e2;
        }
        std::tie(dx_l, dx_r, dy_l, dy_r, this->polygon) = std::make_tuple(e_l->dx, e_r->dx, e_l->dy, e_r->dy, polygon);
        //TODO: recaculate z_l
        x_l = e_l->x;
        x_r = e_r->x;
        dz_x = -polygon->a / polygon->c;
        dz_y = polygon->b / polygon->c;
        // z_l = e_l->v1->point[2] + dz_y * diff_y + dz_x * dx_l * diff_y;
        z_l = e_l->v1->point[2];
    }
}; 