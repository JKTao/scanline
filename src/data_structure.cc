#include "data_structure.hpp"
#include <vector>
#include <tuple>
#include <opencv2/opencv.hpp>

Edge::Edge(PtrVertice v1_, PtrVertice v2_, PtrPolygon polygon):v1(v1_), v2(v2_), polygon(polygon){
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
    z = -(polygon->a * x + polygon->b * y + polygon->d) / polygon->c;
    dz_x = -polygon->a / polygon->c;
    dz_y = polygon->b / polygon->c;
    id = polygon->id;
}

void Edge::plot(cv::Mat & img){
    // cout << "DEBUG" << img.size() << v1->point[0] << " " << v1->point[1] << endl;
    cv::line(img, cv::Point2d(v1->point[0], v1->point[1]), cv::Point2d(v2->point[0], v2->point[1]), color, 1);
}

ActiveEdge::ActiveEdge(PtrEdge e1, PtrEdge e2, PtrPolygon polygon){
    if(e1->x > e2->x || (e1->x == e2->x && e1->dx > e2->dx)){
        std::swap(e1, e2);
    }
    std::tie(x_l, x_r, z_l, dz_x, this->polygon) = std::make_tuple(e1->x, e2->x, e1->z, e1->dz_x, polygon);

}

ActiveSingleEdge::ActiveSingleEdge(){}

ActiveSingleEdge::ActiveSingleEdge(PtrEdge e1, PtrPolygon & polygon){
    std::tie(dx, x, this->polygon, y) = std::make_tuple(e1->dx, e1->x, polygon, e1->y);
    dy = e1->dy;
    //TODO: recaculate z_l
    dz_x = -polygon->a / polygon->c;
    dz_y = polygon->b / polygon->c;
    z = -(polygon->a * x+ polygon->b * y + polygon->d) / polygon->c;
}

std::tuple<int, int, int> Polygon::caculate_polygon(){
auto [min_y_element, max_y_element] = minmax_element(vn.begin(), vn.end(), [](const PtrVertice & v1, PtrVertice &v2){return v1->point[1] < v2->point[1];});
    std::tie(min_y, max_y) = std::make_tuple((*min_y_element)->point[1], (*max_y_element)->point[1]);
    dy = max_y - min_y;
    return std::make_tuple(max_y, min_y, dy);
}

void Polygon::caculate_edge(){
    //TODO: What if the edge is parallel to x axis?
    //TODO: support for interval scanline.
    edges.clear();
    int number_vertices = vn.size();
    for(int i = 0; i < number_vertices; i++){
        double y1, y2, y3;
        std::tie(y1, y2, y3) = std::make_tuple(vn[(i+number_vertices-1)%number_vertices]->point[1], vn[i]->point[1], vn[(i+1)%number_vertices]->point[1]);
        vn[i]->is_not_extrem = ((y2 - y1) < 0) ^ ((y2 - y3) < 0);
    }
    for(int i = 0; i < number_vertices; i++){
        auto edge = new Edge(vn[i], vn[(i+1) % number_vertices], this);
        if(edge->v2->is_not_extrem){
            edge->dy--;
        }
        edges.emplace_back(edge);
    }
}

void Polygon::caculate_normal(){
    Eigen::Vector3d e1 = vn[0]->point - vn[1]->point, e2 = vn[1]->point - vn[2]->point;
    Eigen::Vector3d normal = e1.cross(e2).normalized();
    std::tie(a, b, c, d) = std::make_tuple(normal[0], normal[1], normal[2], -normal.adjoint() * vn[0]->point);
    double theta = acos(abs(normal[2]));
    int color_ = int(color_scale * theta + 254 - color_base);
    color = cv::Vec3b(color_, color_, color_);
    // std::cout << "POLYGON " << id << " " << a << " " << b << " " << c << " " << d << std::endl;
}

double Polygon::caculate_intersection(PtrPolygon polygon, double x_l, double x_r, double y){
    Eigen::Matrix2d A;
    A << a, c, polygon->a, polygon->c;
    Eigen::Vector2d B(b * y + d, polygon->b * y + polygon->d);
    auto intersection = -A.inverse() * B;
    if(intersection[0] >= x_l && x_r >= intersection[0]){
        return intersection[0];
    }
    return (x_l + x_r)/2;
}

double Polygon::caculate_depth(double x, double y){
    double depth = -(a/c * x + b/c * y + d/c);
    return depth;
}

Polygon::Polygon(std::vector<PtrVertice> vn):id(count++){
    this->vn = vn;
}