#include "data_structure.hpp"
#include "model.hpp"
#include <opencv2/opencv.hpp>

using namespace std;
const double color_base = 180;
const double color_scale = color_base/3.14159;
Edge::Edge(int v1_, int v2_, int id, std::vector<Eigen::Vector3d> * ref_vertices):v1(v1_), v2(v2_), id(id), ref_vertices(ref_vertices){
    auto &vec1 = (*ref_vertices)[v1], &vec2 = (*ref_vertices)[v2];
    if(vec1[1] < vec2[1]){
        std::swap(vec1, vec2);
    }
    if(vec1[1] == vec2[1]){
        flag = false;
    }else{
        dx = -(vec1[0] - vec2[0]) / (vec1[1] - vec2[1]);
    }
    y = vec1[1];
    x = vec1[0];
    dy = vec1[1] - vec2[1];
}

void Edge::plot(cv::Mat & img){
    // cout << "DEBUG" << img.size() << v1->point[0] << " " << v1->point[1] << endl;
    cv::line(img, cv::Point2d((*ref_vertices)[v1][0], (*ref_vertices)[v1][1]), cv::Point2d((*ref_vertices)[v2][0], (*ref_vertices)[v2][1]),  color, 1);
}

std::tuple<int, int, int> Polygon::caculate_polygon(){
    auto [min_y_element, max_y_element] = minmax_element(vn.begin(), vn.end(), [&](const int & v1, int &v2){return (*ref_vertices)[v1][1] < (*ref_vertices)[v2][1];});
    std::tie(min_y, max_y) = std::make_tuple((*ref_vertices)[*min_y_element][1], (*ref_vertices)[*max_y_element][1]);
    dy = max_y - min_y;
        return std::make_tuple(max_y, min_y, dy);
}
void Polygon::caculate_edge(){
    //TODO: What if the edge is parallel to x axis?
    int i;
    int len = (*ref_edges).size();
    for(i = 0; i < vn.size() - 1; i++){
        (*ref_edges).emplace_back(vn[i], vn[i+1], id);
        edges.push_back(len + i);
    }
    (*ref_edges).emplace_back(vn[i], vn[0], id);
    edges.push_back(len + i);
}
void Polygon::caculate_normal(){
    Eigen::Vector3d e1 = (*ref_vertices)[vn[0]] - (*ref_vertices)[vn[1]], e2 = (*ref_vertices)[vn[1]] - (*ref_vertices)[vn[2]];
    Eigen::Vector3d normal = e1.cross(e2).normalized();
    std::tie(a, b, c, d) = std::make_tuple(normal[0], normal[1], normal[2], -normal.adjoint() * (*ref_vertices)[vn[0]]);
    double theta = acos(abs(normal[2]));
    int color_ = int(color_scale * theta + 254 - color_base);
    color = cv::Vec3b(color_, color_, color_);
}

Polygon::Polygon(std::vector<int> vn, std::vector<Eigen::Vector3d> * ref_vertices, std::vector<Edge> * ref_edges):id(count++), ref_vertices(ref_vertices),
    ref_edges(ref_edges)
{
    this->vn = vn;
}

ActiveEdge::ActiveEdge(int e1, int e2, int id, std::vector<Edge> * ref_edges, std::vector<Polygon> *ref_polygons){
    if((*ref_edges)[e1].x > (*ref_edges)[e2].x || ((*ref_edges)[e1].x == (*ref_edges)[e2].x && (*ref_edges)[e1].dx > (*ref_edges)[e2].dx)){
        std::swap(e1, e2);
    }
    std::tie(dx_l, dx_r, dy_l, dy_r, this->id) = std::make_tuple((*ref_edges)[e1].dx, (*ref_edges)[e2].dx, (*ref_edges)[e1].dy, (*ref_edges)[e2].dy, id);
    //TODO: recaculate z_l
    x_l = (*ref_edges)[0].x;
    x_r = (*ref_edges)[1].x;
    dz_x = -(*ref_polygons)[id].a / (*ref_polygons)[id].c;
    dz_y = (*ref_polygons)[id].b / (*ref_polygons)[id].c;
    // z_l = e_l->v1->point[2] + dz_y * diff_y + dz_x * dx_l * diff_y;
    y = (*ref_edges)[e1].y;
    z_l = -((*ref_polygons)[id].a * x_l + (*ref_polygons)[id].b * y + (*ref_polygons)[id].d) / (*ref_polygons)[id].c;
}