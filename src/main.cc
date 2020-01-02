#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <vector>
#include <tuple>
#include <regex>
#include <memory>
#include <algorithm>
#include <fstream>
#include "utility.hpp"
#include "yaml-cpp/yaml.h"


using namespace std;
struct Vertice;
struct Triangle;
struct Edge;
struct ActiveEdge;
using PtrVertice = shared_ptr<Vertice>;
using PtrTriangle = shared_ptr<Triangle>;
using PtrEdge = shared_ptr<Edge>;
using PtrActiveEdge = shared_ptr<ActiveEdge>;

struct Vertice{
    Eigen::Vector3d point;
    int index;
    Vertice(double x, double y, double z):point(x, y, z){}
};

struct Edge{
    static cv::Scalar color;
    double x;
    double dx;
    int dy;
    int id;
    PtrVertice v1, v2;
    Edge(PtrVertice v1, PtrVertice v2, int id):v1(v1), v2(v2), x(v1->point[0]), id(id){
        if(v1->point[1] == v2->point[1]){
            dx = 10;
        }else{
            dx = -(v1->point[0] - v2->point[0]) / (v1->point[1] - v2->point[1]);
        }
        dy = (int)v1->point[1] - (int)v2->point[1];
    }
    void plot(cv::Mat & img){
        // cout << "DEBUG" << img.size() << v1->point[0] << " " << v1->point[1] << endl;
        cv::line(img, cv::Point2d(v1->point[0], v1->point[1]), cv::Point2d(v2->point[0], v2->point[1]), color, 1);
    }
};

struct Triangle{
    static int count;
    PtrVertice v[3];
    PtrEdge edge1, edge2, edge3;
    int id;
    double a, b, c, d;
    cv::Vec3b color;
    // double max_y_, min_y_;
    int dy;
    int max_y, min_y;
    tuple<int, int, int> caculate_triangle(){
        sort(begin(v), end(v), [](const PtrVertice & v1, const PtrVertice & v2){return v1->point[1] > v2->point[1];} );
        max_y = v[0]->point[1];
        min_y = v[2]->point[1];
        dy = max_y - min_y;
        return make_tuple(max_y, min_y, dy);
    }
    tuple<int, int, int, PtrEdge, PtrEdge, PtrEdge> caculate_edge(){
        //TODO: What if the edge is parallel to x axis?
        Eigen::Vector3d e1 = v[0]->point - v[1]->point, e2 = v[1]->point - v[2]->point;
        Eigen::Vector3d normal = e1.cross(e2).normalized();
        // cout << "DEBUG " << e1 << "and " << e2 << endl;
        tie(a, b, c, d) = make_tuple(normal[0], normal[1], normal[2], -normal.adjoint() * v[0]->point);

        edge1 = make_shared<Edge>(v[0], v[2], id);
        edge2 = make_shared<Edge>(v[0], v[1], id);
        edge3 = make_shared<Edge>(v[1], v[2], id);
        //divide the point to edge3
        edge2->dy--;

        int color_ = abs(255 * normal[2]);
        color = cv::Vec3b(color_, color_, color_);
        return make_tuple(v[0]->point[1], v[1]->point[1], v[2]->point[1], edge1, edge2, edge3);
    }
    Triangle(PtrVertice v1, PtrVertice v2, PtrVertice v3):v{v1, v2, v3}, id(count++){
    }
};

struct ActiveEdge{
    double x_l, dx_l;
    double x_r, dx_r;
    int dy_l, dy_r;

    double z_l, dz_x, dz_y;
    int id;
    ActiveEdge(PtrEdge & e1, PtrEdge & e2, PtrTriangle & polygon){
        PtrEdge e_l = e2, e_r = e1;
        if(e1->x + e1->dx < e1->x + e2->dx){
            e_l = e1;
            e_r = e2;
        }
        tie(x_l, dx_l, x_r, dx_r, dy_l, dy_r, id) = make_tuple(e_l->x, e_l->dx, e_r->x, e_r->dx, e_l->dy, e_r->dy, polygon->id);
        z_l = polygon->v[0]->point[2];
        dz_x = -polygon->a / polygon->c;
        dz_y = polygon->b / polygon->c;
    }
}; 

cv::Scalar Edge::color = cv::Scalar(0, 255, 0);
int Triangle::count = 0;

void read_configure(string & configure_file_path, int & WIDTH, int & HEIGHT, string & object_file_path){
    YAML::Node config = YAML::LoadFile(configure_file_path);
    object_file_path = config["object_path"].as<string>();
    WIDTH = config["width"].as<int>();
    HEIGHT = config["height"].as<int>();
}

void transform_vertices(vector<PtrVertice> & vertices, int HEIGHT, int WIDTH){
    auto [min_x_element, max_x_element] = minmax_element(vertices.begin(), vertices.end(), [](const PtrVertice & v1, PtrVertice &v2){return v1->point[0] < v2->point[0];});
    auto [min_y_element, max_y_element] = minmax_element(vertices.begin(), vertices.end(), [](const PtrVertice & v1, PtrVertice &v2){return v1->point[1] < v2->point[1];});
    auto [min_x, max_x] = make_tuple((*min_x_element)->point[0], (*max_x_element)->point[0]);
    auto [min_y, max_y] = make_tuple((*min_y_element)->point[1], (*max_y_element)->point[1]);

    double original_center_x = (max_x + min_x) / 2, original_center_y = (max_y + min_y) / 2;
    double original_scale_x = (max_x - min_x), original_scale_y = (max_y - min_y);
    double scale_x = WIDTH / 3.0 / original_scale_x, scale_y = HEIGHT / 3.0 / original_scale_y, scale_z = 1;

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d s(scale_x, scale_y, scale_z);
    Eigen::Matrix3d S = s.asDiagonal();
    Eigen::Vector3d center(original_center_x, original_center_y, -100);
    Eigen::Vector3d bias(WIDTH/2, HEIGHT/2, -100);

    for_each(vertices.begin(), vertices.end(), [R, S, center, bias](PtrVertice & vertice){vertice->point = R * (S * (vertice->point - center) + bias); });
    // cout << "min_x , max_x" <<  "min_y, max_y" << min_x << " " << max_x << " " << min_y << " " << max_y << endl;
}

void render_models(vector<PtrVertice> & vertices, vector<PtrTriangle> triangles, int HEIGHT, int WIDTH){
    vector<vector<PtrTriangle>> polygons_table(HEIGHT + 1);
    // vector<vector<PtrEdge>> edges_table(HEIGHT + 1);
    list<PtrActiveEdge> active_edges_table;
    list<PtrTriangle> active_polygons_table;
    Eigen::MatrixXd z_buffer = Eigen::MatrixXd::Ones(HEIGHT + 1, WIDTH) * 1e5;
    cv::Mat color_buffer(HEIGHT + 1, WIDTH, CV_8UC3);

    //Construct Polygon Table and Edge Table
    for(auto & ptr_tr:triangles){
        int max_y, min_y, dy;
        tie(max_y, min_y, dy) = ptr_tr->caculate_triangle();
        polygons_table[max_y].push_back(ptr_tr);
        int y1, y2, y3;
        PtrEdge edge1, edge2, edge3;
        tie(y1, y2, y3, edge1, edge2, edge3) = ptr_tr->caculate_edge();
    }

    //check if model read successfully: Plot frame.
    cv::Mat img = cv::Mat::zeros(HEIGHT, WIDTH, CV_64FC3);
    for(auto & ptr_tr:triangles){
        ptr_tr->edge1->plot(img);
        ptr_tr->edge2->plot(img);
        ptr_tr->edge3->plot(img);
    }
    cv::imwrite("/home/taokun/test.jpg", img);
    
    //Construct Active Polygon Table, and plot Active Edge Table
    for(int i = HEIGHT; i > 0; i--){
        auto & polygons_list = polygons_table[i];
        // auto & edges_list = edges_table[i];

        //insert polygon from polygons_list into active polygon tables.
        //insert edge from polygon into active edge tables.
        active_polygons_table.insert(active_polygons_table.end(), polygons_list.begin(), polygons_list.end());
        for(auto & polygon:polygons_list){
            //make active edge
            auto e1 = polygon->edge1, e2 = polygon->edge2;
            auto active_edge = make_shared<ActiveEdge>(e1, e2, polygon);
            active_edges_table.push_back(active_edge);
        }
        
        //plot here
        for(auto & active_edge:active_edges_table){
            double z = active_edge->z_l;
            // cout << "DEBUG: id " << active_edge->id << " " << active_edge->x_l << " " << active_edge->x_r << endl; 
            cv::Vec3b color = triangles[active_edge->id]->color;
            for(int x = active_edge->x_l; x <= active_edge->x_r; x++){
                if(z <= z_buffer(i, x)){
                    z_buffer(i, x) = z;
                    color_buffer.at<cv::Vec3b>(i, x) = color;
                    //assign color.
                }
                z += active_edge->dz_x;
            }
        }

        // if(i == 424){
        //     ofstream log_intern("/home/taokun/intern.txt");
        //     for(int j = 160; j <= 320; j++){
        //         log_intern << z_buffer(i, j) << " ";
        //     }
        //     log_intern.close();
        // }

        for(auto it = active_edges_table.begin(); it != active_edges_table.end();){
            auto &active_edge = *it;
            auto &triangle = triangles[active_edge->id];
            auto &edge3 = triangle->edge3;

            //firstly decrease dy of active polygon and active edge.
            triangle->dy--;
            active_edge->dy_l--;
            active_edge->dy_r--;
            active_edge->z_l += active_edge->dz_y + active_edge->dx_l * active_edge->dz_x;
            active_edge->x_l += active_edge->dx_l;
            active_edge->x_r += active_edge->dx_r;
            //see if active edge failed.
            if(triangle->dy < 0){
                //remove this active edge and polygon
                it = active_edges_table.erase(it);
                continue;
            }
            else if(active_edge->dy_r < 0){
                tie(active_edge->x_r, active_edge->dx_r, active_edge->dy_r) = make_tuple(edge3->x, edge3->dx, edge3->dy);
            }else if(active_edge->dy_l < 0){
                tie(active_edge->x_l, active_edge->dx_l, active_edge->dy_l) = make_tuple(edge3->x, edge3->dx, edge3->dy);
            }
            it++;
        }
    }

    cv::imwrite("/home/taokun/result.jpg", color_buffer);
    // ofstream log_txt("/home/taokun/test.txt");
    // for(int i = 0; i < HEIGHT; i++){
    //     for(int j = 0; j < WIDTH; j++){
    //         log_txt << (int)color_buffer.at<cv::Vec3b>(i, j)(1) << " ";
    //     }
    //     log_txt << "\n";
    // }
    // log_txt.close();
}

void parse_object_file(const char *object_file_path, vector<PtrVertice> & vertices, vector<PtrTriangle> & triangles){
    FILE *object_file = fopen(object_file_path, "r");
    char buffer[1000];
    char buffer1[100], buffer2[100], buffer3[100];
    while(fscanf(object_file, "%[^\n]\n", buffer) != -1){
        if(starts_with(buffer, "vn")){
            continue;
        }
        else if(starts_with(buffer, "v")){
            double x, y, z;
            sscanf(buffer, "v %lf %lf %lf", &x, &y, &z);
            PtrVertice ptr_vertice = make_shared<Vertice>(x, y, z);
            vertices.push_back(ptr_vertice);
        }
        else if(starts_with(buffer, "f")){
            int v1, v2, v3;
            sscanf(buffer, "f %s %s %s", buffer1, buffer2, buffer3);
            sscanf(buffer1, "%d ", &v1);
            sscanf(buffer2, "%d ", &v2);
            sscanf(buffer3, "%d ", &v3);
            PtrTriangle ptr_triangle = make_shared<Triangle>(vertices[v1 - 1], vertices[v2 - 1], vertices[v3 - 1]);
            triangles.push_back(ptr_triangle);
        }
    }
}

int main(){
    string config_path = "/home/taokun/Work/Homework/scanline/config/config.yaml";
    vector<PtrVertice> vertices;
    vector<PtrTriangle> triangles;
    int WIDTH, HEIGHT;
    string object_file_path;
    read_configure(config_path, WIDTH, HEIGHT, object_file_path);

    TicToc t_parse;
    parse_object_file(object_file_path.c_str(), vertices, triangles);
    cout << "Parse object file takes " << t_parse.toc() << "ms" << endl;

    //rotate and scale all the vertices 
    TicToc t_transform;
    transform_vertices(vertices, HEIGHT, WIDTH);
    cout << "Transform model takes " << t_transform.toc() << "ms" << endl;

  
    // auto [min_x_element, max_x_element] = minmax_element(vertices.begin(), vertices.end(), [](const PtrVertice & v1, PtrVertice &v2){return v1->point[0] < v2->point[0];});
    // auto [min_y_element, max_y_element] = minmax_element(vertices.begin(), vertices.end(), [](const PtrVertice & v1, PtrVertice &v2){return v1->point[1] < v2->point[1];});
    // auto [min_x, max_x] = make_tuple((*min_x_element)->point[0], (*max_x_element)->point[0]);
    // auto [min_y, max_y] = make_tuple((*min_y_element)->point[1], (*max_y_element)->point[1]);
    // cout << "min_x , max_x " <<  "min_y, max_y " << min_x << " " << max_x << " " << min_y << " " << max_y << endl;
    TicToc t_render;
    render_models(vertices, triangles, HEIGHT, WIDTH);
    cout << "Render models takes " << t_render.toc() << "ms" << endl;

    return 0;
}