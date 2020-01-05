#include "model.hpp"
#include "utility.hpp"
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

using namespace std;
using PtrVertice = Vertice*;
using PtrPolygon = Polygon*;
using PtrEdge = Edge*;
using PtrActiveEdge = ActiveEdge*;
int Polygon::count = 0;
cv::Scalar Edge::color = cv::Scalar(0, 255, 0);

Model::Model(){}
Model::Model(const string & configure){
    read_configure(configure);
    parse_object_file(object_file_path.c_str());
}
void Model::normalize_vertices(Eigen::Vector3d w){
    auto [min_x_element, max_x_element] = minmax_element(vertices.begin(), vertices.end(), [](const PtrVertice & v1, PtrVertice &v2){return v1->point[0] < v2->point[0];});
    auto [min_y_element, max_y_element] = minmax_element(vertices.begin(), vertices.end(), [](const PtrVertice & v1, PtrVertice &v2){return v1->point[1] < v2->point[1];});
    auto [min_z_element, max_z_element] = minmax_element(vertices.begin(), vertices.end(), [](const PtrVertice & v1, PtrVertice &v2){return v1->point[2] < v2->point[2];});
    auto [min_x, max_x] = make_tuple((*min_x_element)->point[0], (*max_x_element)->point[0]);
    auto [min_y, max_y] = make_tuple((*min_y_element)->point[1], (*max_y_element)->point[1]);
    auto [min_z, max_z] = make_tuple((*min_z_element)->point[2], (*max_z_element)->point[2]);

    // cout << "min_x, max_x " << min_x << " " << max_x << endl;
    // cout << "min_y, max_y " << min_y << " " << max_y << endl;
    // cout << "min_z, max_z " << min_z << " " << max_z << endl;

    double original_center_x = (max_x + min_x) / 2, original_center_y = (max_y + min_y) / 2, original_center_z = (max_z + min_z)/2;
    double original_scale_x = (max_x - min_x), original_scale_y = (max_y - min_y), original_scale_z = (max_z - min_z);
    double original_scale = sqrt(original_scale_x * original_scale_x + original_scale_y * original_scale_y + original_scale_z * original_scale_z);
    double final_scale = min(WIDTH, HEIGHT);
    double scale = final_scale / original_scale / 1.2;
    // cout << "original_scale " << original_scale << "scale " << scale << " " << endl;

    Eigen::AngleAxis rotation_vector(w.norm(), w.normalized());
    Eigen::Matrix3d R = rotation_vector.toRotationMatrix();
    Eigen::Vector3d s(scale, scale, scale);
    Eigen::Matrix3d S = s.asDiagonal();
    Eigen::Vector3d center(original_center_x, original_center_y, original_center_z);
    Eigen::Vector3d bias(WIDTH/2, HEIGHT/2, -100);
    // cout << "center " << center << endl;
    // cout << R << endl;
    for_each(vertices.begin(), vertices.end(), [R, S, center, bias](PtrVertice & vertice){vertice->point =  R * S * (vertice->point - center) + bias; });
    for_each(polygons.begin(), polygons.end(), [](PtrPolygon & polygon){polygon->caculate_normal();});
}

void Model::quantize_vertices(){
    current_vertices.insert(current_vertices.end(), vertices.begin(), vertices.end());
    for(int i = 0; i < current_vertices.size(); i++){
        auto & point = vertices[i]->point;
        current_vertices[i] = new Vertice(point[0], point[1], point[2]);
        std::tie(vertices[i]->point[0], vertices[i]->point[1]) = make_tuple(std::round(point[0]), std::round(point[1]));
    }
    std::swap(current_vertices, vertices);
}

void Model::transform_vertices(Eigen::Vector3d w){
    Eigen::AngleAxis rotation_vector(w.norm(), w.normalized());
    Eigen::Matrix3d R = rotation_vector.toRotationMatrix();
    Eigen::Vector3d bias(WIDTH/2, HEIGHT/2, -100);
    for_each(vertices.begin(), vertices.end(), [R, bias](PtrVertice & vertice){vertice->point = R * (vertice->point - bias) + bias; });
}

void Model::parse_object_file(const char *object_file_path){
    FILE *object_file = fopen(object_file_path, "r");
    char buffer_a[1000], buffer_b[1000];
    char buffer1[100], buffer2[100], buffer3[100];
    while(fscanf(object_file, "%[^\n]\n", buffer_a) != -1){
        if(starts_with(buffer_a, "vn")){
            continue;
        }
        else if(starts_with(buffer_a, "v")){
            double x, y, z;
            sscanf(buffer_a, "v %lf %lf %lf", &x, &y, &z);
            PtrVertice ptr_vertice = new Vertice(x, y, z);
            vertices.push_back(ptr_vertice);
        }
        else if(starts_with(buffer_a, "f")){
            vector<PtrVertice> vn;
            int v1, v2, v3;
            sscanf(buffer_a, "f %[^\n]\n", buffer_b);
            int i = 0, j = 0;
            while(buffer_b[i] != '\0'){
                while(buffer_b[i] == ' '){
                    i++;
                }
                if(buffer_b[i] == '\0'){
                    break;
                }
                j = i;
                while(buffer_b[j] != ' ' && buffer_b[j] != '\0'){
                    j++;
                }
                if(buffer_b[j] == '\0'){
                    buffer_b[j+1] = '\0';
                }
                buffer_b[j] = '\0';
                sscanf(buffer_b + i, "%d ", &v1);
                vn.push_back(vertices[v1 - 1]);
                i = j + 1;
            }
            PtrPolygon ptr_polygon = new Polygon(std::move(vn));
            polygons.push_back(ptr_polygon);
        }
    }
}

void Model::read_configure(const string & configure_file_path){
    YAML::Node config = YAML::LoadFile(configure_file_path);
    object_file_path = config["object_path"].as<string>();
    WIDTH = config["width"].as<int>();
    HEIGHT = config["height"].as<int>();
    YAML::Node rotation_vector = config["rotation_vector"];
    for(int i = 0; i < rotation_vector.size(); i++){
        this->rotation_vector[i] = rotation_vector[i].as<double>();
    }
}


void Model::build_structure(){
    polygons_table.resize(HEIGHT + 1);
    edges_table.resize(HEIGHT + 1);
    z_buffer = Eigen::MatrixXd::Ones(HEIGHT + 1, WIDTH) * 1e5;
    color_buffer = cv::Mat(HEIGHT + 1, WIDTH, CV_8UC3);

    for(auto & ptr_tr:polygons){
        if(abs(ptr_tr->c) < 1e-3){
            continue;
        }
        int max_y, min_y, dy;
        tie(max_y, min_y, dy) = ptr_tr->caculate_polygon();
        polygons_table[max_y].push_back(ptr_tr);
        ptr_tr->caculate_edge();
        for(auto & edge:ptr_tr->edges){
            if(edge->flag){
                // cout << edge->y << endl;
                edges_table[edge->y].push_back(edge);
            }
        }
    }

    // check if model read successfully: Plot frame.
    cv::Mat img = cv::Mat::zeros(HEIGHT, WIDTH, CV_64FC3);
    for(auto & lst:edges_table){
        for(auto & edge:lst){
            edge->plot(img);
        }
    }
    cv::imwrite("/home/taokun/test.jpg", img);
}

void Model::render_model(){
    build_structure();
    z_buffer_scanline();
}

void Model::z_buffer_scanline(){
    //Construct Active Polygon Table, and plot Active Edge Table
    for(int i = HEIGHT; i > 0; i--){
        auto & edges_list = edges_table[i];
        vector<PtrEdge> left_edges_list;

        //insert polygon from polygons_list into active polygon tables.
        //insert edge from polygon into active edge tables.
        sort(edges_list.begin(), edges_list.end(), [](const PtrEdge & edge1, const PtrEdge & edge2){return edge1->id < edge2->id; });
        int j;
        for(j = 0; j < edges_list.size() - 1; j++){
            if(edges_list.size() <= 1){
                break;
            }
            if(edges_list[j]->id == edges_list[j+1]->id){
                auto active_edge = new ActiveEdge(edges_list[j], edges_list[j+1], polygons[edges_list[j]->id]);
                active_edges_table.push_back(active_edge);
                j++;
            }else{
                left_edges_list.push_back(edges_list[j]);
            }
        }
        if(j < edges_list.size()){
            left_edges_list.push_back(edges_list[j]);
        }
        //plot here
        for(auto & active_edge:active_edges_table){
            double z = active_edge->z_l;
            // cout << "DEBUG: id " << active_edge->id << " " << active_edge->x_l << " " << active_edge->x_r << endl; 
            cv::Vec3b color = active_edge->polygon->color;
            if(active_edge->polygon->id == 2){
                // cout << active_edge->x_l << active_edge->x_r << endl;
            }
            int x_l = std::round(active_edge->x_l), x_r = std::round(active_edge->x_r);
            // if(x_l > x_r){
            //     cout << "BAD POINT" << x_l << " " << x_r << " " << active_edge->dx_l << " " <<  active_edge->dx_r << " " << active_edge->polygon->id << " " << i <<  endl;
            // }
            for(int x = active_edge->x_l; x <= active_edge->x_r; x++){
                if(z <= z_buffer(i, x)){
                    z_buffer(i, x) = z;
                    color_buffer.at<cv::Vec3b>(i, x) = color;
                    //assign color.
                }
                z += active_edge->dz_x;
            }
        }

        for(auto it = active_edges_table.begin(); it != active_edges_table.end();){
            auto &active_edge = *it;
            auto &polygon = active_edge->polygon;

            //firstly decrease dy of active polygon and active edge.
            polygon->dy--;
            active_edge->dy_l--;
            active_edge->dy_r--;
            active_edge->z_l += active_edge->dz_y + active_edge->dx_l * active_edge->dz_x;
            active_edge->x_l += active_edge->dx_l;
            active_edge->x_r += active_edge->dx_r;
            //see if active edge failed.
            if((polygon->dy < 0) || (active_edge->dy_r < 0 && active_edge->dy_l < 0)){
                //remove this active edge and polygon
                it = active_edges_table.erase(it);
                continue;
            }
            if(active_edge->dy_r < 0){
                auto edge_element = find_if(left_edges_list.begin(), left_edges_list.end(), [active_edge](const PtrEdge & A){return A->id == active_edge->polygon->id;});
                PtrEdge edge = *edge_element;
                tie(active_edge->x_r, active_edge->dx_r, active_edge->dy_r) = make_tuple(edge->x + edge->dx, edge->dx, edge->dy - 1);
            }else if(active_edge->dy_l < 0){
                auto edge_element = find_if(left_edges_list.begin(), left_edges_list.end(), [active_edge](const PtrEdge & A){return A->id == active_edge->polygon->id;});
                PtrEdge edge = *edge_element;
                tie(active_edge->x_l, active_edge->dx_l, active_edge->dy_l) = make_tuple(edge->x + edge->dx, edge->dx, edge->dy - 1);
            }
            it++;
        }
    }

    cv::imwrite("/home/taokun/result.jpg", color_buffer);
}