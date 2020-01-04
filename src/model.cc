#include "model.hpp"
#include "utility.hpp"
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

using namespace std;
using PtrVertice = shared_ptr<Vertice>;
using PtrPolygon = shared_ptr<Polygon>;
using PtrEdge = shared_ptr<Edge>;
using PtrActiveEdge = shared_ptr<ActiveEdge>;
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
    cout << "original_scale " << original_scale << "scale " << scale << " " << endl;

    Eigen::AngleAxis rotation_vector(w.norm(), w.normalized());
    Eigen::Matrix3d R = rotation_vector.toRotationMatrix();
    Eigen::Vector3d s(scale, scale, scale);
    Eigen::Matrix3d S = s.asDiagonal();
    Eigen::Vector3d center(original_center_x, original_center_y, original_center_z);
    Eigen::Vector3d bias(WIDTH/2, HEIGHT/2, -100);
    cout << "center " << center << endl;
    for_each(vertices.begin(), vertices.end(), [R, S, center, bias](PtrVertice & vertice){vertice->point =  S * (vertice->point - center) + bias; });


}

void Model::transform_vertices(Eigen::Vector3d w){
    Eigen::AngleAxis rotation_vector(w.norm(), w.normalized());
    Eigen::Matrix3d R = rotation_vector.toRotationMatrix();
    Eigen::Vector3d bias(WIDTH/2, HEIGHT/2, -100);
    for_each(vertices.begin(), vertices.end(), [R, bias](PtrVertice & vertice){vertice->point = R * (vertice->point - bias) + bias; });
}

void Model::parse_object_file(const char *object_file_path){
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
            PtrPolygon ptr_polygon = make_shared<Polygon>(vertices[v1 - 1], vertices[v2 - 1], vertices[v3 - 1]);
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
    z_buffer = Eigen::MatrixXd::Ones(HEIGHT + 1, WIDTH) * 1e5;
    color_buffer = cv::Mat(HEIGHT + 1, WIDTH, CV_8UC3);

    //Construct Polygon Table and Edge Table
    //TODO: remove some bad element;
    for(auto & ptr_tr:polygons){
        int max_y, min_y, dy;
        tie(max_y, min_y, dy) = ptr_tr->caculate_polygon();
        polygons_table[max_y].push_back(ptr_tr);
        int y1, y2, y3;
        PtrEdge edge1, edge2, edge3;
        tie(y1, y2, y3, edge1, edge2, edge3) = ptr_tr->caculate_edge();
    }

    //check if model read successfully: Plot frame.
    cv::Mat img = cv::Mat::zeros(HEIGHT, WIDTH, CV_64FC3);
    for(auto & ptr_tr:polygons){
        ptr_tr->edge1->plot(img);
        ptr_tr->edge2->plot(img);
        ptr_tr->edge3->plot(img);
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
            cv::Vec3b color = active_edge->polygon->color;
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
            auto &polygon = active_edge->polygon;
            auto &edge3 = polygon->edge3;

            //firstly decrease dy of active polygon and active edge.
            polygon->dy--;
            active_edge->dy_l--;
            active_edge->dy_r--;
            active_edge->z_l += active_edge->dz_y + active_edge->dx_l * active_edge->dz_x;
            active_edge->x_l += active_edge->dx_l;
            active_edge->x_r += active_edge->dx_r;
            //see if active edge failed.
            if(polygon->dy < 0){
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