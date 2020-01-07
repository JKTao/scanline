#include "model.hpp"
#include "utility.hpp"
#include "cstdio"
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
cv::Vec3b Edge::color = cv::Vec3b(0, 255, 0);

Model::Model(){}
Model::Model(const string & configure){
    read_configure(configure);
    parse_object_file(object_file_path.c_str());
    rotation_matrix = Eigen::Matrix3d::Identity(3, 3);
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

    Eigen::AngleAxisd rotation_vector(w.norm(), w.normalized());
    Eigen::Matrix3d R = rotation_vector.toRotationMatrix();
    Eigen::Vector3d s(scale, scale, scale);
    Eigen::Matrix3d S = s.asDiagonal();
    Eigen::Vector3d center(original_center_x, original_center_y, original_center_z);
    Eigen::Vector3d bias(WIDTH/2, HEIGHT/2, -100);
    // cout << "center " << center << endl;
    // cout << R << endl;
    //Backup vertices
    for_each(vertices.begin(), vertices.end(), [R, S, center, bias](PtrVertice & vertice){vertice->point =  R * S * (vertice->point - center) + bias; });
    backup_vertices.insert(backup_vertices.end(), vertices.begin(), vertices.end());
    for(int i = 0; i < backup_vertices.size(); i++){
        auto & point = vertices[i]->point;
        backup_vertices[i] = new Vertice(point[0], point[1], point[2]);
    }
}

void Model::quantize_vertices(){
    for_each(polygons.begin(), polygons.end(), [](PtrPolygon & polygon){polygon->caculate_normal();});
    for(int i = 0; i < vertices.size(); i++){
        auto & point = vertices[i]->point;
        std::tie(vertices[i]->point[0], vertices[i]->point[1]) = make_tuple(int(point[0] + 0.5), int(point[1] + 0.5));
    }
}

void Model::transform_vertices(Eigen::Vector3d w){
    Eigen::AngleAxisd rotation_vector(w.norm(), w.normalized());
    Eigen::Matrix3d R = rotation_vector.toRotationMatrix();
    Eigen::Vector3d bias(WIDTH/2, HEIGHT/2, -100);
    rotation_matrix = R * rotation_matrix;
    cout << "Rotation Matrix:\n " << rotation_matrix << endl;
    for(int i = 0; i < vertices.size(); i++){
        vertices[i]->point = rotation_matrix * (backup_vertices[i]->point - bias) + bias;
    }
}

void Model::parse_object_file(const char *object_file_path){
    FILE *object_file = fopen(object_file_path, "r");
    if(object_file == NULL){
        cerr << "Unable to open object file. " << object_file_path << endl;
        exit(1);
    }
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
    mode = config["render_mode"].as<int>();
    YAML::Node rotation_vector = config["rotation_vector"];
    for(int i = 0; i < rotation_vector.size(); i++){
        this->rotation_vector[i] = rotation_vector[i].as<double>();
    }
}

void Model::build_structure(){
    // polygons_table.clear();
    // edges_table.clear();
    // polygons_table.resize(HEIGHT + 1);
    polygons_table = vector<vector<PtrPolygon>>(HEIGHT + 1, vector<PtrPolygon>{});
    edges_table = vector<vector<PtrEdge>>(HEIGHT + 1, vector<PtrEdge>{});

    z_buffer = Eigen::MatrixXd::Ones(HEIGHT + 1, WIDTH) * 1e5;
    color_buffer = cv::Mat::zeros(HEIGHT + 1, WIDTH, CV_8UC3);

    for(auto & ptr_tr:polygons){
        if(abs(ptr_tr->c) < 2e-3){
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
    // cv::Mat img = cv::Mat::zeros(HEIGHT, WIDTH, CV_64FC3);
    // for(auto & lst:edges_table){
    //     for(auto & edge:lst){
    //         edge->plot(img);
    //     }
    // }
    // cv::imwrite("/home/taokun/test.jpg", img);
}

void Model::render_model(){
    if(mode == 1){
        z_buffer_scanline();
    }else if(mode == 0){
        interval_scanline();
    }else{
        cerr << "Invalid mode number " << mode << endl;
        exit(1);
    }
}

void Model::interval_scanline(){
    cout << "Render with interval scanline method." << endl;
    // for(auto & vertice:vertices){
        // cout << vertice->point << endl;
    // }
    active_single_edges_table.clear();
    for(int i = HEIGHT; i > 0; i--){
        active_polygons_table.clear();
        auto & edges_list = edges_table[i];

        //insert the sorted vector into list
        insert_sorted_vector_into_active_edge_list(active_single_edges_table, edges_list);
        sort(active_single_edges_table.begin(), active_single_edges_table.end(), [](const ActiveSingleEdge& edge1, const ActiveSingleEdge& edge2) {return edge1.x < edge2.x;});
        if(active_single_edges_table.empty()){
            continue;
        }
        // cout << i << endl;
        for(auto it_current = active_single_edges_table.begin(), it_next = next(it_current); it_next != active_single_edges_table.end();it_current = it_next, it_next = next(it_next) ){
            insert_active_polygons_table(active_polygons_table, *it_current);

            if(active_polygons_table.empty()){
                continue;
            }
            double x_l, x_r, z_l, z_r;
            int id1, id2;
            std::tie(x_l, x_r) = std::make_tuple(it_current->x, it_next->x);
            // if(i >= 170 && i <= 180){
            // if(i >= 0){
            //     // cout << "SPECIAL DEBUG: " << i << " " << x_l << " " << x_r << endl;

            //     for(auto it = active_polygons_table.begin(); it != active_polygons_table.end(); it++){
            //         // cout << (*it)->id << ": " << (*it)->caculate_depth(x_l, i) << " " <<  (*it)->caculate_depth(x_r, i) << endl; 
            //         sum += (*it)->caculate_depth(x_l, i) + (*it)->caculate_depth(x_r, i);
            //     }
            // }
            auto min_zl_element = min_element(active_polygons_table.begin(), active_polygons_table.end(), [x_l, i](const PtrPolygon & A, const PtrPolygon & B){return A->caculate_depth(x_l, i) < B->caculate_depth(x_l, i); });
            auto min_zr_element = min_element(active_polygons_table.begin(), active_polygons_table.end(), [x_r, i](const PtrPolygon & A, const PtrPolygon & B){return A->caculate_depth(x_r, i) < B->caculate_depth(x_r, i); });
            std::tie(id1, id2) = std::make_tuple((*min_zl_element)->id, (*min_zr_element)->id);
            if(id1 == id2){
                // No through
                draw_line(x_l, x_r, i, polygons[id1]->color);
                    //cv::line(color_buffer, { (int)std::round(x_l), i }, { (int)std::round(x_r), i }, polygons[id1]->color, 1);
            }else{
                //find the intersection point;
                auto x_m = polygons[id1]->caculate_intersection(polygons[id2], x_l, x_r, i);
                //TODO: use this for time measure.
 //               if(1 == 1){
 //                   cout << "INTERSECTION " << i << " " << x_l << " " << x_m << " " << x_r << endl;
 //               }
                draw_line(x_l, x_m, i, polygons[id1]->color);
                draw_line(x_m, x_r, i, polygons[id2]->color);

            }

        }

        int edge_table_place = active_single_edges_table.size();
        for (int j = 0; j < edge_table_place; j++) {
            auto& edge = active_single_edges_table[j];
            edge.dy--;
            edge.x += edge.dx;
            edge.z += edge.dz_y + edge.dx * edge.dz_x;
            if (edge.dy < 0) {
                std::swap(active_single_edges_table[j], active_single_edges_table[--edge_table_place]);
                j--;
            }
        }
        active_single_edges_table.resize(edge_table_place);
        // cout << "SEPCIAL DEBUG: " << i << " " << sum << endl;
    }
}
void Model::insert_active_polygons_table(list<PtrPolygon> & active_polygons_table, ActiveSingleEdge & edge){
    int id = edge.polygon->id;
    auto it = find_if(active_polygons_table.begin(), active_polygons_table.end(), [id](const PtrPolygon & polygon){return polygon->id == id;});
    if(it == active_polygons_table.end()){
        active_polygons_table.push_back(polygons[id]);
    }else{
        active_polygons_table.erase(it);
    }
}
void Model::draw_line(double x_l_, double x_r_, int y, cv::Vec3b& color) {
    int x_l = int(x_l_ + 0.5), x_r = int(x_r_ + 0.5);
    for(int x = x_l; x <= x_r; x++){
        color_buffer.at<cv::Vec3b>(y, x) = color;
        // cv::line(color_buffer, { x_l, y }, { x_r, y }, color, 1);
    }

}

void Model::insert_sorted_vector_into_active_edge_list(vector<ActiveSingleEdge> & active_single_edges_table, vector<PtrEdge> & edges_list){
    int m = active_single_edges_table.size(), n = edges_list.size();
    active_single_edges_table.reserve(m + n);
    for(int j = 0; j < edges_list.size(); j++){
        active_single_edges_table.emplace_back(ActiveSingleEdge(edges_list[j], edges_list[j]->polygon));
    }
}

void Model::z_buffer_scanline(){
    cout << "Render with z buffer scanline method." << endl;
    active_polygons_table.clear();
    //Construct Active Polygon Table, and plot Active Edge Table
    vector<PtrEdge> edges_list;
    for(int i = HEIGHT; i > 0; i--){
        active_edges_table.clear();
        auto & current_edges_list = edges_table[i];
        edges_list.insert(edges_list.end(), current_edges_list.begin(), current_edges_list.end());
        //insert polygon from polygons_list into active polygon tables.
        //insert edge from polygon into active edge tables.
        sort(edges_list.begin(), edges_list.end(), [](const PtrEdge & edge1, const PtrEdge & edge2){return (edge1->id < edge2->id) || ((edge1->id == edge2->id) && (edge1->x < edge2->x)); });
        int j;
        //for debug
        if(edges_list.empty()){
            continue;
        }

        // cout << "LINE: " << i << " ";
        // cout << "EDGES_LIST: " << edges_list.size() << "            ";
        // for(int k = 0; k < edges_list.size(); k++){
        //     cout << edges_list[k]->id << ":" << edges_list[k]->x << "," << edges_list[k]->z << " ";
        // }
        // cout << endl;

        for(j = 0; j < edges_list.size() - 1; j++){
            if(edges_list.size() <= 1){
                break;
            }
            if(edges_list[j]->id == edges_list[j+1]->id){
                auto active_edge = new ActiveEdge(edges_list[j], edges_list[j+1], polygons[edges_list[j]->id]);
                active_edges_table.push_back(active_edge);
                j++;
            }else{
                // cout << "What happend?" << endl;
                // left_edges_list.push_back(edges_list[j]);
            }
        }

        //plot here
        for(auto & active_edge:active_edges_table){
            double z = active_edge->z_l;
            cv::Vec3b color = active_edge->polygon->color;
            for(int x = active_edge->x_l; x <= active_edge->x_r; x++){
                if(z <= z_buffer(i, x)){
                    z_buffer(i, x) = z;
                    color_buffer.at<cv::Vec3b>(i, x) = color;
                }
                // else{
                //     cout << "Z_BUFFER" << x << " " << i << " " << z << " " << z_buffer(i, x) << endl;
                // }
                z += active_edge->dz_x;
            }
        }
        // cout << endl;



        int edge_list_place = edges_list.size();
        for (int j = 0; j < edge_list_place; j++) {
            auto edge = edges_list[j];
            edge->dy--;
            edge->x += edge->dx;
            edge->z += edge->dz_y + edge->dx * edge->dz_x;
            if(edge->z > 1e3){
                cout << "EDGE:" << edge->polygon->id << " " << edge->z << " " << edge->dz_x << " " << edge->dz_y << endl;
            }
            if (edge->dy < 0) {
                std::swap(edges_list[j], edges_list[--edge_list_place]);
                j--;
            }
        }
        edges_list.resize(edge_list_place);
    }

}

void Model::show(){
    cv::imwrite("/home/taokun/result.jpg", color_buffer);
    cv::imshow("Image", color_buffer);
}