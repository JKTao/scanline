#pragma once
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "data_structure.hpp"


struct Model{
    using PtrVertice = std::shared_ptr<Vertice>; 
    using PtrPolygon = std::shared_ptr<Polygon>;
    using PtrEdge = std::shared_ptr<Edge>;
    using PtrActiveEdge = std::shared_ptr<ActiveEdge>;

    std::vector<PtrVertice> vertices;
    std::vector<PtrVertice> current_vertices;
    std::vector<PtrPolygon> polygons;
    int WIDTH, HEIGHT;
    Eigen::Vector3d rotation_vector;
    std::string object_file_path;

    std::vector<std::vector<PtrPolygon>> polygons_table;
    std::vector<std::vector<PtrEdge>> edges_table;
    std::list<PtrActiveEdge> active_edges_table;
    std::list<PtrPolygon> active_polygons_table;
    cv::Mat color_buffer;
    Eigen::MatrixXd z_buffer;

    Model();
    Model(const std::string & configure_file_path);
    void parse_object_file(const char *object_file_path);
    void read_configure(const std::string & configure_file_path);
    void normalize_vertices(Eigen::Vector3d w = Eigen::Vector3d(0, 0, 0));

    void quantize_vertices();

    void transform_vertices(Eigen::Vector3d w = Eigen::Vector3d(0, 0, 0));
    void z_buffer_scanline();
    void render_model();
    void build_structure();
};