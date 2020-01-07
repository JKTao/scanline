#include <string>
#include <iostream>
#include <map>
#include <set>
#include "utility.hpp"
#include "model.hpp"
#include "opencv2/opencv.hpp"
#include <Eigen/Core>

using namespace std;


int main(){
    cout << "Please modify config file to choose model file and render method." << endl;
    cout << "Its default path is ../config/config.yaml." << endl;
    string config_path = "/home/taokun/Work/Homework/scanline/config/config.yaml";
    set<int> valid_key = {'A', 'D', 'W', 'S', 'J', 'K', 'Q'};
    map<int, Eigen::Vector3d> map_key_rotation = {
        {'H', {0.05, 0, 0}},
        {'L', {-0.05, 0, 0}},
        {'J', {0, 0.05, 0}},
        {'K', {0, -0.05, 0}},
        {'I', {0, 0, -0.05}},
        {'M', {0, 0, 0.05}}
    };
    TicToc t_parse;
    Model model(config_path);
    cout << "Parse object file takes " << t_parse.toc() << "ms" << endl;
    cout << "There are " << model.polygons.size() << " faces"<< endl;

    //rotate and scale all the vertices 
    TicToc t_transform;
    model.normalize_vertices(model.rotation_vector);
    cout << "Normalize model takes " << t_transform.toc() << "ms" << endl;
    cout << "---------------------------------------------------" << endl;
    // cout << "Please choose render method. [0]interval scanline, [1]z_buffer scanline" << endl;
    // int c = getchar();
    // method = c - '0';
    while(1){
        TicToc t_quantize;
        model.quantize_vertices();
        cout << "Quantize model takes " << t_quantize.toc() << "ms" << endl;

        TicToc t_build;
        model.build_structure();
        cout << "Build structure takes " << t_build.toc() << "ms" << endl;


        TicToc t_render;
        model.render_model();
        cout << "Render models takes " << t_render.toc() << "ms" << endl;
        
        model.show();
        int key = cv::waitKey();
        while(valid_key.find(key) == valid_key.end()){
            key = cv::waitKey();
        }
        if(key == 'Q'){
            break;
        }else{
            model.transform_vertices(map_key_rotation[key]);
        }
    }
    return 0;
}