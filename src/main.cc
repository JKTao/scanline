#include <string>
#include <iostream>
#include "utility.hpp"
#include "model.hpp"

using namespace std;


int main(){
    string config_path = "/home/taokun/Work/Homework/scanline/config/config.yaml";

    TicToc t_parse;
    Model model(config_path);
    cout << "Parse object file takes " << t_parse.toc() << "ms" << endl;
    cout << "There are " << model.polygons.size() << " faces"<< endl;

    //rotate and scale all the vertices 
    TicToc t_transform;
    model.normalize_vertices(model.rotation_vector);
    cout << "Transform model takes " << t_transform.toc() << "ms" << endl;

    TicToc t_quantize;
    model.quantize_vertices();
    cout << "Quantize model takes " << t_quantize.toc() << "ms" << endl;

    TicToc t_build;
    model.build_structure();
    cout << "Build structure takes " << t_build.toc() << "ms" << endl;


    TicToc t_scanline;
    model.z_buffer_scanline();
    cout << "Render models takes " << t_scanline.toc() << "ms" << endl;
    return 0;
}