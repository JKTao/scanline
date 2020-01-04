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

    //rotate and scale all the vertices 
    TicToc t_transform;
    model.normalize_vertices();
    cout << "Transform model takes " << t_transform.toc() << "ms" << endl;

    TicToc t_render;
    model.render_model();
    cout << "Render models takes " << t_render.toc() << "ms" << endl;


    return 0;
}