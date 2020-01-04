#include "utility.hpp"
#include <string>
#include <chrono>

TicToc::TicToc() {   
    tic();
}   

void TicToc::tic(){   
    start = std::chrono::system_clock::now();
}   

double TicToc::toc(){   
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;
}   


bool starts_with(char *str, const char *pattern){
    int i = 0;
    while(str[i] == pattern[i] && str[i] != '\0'){
        i++;
    }
    return (pattern[i] == '\0');
}