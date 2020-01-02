#pragma once
#include <chrono>

class TicToc
{
  public:
    TicToc()
    {   
        tic();
    }   

    void tic()
    {   
        start = std::chrono::system_clock::now();
    }   

    double toc()
    {   
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }   

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

bool starts_with(char *str, const char *pattern){
    int i = 0;
    while(str[i] == pattern[i] && str[i] != '\0'){
        i++;
    }
    return (pattern[i] == '\0');
}