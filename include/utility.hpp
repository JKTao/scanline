#pragma once
#include <chrono>

class TicToc
{
public:
    TicToc();
    void tic();
    double toc();
private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

bool starts_with(char *str, const char *pattern);