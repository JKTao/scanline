#include <string>
using namespace std;
bool start_with(const string & str, const string & pattern){
    return str.rfind(pattern, 0) == 0;
}

