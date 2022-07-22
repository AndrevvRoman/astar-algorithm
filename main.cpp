#include "src/MapSearcher.hpp"

int main(int argc, char *argv[])
{
    srand(time(0));
    bool isShutDown = false;
    while (!isShutDown)
    {
        MapSearcher search;
        isShutDown = search.run();
    }
    return 0;
}
