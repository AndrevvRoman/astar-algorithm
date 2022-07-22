#include "src/MapSearchImpl.hpp"

#include <iostream>
#include <stdio.h>
#include <math.h>

using namespace std;

int main(int argc, char *argv[])
{
    srand(time(0));
    MapSearcher search;
    search.run();
    return 0;
}
