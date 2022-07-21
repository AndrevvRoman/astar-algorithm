#pragma once
#include "ArrayMap.hpp"
#include "MapSearchImpl.hpp"

class RandomPointGenerator
{
public:
    MapSearchNode generatePoint() const
    {
        ArrayMap map = ArrayMap::getInstance();
        MapSearchNode point;
        do
        {
            point.x = rand() % map.MAP_WIDTH;
            point.y = rand() % map.MAP_HEIGHT;
        } while (map.logical[point.y][point.x] == 9);
        return point;
    }
};
