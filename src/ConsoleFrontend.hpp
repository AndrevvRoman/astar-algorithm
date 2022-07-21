#pragma once
#include <iostream>
#include <memory>

#include "ArrayMap.hpp"

class ConsoleFrontend
{
public:
    void draw(ArrayMap map) const
    {
        for (size_t i = 0; i < map.MAP_HEIGHT; ++i)
        {
            std::cout << "|";
            for (size_t j = 0; j < map.MAP_WIDTH; ++j)
            {
                auto currentCell = static_cast<ArrayMap::CellType>(map.frontend[i][j]);
                switch (currentCell)
                {
                case ArrayMap::CellType::WALL_POS:
                    std::cout << "X";
                    break;
                case ArrayMap::CellType::PATH_POS:
                    std::cout << "*";
                    break;
                case ArrayMap::CellType::START_POS:
                    std::cout << "s";
                    break;
                case ArrayMap::CellType::GOAL_POS:
                    std::cout << "g";
                    break;
                case ArrayMap::CellType::OPEN_PATH_POS:
                    std::cout << "+";
                    break;
                case ArrayMap::CellType::CLOSE_PATH_POS:
                    std::cout << "-";
                    break;
                default:
                    std::cout << " ";
                    break;
                }
            }
            std::cout << "|" << std::endl;
        }
    }
};
