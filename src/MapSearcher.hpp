#pragma once
#include <iostream>

#include "AStarSearch.hpp"
#include "ArrayMap.hpp"
#include "SFML_Frontend.hpp"
#include "ConsoleFrontend.hpp"
#include "MapSearchNode.hpp"

/**
 * @brief Class that generates two random points on grid and starts A* search
 */
class MapSearcher
{
public:
    bool run()
    {
        ArrayMap &map = ArrayMap::getInstance();
        ConsoleFrontend consoleFrontend;
        SFML_Frontend prettyFrontend;

        // Define a start state
        MapSearchNode nodeStart = _generateRandomPoint();
        // Define the goal state
        MapSearchNode nodeGoal = _generateRandomPoint();

        // Set Start and goal states
        AStarSearch<MapSearchNode> astarsearch(nodeStart, nodeGoal);
        SearchState searchResult = astarsearch.preformSearch();
        if (searchResult == SearchState::FAILED)
        {
            std::cout << "Search terminated. Did not find goal state" << std::endl;
        }
        else if (searchResult == SearchState::SUCCEEDED)
        {
            auto visitedNodes = astarsearch.getVisitedNodes();
            std::cout << "Search have founded goal state\n";
            for (const auto &node : visitedNodes)
            {
                map.setCell(node.x, node.y, ArrayMap::CellType::OPEN_PATH_POS);
            }

            map.setCell(nodeStart.x, nodeStart.y, ArrayMap::CellType::START_POS);
            map.setCell(nodeGoal.x, nodeGoal.y, ArrayMap::CellType::GOAL_POS);

            auto solutionNodes = astarsearch.linearizeSolution();
            for (const auto node : solutionNodes)
            {
                if (!node.isSameState(nodeGoal) && !node.isSameState(nodeStart))
                {
                    map.setCell(node.x, node.y, ArrayMap::CellType::PATH_POS);
                }
            }

            std::cout << "Solution steps count " << solutionNodes.size() << std::endl;
            // Display the number of loops the search went through
            std::cout << "Search Steps count " << astarsearch.getStepCount() << std::endl;
            std::cout << std::endl;
            consoleFrontend.draw(map);
            // prettyFrontend.instantDraw(map);
            return prettyFrontend.stepByStepDraw(map, solutionNodes, visitedNodes);
        }
        return false;
    }

    ~MapSearcher()
    {
        // Clean all changes in map after search
        ArrayMap::getInstance().reset();
    }

private:
    MapSearchNode _generateRandomPoint() const
    {
        ArrayMap &map = ArrayMap::getInstance();
        MapSearchNode point;
        do
        {
            point.x = rand() % map.getWidth();
            point.y = rand() % map.getHeight();
        } while (map.getPoint(point.x, point.y) == ArrayMap::CellType::WALL_POS);
        return point;
    }
};
