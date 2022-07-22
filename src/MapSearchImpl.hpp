#pragma once
#include <iostream>

#include "AStarSearch.hpp"
#include "ArrayMap.hpp"
#include "PrettyFrontend.hpp"
#include "ConsoleFrontend.hpp"

/**
 * @brief UserState implementation that defines searching path in maze
 */
class MapSearchNode
{
public:
    int x; // the (x,y) positions of the node
    int y;

    MapSearchNode() { x = y = 0; }
    MapSearchNode(int px, int py)
    {
        x = px;
        y = py;
    }

    /**
     * @brief The heuristic function that estimates the distance from a Node
     * to the Goal.
     */
    float goalDistanceEstimate(MapSearchNode &nodeGoal)
    {
        return abs(x - nodeGoal.x) + abs(y - nodeGoal.y);
    }

    bool isGoal(MapSearchNode &nodeGoal) const
    {
        if ((x == nodeGoal.x) &&
            (y == nodeGoal.y))
        {
            return true;
        }

        return false;
    }

    /**
     * @brief Function generates the successors to the given Node. It uses a helper function called
     * AddSuccessor to give the successors to the AStar class.
     */
    bool getSuccessors(AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node)
    {
        int parent_x = -1;
        int parent_y = -1;

        if (parent_node)
        {
            parent_x = parent_node->x;
            parent_y = parent_node->y;
        }

        MapSearchNode NewNode;

        // push each possible move except allowing the search to go backwards

        auto map = ArrayMap::getInstance();

        if ((map.getPoint(x - 1, y) < ArrayMap::CellType::WALL_POS) && !((parent_x == x - 1) && (parent_y == y)))
        {
            NewNode = MapSearchNode(x - 1, y);
            astarsearch->addSuccessor(NewNode);
        }

        if ((map.getPoint(x, y - 1) < ArrayMap::CellType::WALL_POS) && !((parent_x == x) && (parent_y == y - 1)))
        {
            NewNode = MapSearchNode(x, y - 1);
            astarsearch->addSuccessor(NewNode);
        }

        if ((map.getPoint(x + 1, y) < ArrayMap::CellType::WALL_POS) && !((parent_x == x + 1) && (parent_y == y)))
        {
            NewNode = MapSearchNode(x + 1, y);
            astarsearch->addSuccessor(NewNode);
        }

        if ((map.getPoint(x, y + 1) < ArrayMap::CellType::WALL_POS) && !((parent_x == x) && (parent_y == y + 1)))
        {
            NewNode = MapSearchNode(x, y + 1);
            astarsearch->addSuccessor(NewNode);
        }

        return true;
    }

    /**
     * @brief Returns the cost of movement for given node
     */
    float getCost(MapSearchNode &successor) const
    {
        auto map = ArrayMap::getInstance();
        return (float)map.getPoint(x, y);
    }

    bool isSameState(MapSearchNode &rhs) const
    {
        if ((x == rhs.x) &&
            (y == rhs.y))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
};

/**
 * @brief Class that generates two random points on grid and starts A* search
 */
class MapSearcher
{
public:
    void run()
    {
        ArrayMap map = ArrayMap::getInstance();
        ConsoleFrontend consoleFrontend;
        PrettyFrontend prettyFrontend;

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
            std::cout << "Search have founded goal state\n";
            auto visitedNodes = astarsearch.getVisitedNodes();
            for (const auto &node : visitedNodes)
            {
                map.setCell(node->userState.x, node->userState.y, ArrayMap::CellType::OPEN_PATH_POS);
            }

            map.setCell(nodeStart.x, nodeStart.y, ArrayMap::CellType::START_POS);
            map.setCell(nodeGoal.x, nodeGoal.y, ArrayMap::CellType::GOAL_POS);

            auto solution = astarsearch.linearizeSolution();
            for (const auto node : solution)
            {
                if (!node.isSameState(nodeGoal) && !node.isSameState(nodeStart))
                {
                    map.setCell(node.x, node.y, ArrayMap::CellType::PATH_POS);
                }
            }

            std::cout << "Solution steps count " << solution.size() << std::endl;
            // Display the number of loops the search went through
            std::cout << "Search Steps count " << astarsearch.getStepCount() << std::endl;
            std::cout << std::endl;
            consoleFrontend.draw(map);
            prettyFrontend.draw(map);
        }
    }

private:
    MapSearchNode _generateRandomPoint() const
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
