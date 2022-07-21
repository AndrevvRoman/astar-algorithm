#include "src/AStarSearch.hpp"
#include "src/MapSearchImpl.hpp"
#include "src/RandomPointGenerator.hpp"
#include "src/ConsoleFrontend.hpp"
#include "src/PrettyFrontend.hpp"

#include <iostream>
#include <stdio.h>
#include <math.h>

using namespace std;

int main(int argc, char *argv[])
{
    srand(time(0));
    RandomPointGenerator generator;
    ArrayMap map = ArrayMap::getInstance();
    ConsoleFrontend consoleFrontend;
    PrettyFrontend prettyFrontend;

    // Create a start state
    MapSearchNode nodeStart = generator.generatePoint();
    // Define the goal state
    MapSearchNode nodeGoal = generator.generatePoint();

    // Set Start and goal states
    AStarSearch<MapSearchNode> astarsearch(nodeStart, nodeGoal);

    SearchState searchState;
    unsigned int searchSteps = 0;

    do
    {
        searchState = astarsearch.searchStep();
        searchSteps++;

    } while (searchState == SearchState::SEARCHING);

    if (searchState == SearchState::SUCCEEDED)
    {
        cout << "Search have founded goal state\n";
        for (const auto & node : astarsearch.m_visited)
        {
            map.setCell(node.x, node.y, ArrayMap::CellType::OPEN_PATH_POS);
        }

        map.setCell(nodeStart.x, nodeStart.y,ArrayMap::CellType::START_POS);
        map.setCell(nodeGoal.x, nodeGoal.y,ArrayMap::CellType::GOAL_POS);
        MapSearchNode *node = astarsearch.getSolutionStart();

        int steps = 0;
        bool haveSolutions = true;
        while (haveSolutions)
        {
            node = astarsearch.getSolutionNext();
            if (!node)
            {
                haveSolutions = false;
            }
            else
            {
                if (!node->isSameState(nodeGoal))
                {
                    map.setCell(node->x, node->y,ArrayMap::CellType::PATH_POS);
                }
                steps++;
            }
        }

        cout << "Solution steps count" << steps << endl;
    }
    else if (searchState == SearchState::FAILED)
    {
        cout << "Search terminated. Did not find goal state\n";
    }

    // Display the number of loops the search went through
    cout << "Search Steps count " << searchSteps << "\n";

    std::cout << std::endl;
    consoleFrontend.draw(map);
    prettyFrontend.draw(map);
    return 0;
}
