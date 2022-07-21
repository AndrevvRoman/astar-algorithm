#pragma once
#include "AStarSearch.hpp"
#include "ArrayMap.hpp"

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

    // Here's the heuristic function that estimates the distance from a Node
    // to the Goal.
    float goalDistanceEstimate(MapSearchNode &nodeGoal)
    {
        return abs(x - nodeGoal.x) + abs(y - nodeGoal.y);
    }

    bool isGoal(MapSearchNode &nodeGoal)
    {
        if ((x == nodeGoal.x) &&
            (y == nodeGoal.y))
        {
            return true;
        }

        return false;
    }

    // This generates the successors to the given Node. It uses a helper function called
    // AddSuccessor to give the successors to the AStar class. The A* specific initialisation
    // is done for each node internally, so here you just set the state information that
    // is specific to the application
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

        if ((map.getPoint(x - 1, y) < 9) && !((parent_x == x - 1) && (parent_y == y)))
        {
            NewNode = MapSearchNode(x - 1, y);
            astarsearch->addSuccessor(NewNode);
        }

        if ((map .getPoint(x, y - 1) < 9) && !((parent_x == x) && (parent_y == y - 1)))
        {
            NewNode = MapSearchNode(x, y - 1);
            astarsearch->addSuccessor(NewNode);
        }

        if ((map .getPoint(x + 1, y) < 9) && !((parent_x == x + 1) && (parent_y == y)))
        {
            NewNode = MapSearchNode(x + 1, y);
            astarsearch->addSuccessor(NewNode);
        }

        if ((map .getPoint(x, y + 1) < 9) && !((parent_x == x) && (parent_y == y + 1)))
        {
            NewNode = MapSearchNode(x, y + 1);
            astarsearch->addSuccessor(NewNode);
        }

        return true;
    }

    // given this node, what does it cost to move to successor. In the case
    // of our map the answer is the map terrain value at this node since that is
    // conceptually where we're moving
    float getCost(MapSearchNode &successor)
    {
        auto map = ArrayMap::getInstance();
        return (float)map.getPoint(x, y);
    }

    bool isSameState(MapSearchNode &rhs)
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
