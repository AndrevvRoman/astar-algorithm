#include <catch2/catch_test_macros.hpp>

#include "../src/MapSearchNode.hpp"
#include "../src/AStarSearch.hpp"

#include <iostream>

// Current implementation dosent' allow to change map
// If any changes occurs in map then all tests will be invalid

TEST_CASE("Simple search test for A* algorithm with existing solution")
{
    std::vector<std::vector<int>> mockMap{
        {1, 1, 1, 1},
        {1, 9, 9, 9},
        {1, 1, 1, 1},
        {1, 1, 1, 1}};

    std::vector<std::vector<int>> expectedVisitedNodes{

        {0, 0},
        {1, 0},
        {0, 1},
        {2, 0},
        {0, 2},
        {3, 0},
        {1, 2},
        {0, 3},
        {2, 2},
        {1, 3},
        {3, 2},
        {2, 3}};

    std::vector<std::vector<int>> expectedSolutionNodes{
        {0, 0},
        {0, 1},
        {0, 2},
        {1, 2},
        {2, 2},
        {3, 2},
        {3, 3}};
    
    size_t expectedSolutionSize = expectedSolutionNodes.size();
    size_t expectedVisitedSize = expectedVisitedNodes.size();

    ArrayMap &map = ArrayMap::getInstance();
    map.setMap(mockMap);

    // From top left corner to bottom right
    MapSearchNode nodeStart;
    nodeStart.x = 0;
    nodeStart.y = 0;
    MapSearchNode nodeGoal;
    nodeGoal.x = 3;
    nodeGoal.y = 3;
    AStarSearch<MapSearchNode> astarsearch(nodeStart, nodeGoal);
    SearchState searchResult = astarsearch.preformSearch();

    REQUIRE(searchResult == SearchState::SUCCEEDED);

    auto visitedNodes = astarsearch.getVisitedNodes();
    auto solutionNodes = astarsearch.linearizeSolution();

    REQUIRE(visitedNodes.size() == expectedVisitedSize);
    REQUIRE(solutionNodes.size() == expectedSolutionSize);

    for (size_t i = 0; i < expectedVisitedSize; ++i)
    {
        CHECK(visitedNodes[i].x == expectedVisitedNodes[i][0]);
        CHECK(visitedNodes[i].y == expectedVisitedNodes[i][1]);
    }

    for (size_t i = 0; i < expectedSolutionSize; ++i)
    {
        CHECK(solutionNodes[i].x == expectedSolutionNodes[i][0]);
        CHECK(solutionNodes[i].y == expectedSolutionNodes[i][1]);
    }
}

TEST_CASE("Simple search test for A* algorithm with non-existing solution")
{

    std::vector<std::vector<int>> mockMap{
        {1, 1, 1, 1},
        {9, 9, 9, 9},
        {1, 1, 1, 1},
        {1, 1, 1, 1}};

    std::vector<std::vector<int>> expectedVisitedNodes{
        {0, 0},
        {1, 0},
        {2, 0},
        {3, 0},
    };

    size_t expectedVisitedSize = expectedVisitedNodes.size();

    ArrayMap &map = ArrayMap::getInstance();
    map.setMap(mockMap);

    // From top left corner to bottom right
    MapSearchNode nodeStart;
    nodeStart.x = 0;
    nodeStart.y = 0;
    MapSearchNode nodeGoal;
    nodeGoal.x = 3;
    nodeGoal.y = 3;
    AStarSearch<MapSearchNode> astarsearch(nodeStart, nodeGoal);
    SearchState searchResult = astarsearch.preformSearch();

    REQUIRE(searchResult == SearchState::FAILED);

    auto visitedNodes = astarsearch.getVisitedNodes();

    REQUIRE(visitedNodes.size() == expectedVisitedSize);

    for (size_t i = 0; i < expectedVisitedSize; ++i)
    {
        CHECK(visitedNodes[i].x == expectedVisitedNodes[i][0]);
        CHECK(visitedNodes[i].y == expectedVisitedNodes[i][1]);
    }
}
