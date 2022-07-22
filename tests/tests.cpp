#include <catch2/catch_test_macros.hpp>

#include "../src/MapSearchNode.hpp"
#include "../src/AStarSearch.hpp"

// Current implementation dosent' allow to change map
// If any changes occurs in map then all tests will be invalid

TEST_CASE("Simple search test for A* algorithm with existing solution")
{
    constexpr size_t expectedSolutionSize = 16;
    constexpr size_t expectedVisitedSize = 25;
    std::array<std::array<int, 2>, expectedVisitedSize> expectedVisitedNodes{{
        {3, 8},
        {4, 8},
        {2, 8},
        {5, 8},
        {2, 9},
        {6, 8},
        {2, 10},
        {7, 8},
        {3, 10},
        {4, 10},
        {8, 8},
        {5, 10},
        {9, 8},
        {9, 7},
        {9, 6},
        {8, 6},
        {7, 6},
        {8, 5},
        {7, 5},
        {8, 4},
        {6, 5},
        {6, 6},
        {5, 5},
        {5, 6},
        {5, 4},
    }}; // We do need two braces to avoid error `too many initializers`

    std::array<std::array<int, 2>, expectedSolutionSize> expectedSolutionNodes{{
        {3, 8},
        {4, 8},
        {5, 8},
        {6, 8},
        {7, 8},
        {8, 8},
        {9, 8},
        {9, 7},
        {9, 6},
        {8, 6},
        {7, 6},
        {7, 5},
        {6, 5},
        {5, 5},
        {5, 4},
        {4, 4},
    }}; // We do need two braces to avoid error `too many initializers`

    MapSearchNode nodeStart;
    nodeStart.x = 3;
    nodeStart.y = 8;
    MapSearchNode nodeGoal;
    nodeGoal.x = 4;
    nodeGoal.y = 4;
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
    constexpr size_t expectedSolutionSize = 1;
    constexpr size_t expectedVisitedSize = 236;
    std::array<std::array<int, 2>, expectedVisitedSize> expectedVisitedNodes{{
        {3, 8},
        {4, 8},
        {5, 8},
        {6, 8},
        {2, 8},
        {7, 8},
        {2, 9},
        {8, 8},
        {2, 10},
        {9, 8},
        {3, 10},
        {9, 7},
        {4, 10},
        {9, 6},
        {5, 10},
        {8, 6},
        {7, 6},
        {8, 5},
        {7, 5},
        {8, 4},
        {6, 5},
        {6, 6},
        {10, 6},
        {5, 5},
        {5, 6},
        {10, 5},
        {5, 4},
        {8, 3},
        {10, 8},
        {10, 4},
        {10, 7},
        {11, 5},
        {10, 3},
        {11, 7},
        {4, 4},
        {10, 9},
        {8, 2},
        {12, 5},
        {11, 9},
        {10, 10},
        {12, 4},
        {3, 4},
        {4, 3},
        {10, 2},
        {13, 5},
        {12, 3},
        {4, 2},
        {3, 3},
        {2, 4},
        {11, 10},
        {10, 11},
        {3, 5},
        {13, 4},
        {3, 2},
        {12, 2},
        {14, 4},
        {10, 12},
        {12, 10},
        {2, 5},
        {15, 4},
        {13, 10},
        {10, 13},
        {14, 3},
        {15, 5},
        {10, 14},
        {14, 2},
        {14, 10},
        {15, 6},
        {10, 15},
        {15, 10},
        {16, 5},
        {16, 6},
        {16, 10},
        {17, 5},
        {11, 15},
        {10, 16},
        {9, 16},
        {8, 16},
        {7, 16},
        {8, 15},
        {6, 16},
        {7, 15},
        {8, 14},
        {6, 15},
        {8, 13},
        {8, 12},
        {8, 11},
        {8, 10},
        {7, 10},
        {5, 16},
        {12, 15},
        {9, 17},
        {17, 6},
        {16, 11},
        {10, 17},
        {17, 10},
        {5, 15},
        {18, 5},
        {12, 14},
        {5, 14},
        {18, 4},
        {12, 13},
        {12, 12},
        {13, 15},
        {19, 4},
        {9, 18},
        {18, 3},
        {4, 14},
        {18, 6},
        {4, 13},
        {17, 11},
        {4, 12},
        {13, 14},
        {8, 18},
        {19, 5},
        {7, 18},
        {11, 17},
        {6, 18},
        {18, 10},
        {10, 18},
        {8, 19},
        {7, 19},
        {19, 10},
        {10, 19},
        {5, 18},
        {6, 19},
        {19, 3},
        {19, 6},
        {3, 14},
        {19, 9},
        {18, 11},
        {19, 8},
        {18, 8},
        {14, 14},
        {17, 8},
        {14, 13},
        {16, 8},
        {14, 12},
        {15, 8},
        {19, 7},
        {14, 8},
        {9, 19},
        {13, 8},
        {18, 2},
        {12, 8},
        {5, 19},
        {19, 11},
        {19, 2},
        {2, 14},
        {15, 14},
        {4, 18},
        {11, 19},
        {3, 15},
        {2, 13},
        {18, 12},
        {2, 12},
        {4, 19},
        {19, 1},
        {18, 13},
        {3, 18},
        {19, 12},
        {12, 19},
        {15, 15},
        {2, 15},
        {12, 18},
        {3, 19},
        {18, 14},
        {16, 15},
        {13, 18},
        {19, 0},
        {2, 18},
        {15, 16},
        {18, 0},
        {19, 13},
        {13, 19},
        {17, 0},
        {16, 0},
        {15, 0},
        {14, 0},
        {13, 0},
        {12, 0},
        {11, 0},
        {10, 0},
        {9, 0},
        {8, 0},
        {7, 0},
        {6, 0},
        {16, 16},
        {14, 19},
        {19, 14},
        {5, 0},
        {18, 15},
        {2, 19},
        {15, 17},
        {14, 18},
        {17, 15},
        {17, 16},
        {19, 15},
        {15, 19},
        {1, 19},
        {4, 0},
        {15, 18},
        {18, 16},
        {19, 16},
        {16, 18},
        {3, 0},
        {0, 19},
        {16, 19},
        {0, 18},
        {0, 17},
        {0, 16},
        {1, 17},
        {0, 15},
        {0, 14},
        {0, 13},
        {0, 12},
        {0, 11},
        {0, 10},
        {0, 9},
        {0, 8},
        {0, 7},
        {0, 6},
        {0, 5},
        {0, 4},
        {17, 19},
        {2, 0},
        {0, 3},
        {17, 18},
        {18, 19},
        {0, 2},
        {18, 18},
        {1, 0},
        {19, 19},
        {19, 18},
        {0, 1},
        {0, 0},
    }}; // We do need two braces to avoid error `too many initializers`

    std::array<std::array<int, 2>, expectedSolutionSize> expectedSolutionNodes{{
        {3, 8},
    }}; // We do need two braces to avoid error `too many initializers`

    MapSearchNode nodeStart;
    nodeStart.x = 3;
    nodeStart.y = 8;
    MapSearchNode nodeGoal;
    nodeGoal.x = 6;
    nodeGoal.y = 4;
    AStarSearch<MapSearchNode> astarsearch(nodeStart, nodeGoal);
    SearchState searchResult = astarsearch.preformSearch();

    REQUIRE(searchResult == SearchState::FAILED);

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
