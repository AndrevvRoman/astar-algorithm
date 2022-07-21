#pragma once
#include <iostream>
#include <stdio.h>
#include <assert.h>

#include <memory>
#include <algorithm>
#include <vector>
#include <deque>
#include <cfloat>

using namespace std;

enum class SearchState
{
    SEARCHING,
    SUCCEEDED,
    FAILED,
    OUT_OF_MEMORY
};

// template <class T>
// class AStarState
// {
// public:
//     virtual ~AStarState() {}
//     virtual float GoalDistanceEstimate(T &nodeGoal) = 0;                         // Heuristic function which computes the estimated cost to the goal node
//     virtual bool isGoal(T &nodeGoal) = 0;                                        // Returns true if this node is the goal node
//     virtual bool getSuccessors(AStarSearch<T> *astarsearch, T *parent_node) = 0; // Retrieves all successors to this node and adds them via astarsearch.addSuccessor()
//     virtual float getCost(T &successor) = 0;                                     // Computes the cost of travelling from this node to the successor node
//     virtual bool isSameState(T &rhs) = 0;                                        // Returns true if this node is the same as the rhs node
// };

// The AStar search class. UserState is the users state space type
template <class UserState>
class AStarSearch
{

public:
    // A node represents a possible state in the search
    class Node
    {
    public:
        shared_ptr<Node> parent; // used during the search to record the parent of successor nodes
        shared_ptr<Node> child;  // used after the search for the application to view the search in reverse

        float g; // cost of this node + it's predecessors
        float h; // heuristic estimate of distance to goal
        float f; // sum of cumulative cost of predecessors and self and heuristic

        Node() : parent(0),
                 child(0),
                 g(0.0f),
                 h(0.0f),
                 f(0.0f)
        {
        }

        UserState userState;
    };

    // For sorting the heap the STL needs compare function that lets us compare
    // the f value of two nodes

    class NodeComparator
    {
    public:
        // x > y
        bool operator()(const shared_ptr<Node> x, const shared_ptr<Node> y) const
        {
            return x->f > y->f;
        }
    };

    AStarSearch(UserState &start, UserState &goal) : m_currentSolutionNode(nullptr),
                                                     m_stepsCount(0)
    {
        m_start = _allocateNode();
        m_goal = _allocateNode();

        assert((m_start != nullptr && m_goal != nullptr));

        m_start->userState = start;
        m_goal->userState = goal;

        m_state = SearchState::SEARCHING;

        // Initialise the AStar specific parts of the Start Node
        // The user only needs fill out the state information

        m_start->g = 0;
        m_start->h = m_start->userState.goalDistanceEstimate(m_goal->userState);
        m_start->f = m_start->g + m_start->h;
        m_start->parent = 0;

        // Push the start node on the Open list

        m_openList.push_back(m_start); // heap now unsorted

        // Sort back element into heap
        std::push_heap(m_openList.begin(), m_openList.end(), NodeComparator());

        // Initialise counter for search steps
        m_stepsCount = 0;
    }

    // Preform search one step
    SearchState searchStep()
    {
        if ((m_state == SearchState::SUCCEEDED) ||
            (m_state == SearchState::FAILED))
        {
            return m_state;
        }

        // Failure is defined as emptying the open list as there is nothing left to
        // search...
        if (m_openList.empty())
        {
            _freeAllNodes();
            m_state = SearchState::FAILED;
            return m_state;
        }

        // Incremement step count
        m_stepsCount++;

        // Pop the best node (the one with the lowest f)
        shared_ptr<Node> current_node = m_openList.front(); // get pointer to the node
        m_visited.push_back(current_node->userState);
        std::pop_heap(m_openList.begin(), m_openList.end(), NodeComparator());
        m_openList.pop_back();

        // Check for the goal, once we pop that we're done
        if (current_node->userState.isGoal(m_goal->userState))
        {
            // The user may want to use the Goal Node he passed in
            // so copy the parent pointer of current_node
            m_goal->parent = current_node->parent;
            m_goal->g = current_node->g;

            // A special case is that the goal was passed in as the start state
            if (current_node->userState.isSameState(m_start->userState) == false)
            {
                _freeNode(current_node);

                // set the child pointers in each node (except goal which has no child)
                shared_ptr<Node> nodeChild = m_goal;
                shared_ptr<Node> nodeParent = m_goal->parent;

                while (nodeChild != m_start)
                {
                    nodeParent->child = nodeChild;
                    nodeChild = nodeParent;
                    nodeParent = nodeParent->parent;
                }
            }

            // delete nodes that aren't needed for the solution
            _freeUnusedNodes();

            m_state = SearchState::SUCCEEDED;

            return m_state;
        }
        else // not goal
        {

            // We now need to generate the successors of this node
            // The user helps us to do this, and we keep the new nodes in
            // m_Successors ...

            m_successors.clear(); // empty vector of successor nodes to current_node

            // User provides this functions and uses AddSuccessor to add each successor of
            // node 'current_node' to m_Successors
            auto parent = current_node->parent ? &current_node->parent->userState : nullptr;
            bool ret = current_node->userState.getSuccessors(this, parent);

            if (!ret)
            {
                iterator_t successor;

                // free the nodes that may previously have been added
                for (successor = m_successors.begin(); successor != m_successors.end(); successor++)
                {
                    _freeNode((*successor));
                }

                m_successors.clear(); // empty vector of successor nodes to current_node

                // free up everything else we allocated
                _freeNode(current_node);
                _freeAllNodes();

                m_state = SearchState::OUT_OF_MEMORY;
                return m_state;
            }

            // Now handle each successor to the current node ...
            for (auto successor = m_successors.begin(); successor != m_successors.end(); successor++)
            {

                // 	The g value for this successor
                float newg = current_node->g + current_node->userState.getCost((*successor)->userState);

                // Now we need to find whether the node is on the open or closed lists
                // If it is but the node that is already on them is better (lower g)
                // then we can forget about this successor

                // First linear search of open list to find node

                iterator_t openListResult;

                for (openListResult = m_openList.begin(); openListResult != m_openList.end(); openListResult++)
                {
                    if ((*openListResult)->userState.isSameState((*successor)->userState))
                    {
                        break;
                    }
                }

                // we found same state on open
                if (openListResult != m_openList.end())
                {
                    if ((*openListResult)->g <= newg)
                    {
                        // instance in the Open is cheaper than the current one
                        _freeNode((*successor));
                        // Continue with next successor
                        continue;
                    }
                }

                iterator_t closedListResult;

                for (closedListResult = m_closedList.begin(); closedListResult != m_closedList.end(); closedListResult++)
                {
                    if ((*closedListResult)->userState.isSameState((*successor)->userState))
                    {
                        break;
                    }
                }

                // we found this state on closed
                if (closedListResult != m_closedList.end())
                {
                    if ((*closedListResult)->g <= newg)
                    {
                        // instance in the Closed is cheaper than the current one
                        _freeNode((*successor));
                        // Continue with next successor
                        continue;
                    }
                }

                // The current node is the best node so far with this particular state

                (*successor)->parent = current_node;
                (*successor)->g = newg;
                (*successor)->h = (*successor)->userState.goalDistanceEstimate(m_goal->userState);
                (*successor)->f = (*successor)->g + (*successor)->h;

                // Successor in closed list
                // 1 - Update old version of this node in closed list
                // 2 - Move it from closed to open list
                // 3 - Sort heap again in open list

                if (closedListResult != m_closedList.end())
                {
                    // Update closed node with successor node AStar data
                    (*closedListResult)->parent = (*successor)->parent;
                    (*closedListResult)->g = (*successor)->g;
                    (*closedListResult)->h = (*successor)->h;
                    (*closedListResult)->f = (*successor)->f;

                    // Free successor node
                    _freeNode((*successor));

                    // Push closed node into open list
                    m_openList.push_back((*closedListResult));

                    // Remove closed node from closed list
                    m_closedList.erase(closedListResult);

                    // Sort back element into heap
                    std::push_heap(m_openList.begin(), m_openList.end(), NodeComparator());
                }

                // Successor in open list
                // 1 - Update old version of this node in open list
                // 2 - sort heap again in open list

                else if (openListResult != m_openList.end())
                {
                    // Update open node with successor node AStar data
                    //*(*openlist_result) = *(*successor);
                    (*openListResult)->parent = (*successor)->parent;
                    (*openListResult)->g = (*successor)->g;
                    (*openListResult)->h = (*successor)->h;
                    (*openListResult)->f = (*successor)->f;

                    // Free successor node
                    _freeNode((*successor));

                    // re-make the heap. `sort_heap` called on an invalid heap does not work
                    std::make_heap(m_openList.begin(), m_openList.end(), NodeComparator());
                }

                // New successor
                // 1 - Move it from successors to open list
                // 2 - sort heap again in open list

                else
                {
                    // Push successor node into open list
                    m_openList.push_back((*successor));

                    // Sort back element into heap
                    std::push_heap(m_openList.begin(), m_openList.end(), NodeComparator());
                }
            }

            // push current_node onto Closed, as we have expanded it now
            m_closedList.push_back(current_node);
        }
        return m_state;
    }

    // User calls this to add a successor to a list of successors
    // when expanding the search frontier
    bool addSuccessor(UserState &State)
    {
        shared_ptr<Node> node = _allocateNode();

        if (node)
        {
            node->userState = State;

            m_successors.push_back(node);

            return true;
        }

        return false;
    }

    // Free the solution nodes
    void freeSolutionNodes()
    {
        shared_ptr<Node> current_node = m_start;

        if (m_start->child)
        {
            while (current_node != m_goal)
            {
                shared_ptr<Node> del = current_node;
                current_node = current_node->child;
                _freeNode(del);
                del = nullptr;
            }

            _freeNode(current_node); // Delete the goal
        }
        else
        {
            // For cases when start is a goal we can just remove these two
            _freeNode(m_start);
            _freeNode(m_goal);
        }
    }

    // Functions for traversing the solution

    // Get start node
    UserState *getSolutionStart()
    {
        m_currentSolutionNode = m_start;
        if (m_start)
        {
            return &m_start->userState;
        }
        else
        {
            return nullptr;
        }
    }

    // Get next node
    UserState *getSolutionNext()
    {
        if (m_currentSolutionNode)
        {
            if (m_currentSolutionNode->child)
            {
                shared_ptr<Node> child = m_currentSolutionNode->child;
                m_currentSolutionNode = m_currentSolutionNode->child;
                return &child->userState;
            }
        }
        return nullptr;
    }

    // Get end node
    UserState *getSolutionEnd()
    {
        m_currentSolutionNode = m_goal;
        if (m_goal)
        {
            return &m_goal->userState;
        }
        else
        {
            return nullptr;
        }
    }

    // Step solution iterator backwards
    UserState *getSolutionPrev()
    {
        if (m_currentSolutionNode)
        {
            if (m_currentSolutionNode->parent)
            {
                Node *parent = m_currentSolutionNode->parent;
                m_currentSolutionNode = m_currentSolutionNode->parent;
                return &parent->userState;
            }
        }
        return nullptr;
    }

    // Get final cost of solution
    // Returns FLT_MAX if goal is not defined or there is no solution
    float getSolutionCost()
    {
        if (m_goal && m_state == SearchState::SUCCEEDED)
        {
            return m_goal->g;
        }
        else
        {
            return FLT_MAX;
        }
    }

    // Get the number of steps
    unsigned int getStepCount() { return m_stepsCount; }

private:
    // This is called when a search fails or is cancelled to free all used
    // memory
    void _freeAllNodes()
    {
        // will d delete all nodes
        m_openList.clear();
        m_closedList.clear();

        // delete the goal
        m_goal.reset();

        // delete the start
        m_start.reset();
    }

    // This call is made by the search class when the search ends. A lot of nodes may be
    // created that are still present when the search ends.
    void _freeUnusedNodes()
    {
        // iterate open list and delete unused nodes
        for (auto iterOpen = m_openList.begin(); iterOpen != m_openList.end(); iterOpen++)
        {
            shared_ptr<Node> current_node = (*iterOpen);

            // If node have no childs then we can delete it as uselsess
            if (!current_node->child)
            {
                _freeNode(current_node);
            }
        }

        // iterate closed list and delete unused nodes
        for (auto iterClosed = m_closedList.begin(); iterClosed != m_closedList.end(); iterClosed++)
        {
            shared_ptr<Node> current_node = (*iterClosed);

            // If node have no childs then we can delete it as uselsess
            if (!current_node->child)
            {
                _freeNode(current_node);
                current_node = nullptr;
            }
        }
    }

    // Node memory management
    shared_ptr<Node> _allocateNode()
    {
        return make_shared<Node>();
    }

    void _freeNode(shared_ptr<Node> node)
    {
        node.reset();
    }

    using iterator_t = typename deque<shared_ptr<Node>>::iterator;

    // Heap (simple vector but used as a heap)
    deque<shared_ptr<Node>> m_openList;

    // Closed list is a vector.
    deque<shared_ptr<Node>> m_closedList;

    // Successors is a vector filled out by the user each type successors to a node
    // are generated
    deque<shared_ptr<Node>> m_successors;

    public:
    deque<UserState> m_visited;
    private:
    SearchState m_state;

    unsigned int m_stepsCount;

    // Start and goal state pointers
    shared_ptr<Node> m_start;
    shared_ptr<Node> m_goal;

    shared_ptr<Node> m_currentSolutionNode;
};
