#pragma once
#include <assert.h>
#include <memory>
#include <algorithm>
#include <deque>
#include <cfloat>

using std::deque;
using std::find_if;
using std::make_shared;
using std::shared_ptr;

/**
 * @brief Enum class for determination of current state of search
 */
enum class SearchState
{
    SEARCHING,
    SUCCEEDED,
    FAILED,
    OUT_OF_MEMORY
};

namespace detail
{
    /**
     * @brief Example of user state class that can be passed to template parameter of the AStarSearch.
     * Using of interface is avoided to preform compile time evaluation.
     */
    template <class T>
    class _AbstractStarState
    {
    public:
        virtual ~AStarState() {}
        /**
         * @brief Implementation of heuristic function which computes
         * the estimated cost to the goal node
         */
        virtual float goalDistanceEstimate(T &nodeGoal) = 0;
        /**
         * @brief Returns true if this node is the goal node
         */
        virtual bool isGoal(T &nodeGoal) = 0;
        /**
         * @brief Retrieves all successors to this node and adds them via astarsearch.addSuccessor()
         *
         */
        virtual bool getSuccessors(AStarSearch<T> *astarsearch, T *parent_node) = 0;
        /**
         * @brief Computes the cost of travelling from this node to the successor node
         */
        virtual float getCost(T &successor) = 0;
        /**
         * @brief Returns true if this node is the same as the rhs node
         */
        virtual bool isSameState(T &rhs) = 0;
    };
}

// 
/**
 * @brief The AStar search class. UserState is the users state space type
 * 
 * @tparam UserState class that satisfies _AbstractUserState interface
 */
template <class UserState>
class AStarSearch
{

public:
    /**
     * @brief A node represents a possible state in the search
     */
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

    /**
     * @brief Class comparator for sorting a heap using STL push_heap, make_heap, pop_heap
     */
    class NodeComparator
    {
    public:
        // x > y
        bool operator()(const shared_ptr<Node> x, const shared_ptr<Node> y) const
        {
            return x->f > y->f;
        }
    };

    /**
     * @brief Construct a new AStarSearch search
     * 
     * @param start Start state of search
     * @param goal Goal state of search
     */
    AStarSearch(UserState &start, UserState &goal)
    {
        m_start = _allocateNode();
        m_goal = _allocateNode();

        assert((m_start != nullptr && m_goal != nullptr));

        m_start->userState = start;
        m_goal->userState = goal;

        m_state = SearchState::SEARCHING;

        // Initialise the AStar specific parts of the start Node
        m_start->g = 0;
        m_start->h = m_start->userState.goalDistanceEstimate(m_goal->userState);
        m_start->f = m_start->g + m_start->h;
        m_start->parent = 0;

        // Push the start node on the open nodes as not expanded yet
        m_openNodes.push_back(m_start); // heap now unsorted

        // Sort back element into heap
        // As open nodes was empty we can skip step with make_head invocation
        std::push_heap(m_openNodes.begin(), m_openNodes.end(), NodeComparator());
    }

    /**
     * @brief Function to run search and get result state in terms of SearchState enum
     */
    SearchState preformSearch()
    {
        SearchState searchState;
        do
        {
            searchState = _searchStep();

        } while (searchState == SearchState::SEARCHING);

        return searchState;
    }

    /**
     * @brief Function that allows user to add new successors of given node to continue search 
     * 
     * @return true If adding was successful
     * @return false If errors with allocation occurs
     */
    bool addSuccessor(UserState &state)
    {
        shared_ptr<Node> node = _allocateNode();

        if (node)
        {
            node->userState = state;
            m_successors.push_back(node);

            return true;
        }

        return false;
    }

    /**
     * @brief Function to put all solution nodes in deque for comfortble use
     */
    deque<UserState> linearizeSolution()
    {
        deque<UserState> solution;
        shared_ptr<Node> current_node = m_start;
        while (current_node)
        {
            solution.push_back(current_node->userState);
            current_node = current_node->child;
        }
        return solution;
    }

    /**
     * @brief Get final cost of solution
     * 
     * @return Returns FLT_MAX if goal is not defined or there is no solution
     * and actual cost otherwise
     */
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

    /**
     * @brief Get number of steps that was made to find solution
     */
    unsigned int getStepCount() const { return m_expandedNodes.size(); }

    /**
     * @brief Get the visited nodes
     */
    const deque<shared_ptr<Node>> &getVisitedNodes() const { return m_expandedNodes; }

private:
    /**
     * @brief Function to preform one search step
     * 
     * @return SearchState that indicates the current state of the search
     */
    SearchState _searchStep()
    {
        if ((m_state == SearchState::SUCCEEDED) ||
            (m_state == SearchState::FAILED))
        {
            return m_state;
        }

        // If we have no other nodes to expand then there is no solution and the search is failed
        if (m_openNodes.empty())
        {
            m_state = SearchState::FAILED;
            return m_state;
        }

        // Pop the best node (the one with the lowest f)
        shared_ptr<Node> current_node = m_openNodes.front(); // get pointer to the node
        std::pop_heap(m_openNodes.begin(), m_openNodes.end(), NodeComparator());
        m_openNodes.pop_back();

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
                m_successors.clear(); // empty vector of successor nodes to current_node

                // free up everything else we allocated
                _freeNode(current_node);

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

                iterator_t openListResult = find_if(m_openNodes.begin(), m_openNodes.end(), [successor](shared_ptr<Node> node)
                                                    { return node->userState.isSameState((*successor)->userState); });

                // we found same state on open
                if (openListResult != m_openNodes.end())
                {
                    if ((*openListResult)->g <= newg)
                    {
                        // instance in the Open is cheaper than the current one
                        _freeNode((*successor));
                        // Continue with next successor
                        continue;
                    }
                }

                iterator_t closedListResult = find_if(m_expandedNodes.begin(), m_expandedNodes.end(), [successor](shared_ptr<Node> node)
                                                      { return node->userState.isSameState((*successor)->userState); });

                // we found this state on closed
                if (closedListResult != m_expandedNodes.end())
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
                // 1 - Update old version of this node in closed list as we have found better version
                // 2 - Move it from closed to open list to be able to investigate better solution
                // 3 - Sort heap again in open list

                if (closedListResult != m_expandedNodes.end())
                {
                    // Update closed node with successor node AStar data
                    (*closedListResult)->parent = (*successor)->parent;
                    (*closedListResult)->g = (*successor)->g;
                    (*closedListResult)->h = (*successor)->h;
                    (*closedListResult)->f = (*successor)->f;

                    // Free successor node
                    _freeNode((*successor));

                    // Push closed node into open list
                    m_openNodes.push_back((*closedListResult));

                    // Remove closed node from closed list
                    m_expandedNodes.erase(closedListResult);

                    // Sort back element into heap
                    std::push_heap(m_openNodes.begin(), m_openNodes.end(), NodeComparator());
                }

                // Successor in open list
                // 1 - Update old version of this node in open list as we have found better version
                // 2 - sort heap again in open list

                else if (openListResult != m_openNodes.end())
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
                    std::make_heap(m_openNodes.begin(), m_openNodes.end(), NodeComparator());
                }

                // New successor
                // 1 - Move it from successors to open list
                // 2 - sort heap again in open list

                else
                {
                    // Push successor node into open list
                    m_openNodes.push_back((*successor));

                    // Sort back element into heap
                    std::push_heap(m_openNodes.begin(), m_openNodes.end(), NodeComparator());
                }
            }

            // push current_node onto Closed, as we have expanded it now
            m_expandedNodes.push_back(current_node);
        }
        return m_state;
    }

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
    deque<shared_ptr<Node>> m_openNodes;

    // Closed list is a vector.
    deque<shared_ptr<Node>> m_expandedNodes;

    // Successors is a vector filled out by the user each type successors to a node
    // are generated
    deque<shared_ptr<Node>> m_successors;

    SearchState m_state;

    // Start and goal state pointers
    shared_ptr<Node> m_start;
    shared_ptr<Node> m_goal;
};
