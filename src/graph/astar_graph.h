
#ifndef GRAPHNAVIGATION_GRAPH_ASTARGRAPH_H
#define GRAPHNAVIGATION_GRAPH_ASTARGRAPH_H

#include <vector>
#include <set>
#include <cstdint>
#include <cassert>
#include <limits>
#include "src/graph/navi_graph.h"

namespace GraphNavigation
{
namespace Graph
{

template <typename PositionT, typename CostT>
class AStarGraph : public NaviGraph<PositionT, CostT>
{
    typedef typename BaseGraph<PositionT, CostT>::out_edge_t edge;

  public:
        AStarGraph(std::function<size_t(PositionT)> hashT)
        : NaviGraph<PositionT, CostT>(hashT){}



    bool FindPath(const uint start, const uint goal, std::vector<uint> &path)
    {
        std::vector<CostT> cost_table(this->graph_.size(), inf);
        cost_table[start] = AstarHeuristic(start, goal);
        ;
        std::set<edge> front;
        front.insert({cost_table[start], start});
        while (!front.empty())
        {
            auto top = front.begin();
            int u = top->second;
            front.erase(top);
            for (auto next : this->graph_[u])
            {
                CostT weight = next.first;
                uint v = next.second;
                CostT new_weight = cost_table[u] + weight -
                                   AstarHeuristic(u, goal) + AstarHeuristic(v, goal);

                if (cost_table[v] > new_weight)
                {
                    if (front.find({cost_table[v], v}) != front.end())
                        front.erase(front.find({cost_table[v], v}));
                    cost_table[v] = new_weight;
                    front.insert({cost_table[v], v});
                }
                if (v == goal)
                {
                    setPath(cost_table, start, goal, path);
                    return true;
                }
            }
        }
        return false;
    }

    bool FindPath(const PositionT& start, const PositionT& goal, std::vector<PositionT> &path)
    {
        const int start_idx = this->GetIndex(start);
        const int goal_idx = this->GetIndex(goal);
        std::vector<uint> path_idx;
        bool res = FindPath(start_idx, goal_idx, path_idx);

        for (auto idx : path_idx)
        {
            path.push_back(this->positions()[idx]);
        }
        return res;    }

  private:
    void setPath(std::vector<CostT> &cost_table,
                 const uint start, const uint goal, std::vector<uint> &path)
    {
        uint c = goal;
        CostT d = cost_table[goal];

        for (uint i = 0; i < cost_table.size(); i++)
        {
            cost_table[i] = cost_table[i] - AstarHeuristic(i, goal);
        }

        while (c != start)
        {
            path.push_back(c);
            for (auto next : this->graph_[c])
            {
                int v = next.second;
                if (d > cost_table[v])
                {
                    d = cost_table[v];
                    c = v;
                }
            }
        }
        path.push_back(start);
    }

    CostT AstarHeuristic(const uint current, const uint goal) const
    {
        return this->BaseGraph<PositionT, CostT>::distance(current, goal);
    }

  private:
    CostT inf = std::numeric_limits<CostT>::infinity();
};

} // namespace Graph
} // namespace GraphNavigation
#endif //GRAPHNAVIGATION_GRAPH_BASHGRAPH_H