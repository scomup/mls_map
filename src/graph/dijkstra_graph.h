#ifndef GRAPHNAVIGATION_GRAPH_DIJKSTRAGRAPH_H
#define GRAPHNAVIGATION_GRAPH_DIJKSTRAGRAPH_H

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
class DijkstraGraph : public NaviGraph<PositionT, CostT>
{
    typedef typename BaseGraph<PositionT, CostT>::out_edge_t edge;

  public:
      DijkstraGraph(std::function<size_t(PositionT)> hashT)
        : NaviGraph<PositionT, CostT>(hashT){}


    bool FindPath(const uint start, const uint goal, std::vector<uint> &path)
    {
        std::vector<CostT> cost_table(this->graph_.size(), inf);
        cost_table[start] = 0;
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
                if (cost_table[v] > cost_table[u] + weight)
                {
                    if (front.find({cost_table[v], v}) != front.end())
                        front.erase(front.find({cost_table[v], v}));
                    cost_table[v] = cost_table[u] + weight;
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
        if(start_idx == -1){
            std::cout<<"cont find start point!\n";
            return -1;
        }
        const int goal_idx = this->GetIndex(goal);
        if (goal_idx == -1)
        {
            std::cout << "cont find start point!\n";
            return -1;
        }

        std::vector<uint> path_idx;
        bool res = FindPath(start_idx, goal_idx, path_idx);

        for (auto idx : path_idx)
        {
            path.push_back(this->positions()[idx]);
        }
        return res;
    }

  private:
    void setPath(const std::vector<CostT> &cost_table,
                 const uint start, const uint goal, std::vector<uint> &path)
    {
        uint c = goal;
        CostT d = cost_table[goal];
        while (c != start)
        {
            path.push_back(c);
            for (const auto &next : this->graph_[c])
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

  private:
    CostT inf = std::numeric_limits<CostT>::infinity();
};

} // namespace Graph
} // namespace GraphNavigation
#endif //GRAPHNAVIGATION_GRAPH_BASHGRAPH_H