
#ifndef GRAPHNAVIGATION_GRAPH_BASHGRAPH_H
#define GRAPHNAVIGATION_GRAPH_BASHGRAPH_H

#include <vector>
#include <set>
#include <cstdint>
#include <cassert>
#include <unordered_map>

namespace GraphNavigation
{
namespace Graph
{

template <typename PositionT, typename CostT>
class BaseGraph
{
  public:
	typedef std::pair<CostT, uint> out_edge_t;
	typedef std::set<out_edge_t> node_t;

	BaseGraph(std::function<size_t(PositionT)> hashT) : nodemap_(20, hashT) {}

	virtual void AddNode(const PositionT p){
		node_t n;
		positions_.push_back(p);
		graph_[positions_.size()-1] = n;
		nodemap_.insert(std::pair<PositionT, uint>(p, positions_.size()-1));
	}

	virtual void AddEdge(const uint a, const uint b){
		CostT w = distance(a, b);
        graph_[a].insert({w, b});
        graph_[b].insert({w, a});
	}

	virtual void AddEdge(const uint a, const uint b, const CostT w){
        graph_[a].insert({w, b});
        graph_[b].insert({w, a});
	}

	virtual void AddEdge(const PositionT& pos_a, const PositionT& pos_b)
	{
		CostT w = (pos_a - pos_b).norm();
		uint a = GetIndex(pos_a);
		uint b = GetIndex(pos_b);
		graph_[a].insert({w, b});
		graph_[b].insert({w, a});
	}

	virtual CostT distance(const uint a, const uint b) const{
		return (positions_[a] - positions_[b]).norm();
	}

	const std::vector<PositionT> &positions() const
	{
		return positions_;
	}

	const std::unordered_map<uint, node_t> &graph() const
	{
		return graph_;
	}

	const int GetIndex(const PositionT& p) const
	{
		auto search = nodemap_.find(p);
		if (search != nodemap_.end())
		{
			return search->second;
		}
		else
		{
			return -1;
		}
	}
	
  protected:
	std::vector<PositionT> positions_;
	std::unordered_map<uint, node_t> graph_;
	std::unordered_map<PositionT, uint, std::function<size_t(PositionT)>> nodemap_;
};

} // namespace Graph
} // namespace GraphNavigation
#endif //GRAPHNAVIGATION_GRAPH_BASHGRAPH_H