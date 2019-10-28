#ifndef ASTAR_SEARCH_H_
#define ASTAR_SEARCH_H_

#include <iostream>
#include <string>
#include <utility>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/astar_search.hpp>
#include "hierarchical_trajectory_database/graph.h"

using AStarGraph = boost::adjacency_list<boost::listS, boost::vecS, boost::directedS,
boost::no_property, boost::property<boost::edge_weight_t, int>>;
using Vertex = boost::graph_traits<AStarGraph>::vertex_descriptor;
using Cost = double;

// ヒューリスティック関数
template <class AStarGraph, class CostType>
class distance_heuristic : public boost::astar_heuristic<AStarGraph, CostType> {
public:
  distance_heuristic(Vertex goal, AStarGraph& g)
  : goal_(goal), graph_(g) {}

  CostType operator()(Vertex u) const
  {
    // てきとうなコスト計算。
    return 0;
    // return get(boost::vertex_index, graph_, goal_) - get(boost::vertex_index, graph_, u);
  }

private:
  Vertex goal_;
  AStarGraph& graph_;
};

struct found_goal {};

template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
  using Vertex2 = typename boost::graph_traits<AStarGraph>::vertex_descriptor;
  astar_goal_visitor(Vertex2 goal) : m_goal(goal) {}

  // 頂点を調べる
  template <class AStarGraph>
  void examine_vertex(Vertex2 u, AStarGraph&)
  {
    if (u == m_goal) // ゴールが見つかった
    throw found_goal();
  }
private:
  Vertex2 m_goal;
};

bool astar_search(const Graph& graph, int start_node_id, int end_node_id, std::vector<int>& route)
{
  route.clear();

  int edge_size = 0;
  for (size_t i = 0; i < graph.nodes.size(); i++) {
    edge_size += graph.nodes[i].edges.size();
  }

  std::vector<std::pair<int, int> > edges(edge_size);
  std::vector<double> edge_weights(edge_size);

  for (size_t i = 0; i < graph.nodes.size(); i++) {
    const Node* node = &(graph.nodes[i]);
    int node_id = node->id;
    for (auto itr = node->edges.begin(); itr != node->edges.end(); ++itr) {
      int id = itr->first;
      double weight = itr->second;
      edges.push_back(std::pair<int, int>(node_id, id));
      edge_weights.push_back(weight);
    }
  }

  // 入力 graph, nodes, edges, weightsからA*探索用グラフ作成
  AStarGraph g(edges.begin(), edges.end(), edge_weights.begin(), graph.nodes.size());

  Vertex start = vertex(start_node_id, g);
  Vertex goal = vertex(end_node_id, g);

  std::vector<Vertex> parents(boost::num_vertices(g));
  std::vector<Cost> distances(boost::num_vertices(g));

  try {
    boost::astar_search_tree(g, start,
      distance_heuristic<AStarGraph, Cost>(goal, g),
      boost::predecessor_map(&parents[0]).
      distance_map(&distances[0]).
      visitor(astar_goal_visitor<Vertex>(goal)));
  }
  catch (found_goal fg) {
    // 経路なし
    if (parents[goal] == goal) {
      route.push_back(goal);
      return (true);
    }
    // 最短経路の頂点リストを作成
    for (Vertex v = goal; v != start; v = parents[v]) {
      auto it = route.begin();
      route.insert(it, v);
    }
    auto it = route.begin();
    route.insert(it, start);
    return (true);
  }
  return (false);
}

#endif
