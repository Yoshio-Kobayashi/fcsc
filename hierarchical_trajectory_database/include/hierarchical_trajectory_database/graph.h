#ifndef HIERARCHICAL_GRAPH
#define HIERARCHICAL_GRAPH
#include <iostream>
#include <unordered_map>

struct Point {
	int id_point;
	int id_cluster;
	int id_trajectory;
  std::vector<double> values;
	int total_values;
	bool is_end;
};

struct Node {
  int id;
  std::vector<int> state_ids;
  Point mean_state;
  std::vector<int> child_ids;
  int parent_id;
	std::unordered_map<int, double> edges;

	// 干渉チェック付きA*用
	int parent;
	bool is_open;
	double f;
};

struct Graph {
  int depth;
  std::vector<Node> nodes;
};

struct HierarchicalGraph {
  int depth;
	std::vector<Point> states;
  std::vector<Graph> graphs;
};

struct Path {
  int depth;
  std::vector<int> node_indices;
};

#endif
