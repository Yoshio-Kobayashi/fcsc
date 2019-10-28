#include "hierarchical_trajectory_database/my_astar_search.h"
#include <ros/ros.h>

using namespace my_astar;

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_astar");

  std::priority_queue<AStarNode,std::vector<AStarNode>,CompareDist> queue;

  // 昇順で挿入
  std::cerr << "-- 昇順で挿入 --" << '\n';
  for (int i = 0; i < 10; ++i) {
    AStarNode node;
    node.id = i;
    node.f = (double)i;
    std::cerr << node.id << " " << node.f << '\n';
    queue.push(node);
  }
  std::cerr  << '\n';

  std::cerr << "-- キューを表示 --" << '\n';
  while (!queue.empty()) {
    AStarNode node;
    node = queue.top();
    queue.pop();
    std::cerr << node.id << " " << node.f << '\n';
  }
  std::cerr << '\n';

  // 降順で挿入
  std::cerr << "-- 降順で挿入 --" << '\n';
  for (int i = 10; i > 0; --i) {
    AStarNode node;
    node.id = i;
    node.f = (double)i;
    std::cerr << node.id << " " << node.f << '\n';
    queue.push(node);
  }
  std::cerr << '\n';

  std::cerr << "-- キューを表示 --" << '\n';
  while (!queue.empty()) {
    AStarNode node;
    node = queue.top();
    queue.pop();
    std::cerr << node.id << " " << node.f << '\n';
  }
  std::cerr << '\n';

  std::unordered_map<int, double> edges;
  for (int i = 0; i < 10; i++) {
    edges[i] = (double)(2 * i);
  }
  for (auto itr = edges.begin(); itr != edges.end(); ++itr) {
    std::cerr << itr->first << " " << itr->second << '\n';
  }
  std::cerr << '\n';
  for (int i = 0; i < 10; i++) {
    std::cerr << i << " " << edges[i] << '\n';
  }

  return 0;
}
