// 自分実装のA*アルゴリズム
// A*探索中にロボットアームの干渉チェックをするために作成した
#ifndef MY_ASTAR_SEARCH_H_
#define MY_ASTAR_SEARCH_H_

#include <iostream>
#include <queue>
#include <cmath>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "hierarchical_trajectory_database/graph.h"

namespace my_astar
{
  struct AStarNode {
    int id;
    int parent;
    double f;
  };

  class CompareDist
  {
  public:
    bool operator()(AStarNode n1,AStarNode n2) {
      return (n1.f > n2.f);
    }
  };

  inline double getCost(Node& node1, Node& node2)
  {
    double cost = node1.edges[node2.id];
    return cost;
  }

  inline double squareDistance(const Point& p1, const Point& p2)
  {
    if (p1.values.size() != p2.values.size()) {
      return (-1.0);
    }

    double square_dist = 0.0;
    for (size_t i = 0; i < p1.values.size(); i++) {
      double dif = p1.values[i] - p2.values[i];
      square_dist += dif * dif;
    }
    return (square_dist);
  }

  inline double distance(const Point& p1, const Point& p2)
  {
    if (p1.values.size() != p2.values.size()) {
      return (-1.0);
    }

    double square_dist = 0.0;
    for (size_t i = 0; i < p1.values.size(); i++) {
      double dif = p1.values[i] - p2.values[i];
      square_dist += dif * dif;
    }
    return (std::sqrt(square_dist));
  }

  inline double getH(const Node& node1, const Node& node2, double max_disance)
  {
    double h = squareDistance(node1.mean_state, node2.mean_state) / max_disance;
    return h;
  }

  inline void createRoute(Graph& graph, int start_node_id, int goal_node_id, std::vector<int>& route)
  {
    for (int id = goal_node_id; id != start_node_id; id = graph.nodes[id].parent) {
      route.insert(route.begin(), id);
    }
    route.insert(route.begin(), start_node_id);
  }

  inline bool astar_search(Graph& graph, int start_node_id, int goal_node_id, std::vector<int>& route, bool check_collision=false, moveit::core::RobotState* robot_state=NULL, robot_state::JointModelGroup* joint_model_group=NULL, planning_scene_monitor::PlanningSceneMonitor* planning_scene_monitor=NULL)
  {
    route.clear();

    if (start_node_id == goal_node_id) {
      route.push_back(start_node_id);
      return (true);
    }

    // 全ノードをOPENに設定する
    for (size_t i_n = 0; i_n < graph.nodes.size(); i_n++) {
      graph.nodes[i_n].is_open = true;
    }

    // １　スタートノード、OPENリストを用意する。
    std::priority_queue<AStarNode,std::vector<AStarNode>,CompareDist> open_list;
    Node* start_node = &(graph.nodes[start_node_id]);

    // ２　ゴールノードを用意する．
    Node* goal_node = &(graph.nodes[goal_node_id]);

    double max_disance = squareDistance(start_node->mean_state, goal_node->mean_state);

    // ３ スタートノードをOPENリストに追加する．このとき g(S)=0 であり f(S)=h(S) となる．
    AStarNode astar_node;
    astar_node.id     = start_node->id;
    astar_node.f      = start_node->f = getH(*start_node, *goal_node, max_disance);
    astar_node.parent = start_node->parent = -1;
    open_list.push(astar_node);

    while (1) {
      // ４ Openリストが空なら探索は失敗とする（スタートからゴールにたどり着くような経路が存在しなかったことになる）．
      if (open_list.empty()) {
        return (false);
      }

      // ５ Openリストに格納されているノードの内、最小のf(n)を持つノードnを１つ取り出す．同じf(n)を持つノードが複数ある場合、何らかの手法でどれかのノードを選択する
      AStarNode n_astar_node;
      n_astar_node = open_list.top();
      open_list.pop();

      Node* n_node = &(graph.nodes[n_astar_node.id]);

      // open_listに重複してノードが格納されており、すでにノードの展開が終了(CLOSE)していた場合
      if (n_node->is_open == false) {
        continue;
      }

      n_node->f      = n_astar_node.f;
      n_node->parent = n_astar_node.parent;

      // 6 n == G であるなら探索を終了する．それ以外の場合はnをcloseにする
      if (n_node->id == goal_node_id) {
        // 9 探索終了後、発見されたゴールngから親を順次たどっていくとSからゴールngまでの最短経路が得られる．
        createRoute(graph, start_node_id, goal_node_id, route);
        return (true);
      } else {
        n_node->is_open = false;
      }

      // ７ nに隣接している全てのノード(隣接ノードをm)に対して以下の操作を行う
      for (auto itr = n_node->edges.begin(); itr != n_node->edges.end(); ++itr) {
        int id = itr->first;
        Node* m_node = &(graph.nodes[id]);

        // 干渉チェック
        if (check_collision) {
          collision_detection::CollisionRequest collision_request;
          collision_detection::CollisionResult  collision_result;
          std::vector<double> joint_positions;
          joint_positions.insert(joint_positions.end(), m_node->mean_state.values.begin(), m_node->mean_state.values.begin() + 6);
          robot_state->setJointGroupPositions(joint_model_group, joint_positions);
          planning_scene_monitor->getPlanningScene()->checkCollision(collision_request, collision_result, *robot_state);
          if (collision_result.collision) {
            m_node->is_open = false;
            continue;
          }
        }

        // 1 f'(m) = g(n) + COST(n,m) + h(m)を計算する．ここでCOST(n,m)はノードnからノードmへの移動するときのコストである．またg(n)はg(n)=f(n)-h(n)で求める
        double cost = getCost(*n_node, *m_node);
        double g = n_node->f - getH(*n_node, *goal_node, max_disance);
        double h = getH(*m_node, *goal_node, max_disance);

        // ２ mの状態に応じて以下の操作を行う
        // OPENにノードを追加する際には、m が Openリストに含まれているかは検証せず、重複を許して追加する。かつ、このときノードではなくIDのみを追加する
        AStarNode m_astar_node;
        m_astar_node.f      = g + cost + h;
        m_astar_node.parent = n_node->id;
        m_astar_node.id     = m_node->id;
        if (m_node->is_open == false && m_astar_node.f >= m_node->f) {
          continue;
        }
        m_node->is_open = true;
        open_list.push(m_astar_node);
      }
      // ８ 3以降を繰り返す
    }
  }
}

#endif
