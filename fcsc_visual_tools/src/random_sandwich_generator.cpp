// サンドイッチを棚にランダムに並べる
#include <fcsc_visual_tools/random_sandwich_generator.h>
#include <fcsc_msgs/RecognizedObject.h>

RandomSandwichGenerator::RandomSandwichGenerator()
{
  sandwich_size.width = 0.08;
  sandwich_size.depth = 0.09;
  sandwich_size.height = 0.1;
  // sandwich_size.width = 0.07;
  // sandwich_size.depth = 0.09;
  // sandwich_size.height = 0.14;
}

void RandomSandwichGenerator::setRegionFrame(std::string frame_id)
{
  regin_frame_id = frame_id;
}

void RandomSandwichGenerator::setRegionRange(double min_x, double max_x, double min_y, double max_y)
{
  this->min_x = min_x;
  this->max_x = max_x;
  this->min_y = min_y;
  this->max_y = max_y;
}

bool RandomSandwichGenerator::isIntersected(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy)
{
  double ta = (cx - dx) * (ay - cy) + (cy - dy) * (cx - ax);
  double tb = (cx - dx) * (by - cy) + (cy - dy) * (cx - bx);
  double tc = (ax - bx) * (cy - ay) + (ay - by) * (ax - cx);
  double td = (ax - bx) * (dy - ay) + (ay - by) * (ax - dx);

  return tc * td <= 0 && ta * tb <= 0;
}


void RandomSandwichGenerator::generateSandwich(int num, std::vector<fcsc_msgs::RecognizedObject> &sandwiches)
{
  fcsc_msgs::RecognizedObject sandwich;
  std::vector<Line> lines;

  sandwiches.clear();

  sandwich.pose.header.frame_id = regin_frame_id;
  sandwich.type = fcsc_msgs::RecognizedObject::SANDWICH;

  // 棚の領域（幅、奥行き）を線分のベクトル型に追加
  // (0,0) -> (x,0) ->(x,y) -> (0,y)
  lines.resize(4);
  for (size_t i = 0; i < lines.size(); i++) {
    geometry_msgs::Point p;
    i == 1 || i == 2 ? p.x = max_x : p.x = min_x;
    i == 2 || i == 3 ? p.y = max_y : p.y = min_y;
    lines[i].start_point = p;
    lines[(lines.size() - 1 + i) % lines.size()].end_point = p;
  }

  for (size_t n_sanwich = 0; n_sanwich < num; n_sanwich++) {
    std::vector<geometry_msgs::Point> bottom_vertices; // サンドイッチの床接地面の頂点の位置
    double roll, pitch, yaw;
    int pattern;
    std::stringstream ss;

    ss << "sandwich_" << (char)('A' + n_sanwich);

    sandwich.name = ss.str();

    // ランダムに配置パターンを決定する
    if (rand_num_generator.uniform01() >= 0.5) {
      pattern = fcsc_msgs::RecognizedObject::BOTTOM;
    } else {
      pattern = rand_num_generator.uniformInteger(fcsc_msgs::RecognizedObject::BACK, fcsc_msgs::RecognizedObject::FRONT);
    }

    switch (pattern) {
      // 立った状態
      case fcsc_msgs::RecognizedObject::BOTTOM: {
        roll = 0;
        pitch = 0;
        sandwich.pose.pose.position.z = 0;
        sandwich.state = fcsc_msgs::RecognizedObject::BOTTOM;
        for (size_t i = 0; i < 4; i++) {
          geometry_msgs::Point vertix;
          i == 1 || i == 2 ? vertix.x = 0 : vertix.x = -sandwich_size.depth;
          i == 2 || i == 3 ? vertix.y = sandwich_size.width / 2.0 : vertix.y = -sandwich_size.width / 2.0;
          bottom_vertices.push_back(vertix);
        }
        break;
      }
      // 背面を下にした状態
      case fcsc_msgs::RecognizedObject::BACK: {
        roll = 0;
        pitch = M_PI / 2;
        sandwich.pose.pose.position.z = 0;
        sandwich.state = fcsc_msgs::RecognizedObject::BACK;
        for (size_t i = 0; i < 4; i++) {
          geometry_msgs::Point vertix;
          i == 1 || i == 2 ? vertix.x = sandwich_size.height : vertix.x = 0;
          i == 2 || i == 3 ? vertix.y = sandwich_size.width / 2.0 : vertix.y = -sandwich_size.width / 2.0;
          bottom_vertices.push_back(vertix);
        }
        break;
      }
      // 側面を下にした状態
      case fcsc_msgs::RecognizedObject::RIGHT_SIDE:
      case fcsc_msgs::RecognizedObject::LEFT_SIDE: {
        // 右側面や左側面を下にするのかをランダムに決定する
        int sign;
        if (rand_num_generator.uniformInteger(0, 1) == 1) {
          sandwich.state = fcsc_msgs::RecognizedObject::RIGHT_SIDE;
          sign = 1;
        } else {
          sandwich.state = fcsc_msgs::RecognizedObject::LEFT_SIDE;
          sign = -1;
        }
        roll = sign * M_PI / 2.0;
        pitch = 0;
        sandwich.pose.pose.position.z = sandwich_size.width / 2.0;
        for (size_t i = 0; i < 3; i++) {
          geometry_msgs::Point vertix;
          i == 2 ? vertix.x = -sandwich_size.depth : vertix.x = 0;
          i == 0 ? vertix.y = -sign * sandwich_size.height : vertix.y = 0;
          bottom_vertices.push_back(vertix);
        }
        break;
      }
      // 前面を下にした状態
      case fcsc_msgs::RecognizedObject::FRONT: {
        double ang = atan(sandwich_size.height / sandwich_size.depth);
        roll = 0;
        pitch = -(M_PI - ang);
        sandwich.pose.pose.position.z = sandwich_size.height * cos(ang);
        sandwich.state = fcsc_msgs::RecognizedObject::FRONT;
        for (size_t i = 0; i < 4; i++) {
          geometry_msgs::Point vertix;
          i == 1 || i == 2 ? vertix.x = sandwich_size.depth * cos(ang): vertix.x = -sandwich_size.height * sin(ang);
          i == 2 || i == 3 ? vertix.y = sandwich_size.width / 2.0 : vertix.y = -sandwich_size.width / 2.0;
          bottom_vertices.push_back(vertix);
        }
        break;
      }
    }

    // 他のサンドイッチや棚に干渉しない位置・姿勢が得られるまでループし続ける
    while (1) {
      // ランダムに姿勢を決める
      yaw = rand_num_generator.uniformReal(0, 2 * M_PI);
      sandwich.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

      // ランダムに位置を決める
      sandwich.pose.pose.position.x = rand_num_generator.uniformReal(min_x, max_x);
      sandwich.pose.pose.position.y = rand_num_generator.uniformReal(min_y, max_y);

      // サンドイッチの位置・姿勢からサンドイッチの床接地面の線分を計算
      std::vector<Line> bottom_lines(bottom_vertices.size());
      for (size_t i = 0; i < bottom_lines.size(); i++) {
        geometry_msgs::Point vertex;
        geometry_msgs::Point p = bottom_vertices[i];

        // 基準座標系で線分の 回転 + 移動
        vertex.x = p.x * cos(yaw) - p.y * sin(yaw) + sandwich.pose.pose.position.x;
        vertex.y = p.x * sin(yaw) + p.y * cos(yaw) + sandwich.pose.pose.position.y;
        bottom_lines[i].start_point = vertex;
        bottom_lines[(bottom_lines.size() - 1 + i) % bottom_lines.size()].end_point = vertex;
      }

      // 線分交差判定で干渉チェック
      bool is_intersected = false;
      for (size_t i = 0; i < lines.size(); i++) {
        for (size_t j = 0; j < bottom_lines.size(); j++) {
          if (isIntersected( lines[i].start_point.x, lines[i].start_point.y,
                              lines[i].end_point.x, lines[i].end_point.y,
                              bottom_lines[j].start_point.x, bottom_lines[j].start_point.y,
                              bottom_lines[j].end_point.x, bottom_lines[j].end_point.y) ) {
            // 干渉していれば姿勢・位置変更
            is_intersected = true;
            break;
          }
        }
        if (is_intersected) {
          break;
        }
      }

      if (!is_intersected) {
        lines.insert(lines.end(), bottom_lines.begin(), bottom_lines.end());
        break;
      }

    }

    // サンドイッチを出す
    sandwiches.push_back(sandwich);
  }

}
