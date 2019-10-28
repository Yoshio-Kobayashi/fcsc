#ifndef RAND_SAND_GEN
#define RAND_SAND_GEN

// サンドイッチを棚にランダムに並べる
#include <ros/ros.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <random_numbers/random_numbers.h>
#include <fcsc_msgs/RecognizedObject.h>

class RandomSandwichGenerator {
public:
  RandomSandwichGenerator();
  void generateSandwich(int num, std::vector<fcsc_msgs::RecognizedObject> &sandwiches);
  void setRegionFrame(std::string frame_id);
  void setRegionRange(double min_x, double max_x, double min_y, double max_y);

private:
  struct SandwichSize {
    double width;
    double depth;
    double height;
  };

  struct Line {
    geometry_msgs::Point start_point;
    geometry_msgs::Point end_point;
  };

  bool isIntersected(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy);

  double min_x, max_x, min_y, max_y;
  std::string regin_frame_id;
  SandwichSize sandwich_size;
  random_numbers::RandomNumberGenerator rand_num_generator;
};

#endif
