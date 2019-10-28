// サンドイッチを棚にランダムに並べる
#include <ros/ros.h>
#include <iostream>
#include <random_numbers/random_numbers.h>
#include <fcsc_visual_tools/fcsc_visual_tools.h>
#include <fcsc_visual_tools/random_sandwich_generator.h>

using namespace std;

struct Shelf {
  double width;
  double depth;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "spawn_randomly_sandwich");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  Shelf shelf;
  std::string shelf_board_name;
  moveit_visual_tools::FCSCVisualTools object_visualizer;
  RandomSandwichGenerator random_sandwich_generator;
  int sandwich_num;
  double offset = 0.01;
  std::vector<fcsc_msgs::RecognizedObject> sandwiches;

  private_nh.param<int>("num", sandwich_num, 8);
  nh.getParam("shelf_size/shelfB/width", shelf.width);
  nh.getParam("shelf_size/shelfB/depth", shelf.depth);
  nh.getParam("stocking_area/sandwich/parent_frame_id", shelf_board_name);

  std::cerr << "shelf_width:" << shelf.width << '\n';
  std::cerr << "shelf_depth:" << shelf.depth << '\n';
  std::cerr << "shelf_board_name:" << shelf_board_name << '\n';
  std::cerr << "sandwich_num:" << sandwich_num <<  '\n';

  random_sandwich_generator.setRegionFrame(shelf_board_name);
  random_sandwich_generator.setRegionRange(0+offset, shelf.depth-offset, 0+offset, shelf.width-offset);
  random_sandwich_generator.generateSandwich(sandwich_num, sandwiches);

  for (size_t i = 0; i < sandwiches.size(); i++) {
    object_visualizer.spawnProduct(sandwiches[i].name, sandwiches[i].pose);
  }

  return (1);
}
