#include "fcsc_moveit/smach_daihen_ur5_moveit_core.h"
#include "fcsc_moveit/utility.h"


bool FcscCore::pickupSandwich(fcsc_msgs::Manipulate::Request &req, fcsc_msgs::Manipulate::Response &res)
{

  geometry_msgs::PoseStamped pose_stmp_tmp;
  int first_index, last_index;
  int faceup_success_count = 0;
  int sandwich_size;
  bool is_scrap;
  std::vector<int> grasp_indices;

  ROS_WARN("[pickupSandwich]%s is %s", req.object.name.c_str(), getObjectStateString(req.object).c_str());

  move_group.setSupportSurfaceName(shelfB_name);

  res.success = false;

  asyncMoveArm(recover_pose_name);

  res.success = false;

  //把持姿勢のソート
  sortSandwichGrasps(req.object.name, grasp_indices, req.manipulation_type);

  for (size_t i = 0; i < grasp_indices.size(); i++) {

    product_grasps[SANDWICH].grasps[grasp_indices[i]].grasp_pose.header.frame_id = req.object.name;
    product_grasps[SANDWICH].grasps[grasp_indices[i]].pre_grasp_approach.direction.header.frame_id = req.object.name;

    ROS_WARN("[pickupSandwich]:%s", product_grasps[SANDWICH].grasps[grasp_indices[i]].id.c_str());

    bool success = pickup(req.object.name, product_grasps[SANDWICH].grasps[grasp_indices[i]]);
    if (success) {
      res.success = true;
      break;
    }

  }
  return (true);
}
