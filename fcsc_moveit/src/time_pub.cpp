#include <ros/ros.h>
#include <std_msgs/Empty.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "minute_time");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::Publisher time_pub = nh.advertise<std_msgs::Empty> ("time_finish", 1);

  double min;

  // 作業の残り時間（分）の設定
  private_nh.param<double>("min", min, 17.5);

  // 残り時間の１分前になったら通知する
  min = min - 1.0;
  double sec = min * 60.0;

  ROS_WARN("%f [min]", min);

  //ros::Time start = ros::Time::now();
  ros::WallTime start_wall = ros::WallTime::now();

  ros::Rate loop_rate(1);

  while (ros::ok()) {
    ros::spinOnce();
    std_msgs::Empty msg;

    //ros::Time now = ros::Time::now();
    ros::WallTime now_wall = ros::WallTime::now();
    //ros::Duration pass_time = now - start;
    ros::WallDuration pass_time_wall = now_wall - start_wall;

    std::cerr << pass_time_wall << '\n';

    if (pass_time_wall >= ros::WallDuration(sec) && pass_time_wall <= ros::WallDuration(sec+5)) {  // number is second
      time_pub.publish (msg);
      ROS_WARN(" %f min passed. Time's up!!!!", min);
    }

    if (pass_time_wall >= ros::WallDuration(sec+5)) {
      break;
    }

/*
    if (pass_time_wall >= ros::WallDuration(1020)) {  // number is second
      for (int i = 0; i < 5; i++) {
        time_pub.publish (msg);
        //ROS_INFO("time : %u\n", pass_time_wall.sec);
      }
      break;
    }
*/

    loop_rate.sleep();
  }
}
