#include "kitti_odom.h"

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "kitti_odom_node");
  ros::NodeHandle nh("~");

  int scan_num = 1200;
  std::string sequence_dir = "/home/steven/kitti/10/";

  KittiOdomData kitti_odom_data(nh);
  kitti_odom_data.read_left_camera_poses("/home/steven/kitti/10/10.txt");
  kitti_odom_data.process_scans(sequence_dir, scan_num);
  ros::spin();

  return 0;
}
