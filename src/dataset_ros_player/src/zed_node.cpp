#include "spot.h"

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "spot_node");
  ros::NodeHandle nh("~");

  int scan_num = 1200;
  std::string sequence_dir = "/home/steven/kitti/spot/";

  SpotData spot_data(nh);
  spot_data.read_left_camera_poses("/home/steven/kitti/spot/spot.txt");
  spot_data.process_scans(sequence_dir, scan_num);
  ros::spin();

  return 0;
}
