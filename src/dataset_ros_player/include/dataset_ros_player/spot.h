#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <vector>

class SpotData{
  public:
    SpotData(ros::NodeHandle& nh) : nh_(nh) {
      left_color_image_publisher_ = nh_.advertise<sensor_msgs::Image>("left_color_image", 1);
      depth_image_publisher_ = nh_.advertise<sensor_msgs::Image>("depth_image", 1);
      left_color_camera_pose_publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("left_color_camera_pose", 1);
    }

    void process_scans(std::string sequence_dir, int scan_num) {
      
      std::string left_color_image_dir = sequence_dir + "image_2/";
      std::string depth_image_dir = sequence_dir + "depth_img/";
      
      for (int scan_id = 0; scan_id <= scan_num; ++scan_id) {
        
        ros::Duration(3.5).sleep();

        // Read pose
        Eigen::Matrix4d left_camera_pose = left_camera_poses_[scan_id];
        Eigen::Matrix4d init_trans_to_ground;
        init_trans_to_ground <<  1, 0, 0, 0,
                                 0, 0, 1, 0,
                                 0,-1, 0, 1,
                                 0, 0, 0, 1;
        Eigen::Matrix4d T_cam0_cam2 = Eigen::Matrix4d::Identity();
        T_cam0_cam2(0, 3) = -3.334597000000e+02 / 7.070912000000e+02;
        T_cam0_cam2(1, 3) = 1.930130000000e+00 / 7.070912000000e+02;
        T_cam0_cam2(2, 3) = 3.318498000000e-03;
        Eigen::Matrix4d left_color_camera_pose = init_trans_to_ground * left_camera_pose * T_cam0_cam2;
        tf::Transform tf_pose;
        Eigen::Affine3d eigen_affine_pose(left_color_camera_pose);
        tf::transformEigenToTF(eigen_affine_pose, tf_pose);

        // Read calib
        Eigen::Matrix4d Tr;
        Tr << -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03,
              -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02,
              9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
              0, 0, 0, 1;
        tf::Transform tf_Tr;
        Eigen::Affine3d eigen_affine_Tr(T_cam0_cam2.inverse() * Tr);
        tf::transformEigenToTF(eigen_affine_Tr, tf_Tr);
      
        char scan_id_c[256];
        sprintf(scan_id_c, "%06d", scan_id);
        ros::Time t_cur = ros::Time::now();

        // Read left color image
        std::string left_color_image_name = left_color_image_dir + std::string(scan_id_c) + ".png";
        cv::Mat left_color_image = cv::imread(left_color_image_name, cv::IMREAD_UNCHANGED);
        cv_bridge::CvImage left_color_msg;
        left_color_msg.encoding = sensor_msgs::image_encodings::RGB8;
        left_color_msg.image = left_color_image;
        left_color_msg.header.stamp = t_cur;
        
        // Read depth image
        std::string depth_image_name = depth_image_dir + std::string(scan_id_c) + ".png";
        cv::Mat depth_image = cv::imread(depth_image_name, cv::IMREAD_ANYDEPTH);
        cv_bridge::CvImage depth_msg;
        depth_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        depth_msg.image = depth_image;
        depth_msg.header.stamp = t_cur;
 
        // publish tf
        br_.sendTransform(tf::StampedTransform(tf_pose, t_cur, "map", "left_color_camera"));
       
        // publish msgs
        left_color_image_publisher_.publish(left_color_msg);
        depth_image_publisher_.publish(depth_msg);
        publish_posecov(eigen_affine_pose, t_cur);
        
        std::cout << scan_id << std::endl;
      }
    }

    void publish_posecov(Eigen::Affine3d pose, ros::Time t) {
      geometry_msgs::PoseWithCovarianceStamped pose_msg;
      pose_msg.header.frame_id = "left_camera";
      pose_msg.header.stamp = t;

      // set pose
      tf::poseEigenToMsg(pose, pose_msg.pose.pose);

      // set cov
      pose_msg.pose.covariance[6*0+0] = 0.0;
      pose_msg.pose.covariance[6*0+1] = 0.0;
      pose_msg.pose.covariance[6*0+2] = 0.0;
      pose_msg.pose.covariance[6*0+3] = 0.0;
      pose_msg.pose.covariance[6*0+4] = 0.0;
      pose_msg.pose.covariance[6*0+5] = 0.0;
      pose_msg.pose.covariance[6*1+0] = 0.0;
      pose_msg.pose.covariance[6*1+1] = 0.0;
      pose_msg.pose.covariance[6*1+2] = 0.0;
      pose_msg.pose.covariance[6*1+3] = 0.0;
      pose_msg.pose.covariance[6*1+4] = 0.0;
      pose_msg.pose.covariance[6*1+5] = 0.0;
      pose_msg.pose.covariance[6*2+0] = 0.0;
      pose_msg.pose.covariance[6*2+1] = 0.0;
      pose_msg.pose.covariance[6*2+2] = 0.0;
      pose_msg.pose.covariance[6*2+3] = 0.0;
      pose_msg.pose.covariance[6*2+4] = 0.0;
      pose_msg.pose.covariance[6*2+5] = 0.0;
      pose_msg.pose.covariance[6*3+0] = 0.0;
      pose_msg.pose.covariance[6*3+1] = 0.0;
      pose_msg.pose.covariance[6*3+2] = 0.0;
      pose_msg.pose.covariance[6*3+3] = 0.0;
      pose_msg.pose.covariance[6*3+4] = 0.0;
      pose_msg.pose.covariance[6*3+5] = 0.0;
      pose_msg.pose.covariance[6*4+0] = 0.0;
      pose_msg.pose.covariance[6*4+1] = 0.0;
      pose_msg.pose.covariance[6*4+2] = 0.0;
      pose_msg.pose.covariance[6*4+3] = 0.0;
      pose_msg.pose.covariance[6*4+4] = 0.0;
      pose_msg.pose.covariance[6*4+5] = 0.0;
      pose_msg.pose.covariance[6*5+0] = 0.0;
      pose_msg.pose.covariance[6*5+1] = 0.0;
      pose_msg.pose.covariance[6*5+2] = 0.0;
      pose_msg.pose.covariance[6*5+3] = 0.0;
      pose_msg.pose.covariance[6*5+4] = 0.0;
      pose_msg.pose.covariance[6*5+5] = 0.0;

      // publish
      left_color_camera_pose_publisher_.publish(pose_msg);
    }
        
    bool read_left_camera_poses(const std::string pose_name) {
      if (std::ifstream(pose_name)) {
        std::ifstream fPoses;
        fPoses.open(pose_name.c_str());
        while (!fPoses.eof()) {
          std::string s;
          std::getline(fPoses, s);
          if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            Eigen::Matrix4d t_matrix = Eigen::Matrix4d::Identity();
            for (int i = 0; i < 3; ++i)
              for (int j = 0; j < 4; ++j)
                ss >> t_matrix(i, j);
            left_camera_poses_.push_back(t_matrix);
          }
        }
        fPoses.close();
        return true;
        } else {
         ROS_ERROR_STREAM("Cannot open pose file " << pose_name);
         return false;
      }
    }

  private:
    cv::Mat left_color_image_;
    std::vector<Eigen::Matrix4d> left_camera_poses_;

    ros::NodeHandle nh_;
    ros::Publisher left_color_image_publisher_;
    ros::Publisher depth_image_publisher_;
    ros::Publisher left_color_camera_pose_publisher_;
    tf::TransformBroadcaster br_;
};
