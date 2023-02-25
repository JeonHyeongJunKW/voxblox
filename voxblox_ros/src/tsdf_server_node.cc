#include "voxblox_ros/tsdf_server.h"

#include <gflags/gflags.h>//logging api

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox"); //노드 이름선언
  google::InitGoogleLogging(argv[0]); //
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh; //노드 생성
  ros::NodeHandle nh_private("~"); //노드 또 생성

  voxblox::TsdfServer node(nh, nh_private);

  ros::spin();
  return 0;
}
