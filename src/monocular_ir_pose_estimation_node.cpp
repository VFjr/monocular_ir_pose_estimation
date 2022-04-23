
#include <monocular_ir_pose_estimation/monocular_ir_pose_estimation.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "monocular_ir_pose_estimation");
  ros::NodeHandle n;

  PoseEstimator estimator(&n);

  ROS_INFO("Monocular Pose Estimation Node ready");

  ros::spin();

  return 0;
}
