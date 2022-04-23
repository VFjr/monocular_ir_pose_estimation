
#include <monocular_ir_pose_estimation/monocular_ir_pose_estimation.h>

PoseEstimator::PoseEstimator(ros::NodeHandle *nh): it_(*nh)
{
    node_handle_ = *nh;

    //init parameters before next thing



    image_sub_ = it_.subscribe("/camera/image_raw", 1,
      &PoseEstimator::image_callback, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    camera_info_topic_ = "/camera/camera_info";

    //blob detector

    // Change thresholds
    blob_detector_params.minThreshold = 10;
    blob_detector_params.maxThreshold = 200;

    // Filter by Area.
    blob_detector_params.filterByArea = true;
    blob_detector_params.minArea = 5;
  
    // Filter by Circularity
    blob_detector_params.filterByCircularity = false;
    blob_detector_params.minCircularity = 0.1;

    // Filter by Convexity
    blob_detector_params.filterByConvexity = false;
    blob_detector_params.minConvexity = 0.87;

    // Filter by Inertia
    blob_detector_params.filterByInertia = false;
    blob_detector_params.minInertiaRatio = 0.01;

    blob_detector = cv::SimpleBlobDetector::create(blob_detector_params);

    

  camera_info_received = false;

    // get camera info if not received already
	while (!camera_info_received)
	{

		camera_info_ = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_);
		bool valid_camera_info = false;

		for (size_t i = 0; i < camera_info_->K.size(); ++i)
		{
			if (camera_info_->K[i] != 0.0)
			{
				valid_camera_info = true;
				break;
			}
		}

		if (valid_camera_info)
		{
			camera_intrinsics_matrix = cv::Mat::zeros(3, 3, CV_64F);
			camera_distortion_coefficients_matrix = cv::Mat::zeros(1, 5, CV_64F);

			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					camera_intrinsics_matrix.at<double>(i, j) = camera_info_->K[i * 3 + j];
				}
			}

			for (int i = 0; i < 5; i++)
			{
				camera_distortion_coefficients_matrix.at<double>(0, i) = camera_info_->D[i];
			}

			camera_info_received = true;
		}
		else
		{
			ROS_WARN("Received invalid camera intrinsics (K all zeros)");
		}
    ROS_WARN("inside camera info loop");
	}
  ROS_WARN("outside camrea info loop");

};


void PoseEstimator::image_callback(const sensor_msgs::ImageConstPtr& msg){

    msg_header = msg->header;
    frame_id = msg_header.frame_id.c_str();
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::undistort(cv_ptr->image,undistorted, camera_intrinsics_matrix, camera_distortion_coefficients_matrix);

    cv::cvtColor(undistorted, bw_image, CV_BGR2GRAY);
    cv::threshold(bw_image, binary_bw_image, 100, 255, cv::THRESH_BINARY);
    cv::bitwise_not(binary_bw_image, inverted_binary_bw_image);

    blob_detector->detect(inverted_binary_bw_image,keypoints);

    cv::drawKeypoints(cv_ptr->image, keypoints, cv_ptr->image, cv::Scalar(0,255,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());

}