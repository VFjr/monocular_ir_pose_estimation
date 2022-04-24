
#include <monocular_ir_pose_estimation/monocular_ir_pose_estimation.h>

PoseEstimator::PoseEstimator(ros::NodeHandle *nh) : it_(*nh)
{
  node_handle_ = *nh;

  // init parameters before next thing

  image_sub_ = it_.subscribe("/camera/image_rect_color", 1, &PoseEstimator::image_callback, this);
  image_pub_ = it_.advertise("/image_converter/output_video", 1);

  camera_info_topic_ = "/camera/camera_info";

  // blob detector

  // Change thresholds
  blob_detector_params.minThreshold = 10;
  blob_detector_params.maxThreshold = 200;

  // Filter by Area.
  blob_detector_params.filterByArea = true;
  blob_detector_params.minArea = 10;

  // Filter by Circularity
  blob_detector_params.filterByCircularity = false;
  blob_detector_params.minCircularity = 0.1;

  // Filter by Convexity
  blob_detector_params.filterByConvexity = false;
  blob_detector_params.minConvexity = 0.87;

  // Filter by Inertia
  blob_detector_params.filterByInertia = false;
  blob_detector_params.minInertiaRatio = 0.01;

  blob_detector_params.minDistBetweenBlobs = 15;
  

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
        ROS_WARN_STREAM(camera_info_->D[i]);
      }

      camera_info_received = true;
    }
    else
    {
      ROS_WARN("Received invalid camera intrinsics (K all zeros)");
    }
  }

  // add the 3d Points

  /*
  points_3d.push_back(cv::Point3f(0, 0, 0));
  points_3d.push_back(cv::Point3f(0, -0.072, 0));
  points_3d.push_back(cv::Point3f(0, -0.097, 0));
  points_3d.push_back(cv::Point3f(-0.099, 0, 0));
  points_3d.push_back(cv::Point3f(-0.099, -0.097, 0));
  */


  points_3d.push_back(cv::Point3f(0, 0, 0));
  //points_3d.push_back(cv::Point3f(0.05, 0.05, -0.03));
  points_3d.push_back(cv::Point3f(0.05, -0.05, -0.03));
  points_3d.push_back(cv::Point3f(-0.05, 0.05, -0.03));
  points_3d.push_back(cv::Point3f(-0.05, -0.05, -0.03));
  points_3d.push_back(cv::Point3f(0.02, 0.067, -0.03));

  axis_3d.push_back(cv::Point3f(0, 0, 0));
  axis_3d.push_back(cv::Point3f(0.05, 0, 0));
  axis_3d.push_back(cv::Point3f(0, 0.05, 0));
  axis_3d.push_back(cv::Point3f(0, 0, 0.05));

  last_detection_successful = false;
};

void PoseEstimator::image_callback(const sensor_msgs::ImageConstPtr &msg)
{

  msg_header = msg->header;
  frame_id = msg_header.frame_id.c_str();

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::cvtColor(cv_ptr->image, bw_image, CV_BGR2GRAY);
  cv::threshold(bw_image, binary_bw_image, 50, 255, cv::THRESH_BINARY);
  cv::bitwise_not(binary_bw_image, inverted_binary_bw_image);

  blob_detector->detect(inverted_binary_bw_image, keypoints);

  cv::drawKeypoints(cv_ptr->image, keypoints, cv_ptr->image, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

  cv::Mat rvec(3, 1, cv::DataType<double>::type);
  cv::Mat tvec(3, 1, cv::DataType<double>::type);

  cv::Mat best_rvec(3, 1, cv::DataType<double>::type);
  cv::Mat best_tvec(3, 1, cv::DataType<double>::type);

  std::vector<cv::Point2f> points_2d_unordered, points_2d_ordered, points_2d_ordered_best, temp_points_2d_ordered;

  float temp_repr_error;

  if (keypoints.size() >= 5)
  {

    points_2d_unordered.clear();

    for (auto keypoint = keypoints.cbegin(); keypoint != keypoints.cend(); ++keypoint)
    {

      points_2d_unordered.push_back(keypoint->pt);
    }

    if (last_detection_successful)
    {
      // try the matching

      temp_points_2d_ordered = match_points_order(points_2d_ordered_last, points_2d_unordered);

      //ROS_WARN_STREAM("ord_last " << points_2d_ordered_last.size());
      //ROS_WARN_STREAM("unord " << points_2d_unordered.size());
      //ROS_WARN_STREAM("temp" << temp_points_2d_ordered.size());
      

      bool success = cv::solvePnP(points_3d, temp_points_2d_ordered, camera_intrinsics_matrix, cv::Mat(), rvec, tvec,false,cv::SOLVEPNP_EPNP);

      if (success)
      {

        // caluclate the error
        temp_repr_error = reprojection_mse(points_3d, temp_points_2d_ordered, rvec, tvec);

        // if error too high, do the brute forcing

        if (temp_repr_error < 20)
        {

          tvec.copyTo(best_tvec);
          rvec.copyTo(best_rvec);

          points_2d_ordered_last = temp_points_2d_ordered;
        }
        else
        {

          last_detection_successful = false;
        }
      }
      else
      {
        last_detection_successful = false;
      }
    }

    // do the brute forcing
    if (!last_detection_successful)
    {

      //ROS_WARN_STREAM("test1");
      std::vector<int> permutation_id;
      
      //make a permutation if vector of the size of unordered points

      for (int i = 0; i < points_2d_unordered.size(); i++)
      {
        permutation_id.push_back(i);
        //ROS_WARN_STREAM("test2");
      }

      float min_repr_error = std::numeric_limits<float>::max();

      do
      {

        points_2d_ordered.clear();
        for (int i = 0; i < points_3d.size(); i++)
        {
          points_2d_ordered.push_back(points_2d_unordered[permutation_id[i]]);
        }
    
        bool success = cv::solvePnP(points_3d, points_2d_ordered, camera_intrinsics_matrix, cv::Mat(), rvec, tvec,false,cv::SOLVEPNP_EPNP);
     
        if (success)
        {
          temp_repr_error = reprojection_mse(points_3d, points_2d_ordered, rvec, tvec);
          // ROS_WARN_STREAM("repr err: " << temp_repr_error);

          // ROS_WARN_STREAM("min err: " << min_repr_error);

          if (temp_repr_error < min_repr_error)
          {

            min_repr_error = temp_repr_error;
            tvec.copyTo(best_tvec);
            rvec.copyTo(best_rvec);

            points_2d_ordered_best = points_2d_ordered;
            //ROS_WARN_STREAM("test4");
          }
        }
        else
        {
          ROS_WARN_STREAM("solvepnp fail");
        }

      } while (std::next_permutation(permutation_id.begin(), permutation_id.end()));

      ROS_WARN_STREAM("min err: " << min_repr_error);
      //ROS_WARN_STREAM("test5");
      last_detection_successful = true;
      points_2d_ordered_last = points_2d_ordered_best;
    }

    // ROS_WARN_STREAM("points2d un size: " << points_2d_unordered.size());

    ROS_WARN_STREAM("points2d ord l size: " << points_2d_ordered_last.size());
    ROS_WARN_STREAM("temp repr errro: " << temp_repr_error);
    

    // ROS_WARN_STREAM("permutation_id size: " << permutation_id.size());

    
    draw_axis(cv_ptr->image, best_rvec, best_tvec);
  }

  // Output modified video stream
  image_pub_.publish(cv_ptr->toImageMsg());
}

float PoseEstimator::reprojection_mse(std::vector<cv::Point3f> points_3d_defined, std::vector<cv::Point2f> points_2d_detected, cv::Mat rvec, cv::Mat tvec)
{

  float mse_sum = 0;
  std::vector<cv::Point2f> projectedPoints;
  cv::projectPoints(points_3d_defined, rvec, tvec, camera_intrinsics_matrix, cv::Mat(), projectedPoints);

  for (int i = 0; i < points_3d_defined.size(); i++)
  {
    float x_d, y_d, x_p, y_p;

    x_d = points_2d_detected[i].x;
    y_d = points_2d_detected[i].y;

    x_p = projectedPoints[i].x;
    y_p = projectedPoints[i].y;

    float l2_distance_squared = pow(x_d - x_p, 2) + pow(y_d - y_p, 2);
    mse_sum += l2_distance_squared;
  }

  ROS_WARN_STREAM("ttest " << mse_sum);
  if (mse_sum == 0){
    mse_sum = std::numeric_limits<float>::max();
  }


  return mse_sum / points_3d_defined.size();
}

float PoseEstimator::reprojection_max_error(std::vector<cv::Point3f> points_3d_defined, std::vector<cv::Point2f> points_2d_detected, cv::Mat rvec, cv::Mat tvec)
{

  std::vector<cv::Point2f> projectedPoints;
  cv::projectPoints(points_3d_defined, rvec, tvec, camera_intrinsics_matrix, cv::Mat(), projectedPoints);

  float max_error = 0;
  for (int i = 0; i < points_3d_defined.size(); i++)
  {
    float x_d, y_d, x_p, y_p;

    x_d = points_2d_detected[i].x;
    y_d = points_2d_detected[i].y;

    x_p = projectedPoints[i].x;
    y_p = projectedPoints[i].y;

    float l2_distance = sqrt(pow(x_d - x_p, 2) + pow(y_d - y_p, 2));
    max_error = std::max(max_error, l2_distance);
  }

  

  return max_error;
}

void PoseEstimator::draw_axis(cv::Mat &img, cv::Mat rvec, cv::Mat tvec)
{

  std::vector<cv::Point2f> projected_axis_points;

  cv::projectPoints(axis_3d, rvec, tvec, camera_intrinsics_matrix, cv::Mat(), projected_axis_points);

  cv::line(img, projected_axis_points[0], projected_axis_points[1], cv::Scalar(255, 0, 0), 5);
  cv::line(img, projected_axis_points[0], projected_axis_points[2], cv::Scalar(0, 255, 0), 5);
  cv::line(img, projected_axis_points[0], projected_axis_points[3], cv::Scalar(0, 0, 255), 5);

  return;
}

std::vector<cv::Point2f> PoseEstimator::match_points_order(std::vector<cv::Point2f> old_ordered_vector, std::vector<cv::Point2f> new_unordered_vector)
{

  std::vector<cv::Point2f> new_ordered_vector;
  float min_distance, temp_distance;
  cv::Point2f min_distance_point;

  for (auto &old_point : old_ordered_vector)
  {

    min_distance = std::numeric_limits<float>::max();

    for (auto &new_point : new_unordered_vector)
    {

      temp_distance = l2_distance(old_point, new_point);

      if (temp_distance < min_distance)
      {
        min_distance = temp_distance;
        min_distance_point = new_point;
      }
    }

    new_ordered_vector.push_back(min_distance_point);
  }

  return new_ordered_vector;
}

float PoseEstimator::l2_distance(cv::Point2f p1, cv::Point2f p2)
{
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}