
#include <monocular_ir_pose_estimation/monocular_ir_pose_estimation.h>

PoseEstimator::PoseEstimator(ros::NodeHandle *nh): it_(*nh)
{
    node_handle_ = *nh;

    //init parameters before next thing



    image_sub_ = it_.subscribe("/camera/image_raw", 1,
      &PoseEstimator::image_callback, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

};


void PoseEstimator::image_callback(const sensor_msgs::ImageConstPtr& msg){

    std_msgs::Header msg_header = msg->header;
    std::string frame_id = msg_header.frame_id.c_str();
    
    ROS_INFO_STREAM("New Image from " << frame_id);

    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }



    // Draw a timestamp of the current date and time in the top left of the image
    // FIX-ME: std::asctime appends a '\n' character to the end of the string
    std::time_t result = msg_header.stamp.sec;
    std::stringstream ss;
    ss << std::asctime(std::localtime(&result));

    // Get the size of the text for measurement
    cv::Size text = cv::getTextSize(ss.str().c_str(), cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, 0);

    // Put the text in the bottom right corner
    cv::Point text_point = cvPoint(cv_ptr->image.cols - 20 - text.width, cv_ptr->image.rows - 20 - text.height);

    // Draw a black background behind text
    cv::rectangle(cv_ptr->image, text_point, text_point + cv::Point(text.width, -text.height), CV_RGB(0,0,0), cv::FILLED);

    // Draw the timestamp on the rectangle
    cv::putText(cv_ptr->image, ss.str().c_str(), text_point, cv::FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255,255,255));

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Draw an example crosshair
    cv::drawMarker(cv_ptr->image, cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2),  cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 1);



    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());


}