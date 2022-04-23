#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

class PoseEstimator{

    public:
        PoseEstimator(ros::NodeHandle *nh);
        void image_callback(const sensor_msgs::ImageConstPtr& msg);

    private:

        ros::NodeHandle node_handle_;
        ros::NodeHandle private_node_handle_;

        std::string image_topic_;
        std::string camera_info_topic_;
        std::string processed_image_publish_topic_;

        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        image_transport::Publisher image_pub_;

};