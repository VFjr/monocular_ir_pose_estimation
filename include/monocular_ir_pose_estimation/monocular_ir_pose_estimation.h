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

        cv::SimpleBlobDetector::Params blob_detector_params;
        cv::Ptr<cv::SimpleBlobDetector> blob_detector;

        bool camera_info_received;
        sensor_msgs::CameraInfo::ConstPtr camera_info_;
        cv::Mat camera_intrinsics_matrix;
        cv::Mat camera_distortion_coefficients_matrix;

        cv::Mat undistorted, bw_image, binary_bw_image, inverted_binary_bw_image;

        cv_bridge::CvImagePtr cv_ptr;
        std_msgs::Header msg_header;
        std::string frame_id;

        std::vector<cv::KeyPoint> keypoints;

        std::vector<cv::Point3f> points_3d;
        std::vector<cv::Point3f> axis_3d;

        float reprojection_mse(std::vector<cv::Point3f> points_3d_defined, std::vector<cv::Point2f> points_2d_detected, cv::Mat rvec, cv::Mat tvec);
        float reprojection_max_error(std::vector<cv::Point3f> points_3d_defined, std::vector<cv::Point2f> points_2d_detected, cv::Mat rvec, cv::Mat tvec);
        void draw_axis(cv::Mat& img, cv::Mat rvec, cv::Mat tvec);

    
};