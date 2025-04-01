#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    prev_odom.setIdentity();

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    m_odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("orb_slam3/odom", 10);
    std::cout << "slam changed" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    if (first_time) {
        try {
            auto t_cam = tf_buffer->lookupTransform(
                "base_link", msg->header.frame_id, tf2_ros::fromMsg(msg->header.stamp));
            cam_to_bl = tf2::transformToEigen(t_cam).cast<float>();
            first_time = false;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to base_link: %s",
                msg->header.frame_id.c_str(), ex.what());
        }
        return;
    }

    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::cout<<"one frame has been sent"<<std::endl;
    Sophus::SE3f sophus_tf = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
    Eigen::Isometry3f tf;
    tf = sophus_tf.cast<float>().matrix();
    
    Eigen::Isometry3f curr_odom = prev_odom * (cam_to_bl * tf * cam_to_bl.inverse());
    Eigen::Quaternionf q(curr_odom.linear());

    // Publish the camera trajectory
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = curr_odom.translation().x();
    odom.pose.pose.position.y = curr_odom.translation().y();
    odom.pose.pose.position.z = curr_odom.translation().z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = msg->header.stamp;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation.x = odom.pose.pose.orientation.x;
    odom_tf.transform.rotation.y = odom.pose.pose.orientation.y;
    odom_tf.transform.rotation.z = odom.pose.pose.orientation.z;
    odom_tf.transform.rotation.w = odom.pose.pose.orientation.w;

    tf_broadcaster->sendTransform(odom_tf);

    m_odom_publisher->publish(odom);
}
