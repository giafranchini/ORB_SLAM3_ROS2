#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    prev_odom.setIdentity();

    publish_tf = this->declare_parameter<bool>("publish_tf", false);

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/rgb");
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/depth");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

    odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("orb_slam3/odom", 10);

    RCLCPP_INFO(this->get_logger(), "RGB-D node initialized");
}

RgbdSlamNode::~RgbdSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{

    if (first_time) {
        try {
            auto t_cam = tf_buffer->lookupTransform(
                msgRGB->header.frame_id, "base_link", tf2_ros::fromMsg(msgRGB->header.stamp));
            cam_to_bl = tf2::transformToEigen(t_cam).cast<float>();
            first_time = false;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to base_link: %s",
                msgRGB->header.frame_id.c_str(), ex.what());
        }
        return;
    }

    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f sophus_tf = m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));

    // std::cout << sophus_tf.matrix() << std::endl;

    Eigen::Isometry3f tf;
    tf = sophus_tf.cast<float>().matrix();
    
    // TODO(giafranchini): this needs to be checked
    // Eigen::Isometry3f curr_odom = prev_odom * (cam_to_bl * tf * cam_to_bl.inverse());
    Eigen::Isometry3f curr_odom = tf;
    Eigen::Quaternionf q(curr_odom.linear());

    // Publish the camera trajectory
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = msgRGB->header.stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = msgRGB->header.frame_id;
    odom.pose.pose.position.x = curr_odom.translation().x();
    odom.pose.pose.position.y = curr_odom.translation().y();
    odom.pose.pose.position.z = curr_odom.translation().z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    if (publish_tf) {
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = msgRGB->header.stamp;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = msgRGB->header.frame_id;
        odom_tf.transform.translation.x = odom.pose.pose.position.x;
        odom_tf.transform.translation.y = odom.pose.pose.position.y;
        odom_tf.transform.translation.z = odom.pose.pose.position.z;
        odom_tf.transform.rotation.x = odom.pose.pose.orientation.x;
        odom_tf.transform.rotation.y = odom.pose.pose.orientation.y;
        odom_tf.transform.rotation.z = odom.pose.pose.orientation.z;
        odom_tf.transform.rotation.w = odom.pose.pose.orientation.w;
    
        tf_broadcaster->sendTransform(odom_tf);
    }

    odom_publisher->publish(odom);
}
