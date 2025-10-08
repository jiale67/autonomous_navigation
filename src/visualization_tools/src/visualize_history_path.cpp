#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZI>::Ptr trajectory(new pcl::PointCloud<pcl::PointXYZI>());
ros::Publisher pubTrajectory;

float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float travelingDis = 0;

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
    // 提取位置信息
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;
    
    static float prevX = vehicleX, prevY = vehicleY, prevZ = vehicleZ;
    float dx = vehicleX - prevX;
    float dy = vehicleY - prevY;
    float dz = vehicleZ - prevZ;
    float dis = sqrt(dx*dx + dy*dy + dz*dz);
    travelingDis += dis;
    prevX = vehicleX;
    prevY = vehicleY;
    prevZ = vehicleZ;
    
    pcl::PointXYZI point;
    point.x = vehicleX;
    point.y = vehicleY;
    point.z = vehicleZ;
    point.intensity = travelingDis; // 使用强度存储行驶距离
    trajectory->push_back(point);
    
    // 发布轨迹
    sensor_msgs::PointCloud2 trajectoryMsg;
    pcl::toROSMsg(*trajectory, trajectoryMsg);
    trajectoryMsg.header.stamp = odom->header.stamp;
    trajectoryMsg.header.frame_id = "map";
    pubTrajectory.publish(trajectoryMsg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_visualizer");
    ros::NodeHandle nh;
    
    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, odometryHandler);
    pubTrajectory = nh.advertise<sensor_msgs::PointCloud2>("/history_trajectory", 5);
    
    ROS_INFO("Trajectory visualizer started. Publishing to /trajectory");
    
    ros::spin();
    
    return 0;
}