#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace std;

double robotX = 0;
double robotY = 0;
double robotZ = 0;
double roll = 0;
double pitch = 0;
double yaw = 0;

nav_msgs::Odometry odometryIn;
ros::Publisher *pubOdometryPointer = NULL;
tf::StampedTransform transformToMap;
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;

void laserCloudAndOdometryHandler(const nav_msgs::Odometry::ConstPtr& odometry,
                                  const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
  // 更新里程计数据
  odometryIn = *odometry;

  // 设置TF变换
  transformToMap.setOrigin(
      tf::Vector3(odometryIn.pose.pose.position.x, odometryIn.pose.pose.position.y, odometryIn.pose.pose.position.z));
  transformToMap.setRotation(tf::Quaternion(odometryIn.pose.pose.orientation.x, odometryIn.pose.pose.orientation.y,
                                            odometryIn.pose.pose.orientation.z, odometryIn.pose.pose.orientation.w));

  // 设置并发布同步后的状态估计
  odometryIn.header.stamp = laserCloud2->header.stamp;
  odometryIn.header.frame_id = "map";
  odometryIn.child_frame_id = "sensor_at_scan";
  pubOdometryPointer->publish(odometryIn);

  // 发布TF变换
  transformToMap.stamp_ = laserCloud2->header.stamp;
  transformToMap.frame_id_ = "map";
  transformToMap.child_frame_id_ = "sensor_at_scan";
  tfBroadcasterPointer->sendTransform(transformToMap);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_scan");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  // ROS消息过滤器
  message_filters::Subscriber<nav_msgs::Odometry> subOdometry;
  message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloud;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> syncPolicy;
  typedef message_filters::Synchronizer<syncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
  
  // 订阅输入话题
  subOdometry.subscribe(nh, "/state_estimation", 1);
  subLaserCloud.subscribe(nh, "/registered_scan", 1);
  
  // 创建同步器
  sync_.reset(new Sync(syncPolicy(100), subOdometry, subLaserCloud));
  sync_->registerCallback(boost::bind(laserCloudAndOdometryHandler, _1, _2));

  // 创建发布器
  ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry>("/state_estimation_at_scan", 5);
  pubOdometryPointer = &pubOdometry;

  // TF广播器
  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  ros::spin();

  return 0;
}