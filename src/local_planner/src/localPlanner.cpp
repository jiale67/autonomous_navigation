#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <algorithm> 

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h> 
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h> 

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

string pathFolder;
double vehicleLength = 0.6;
double vehicleWidth = 0.6;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
bool twoWayDrive = true;
double laserVoxelSize = 0.05;
double terrainVoxelSize = 0.2;
bool useTerrainAnalysis = false;
bool checkObstacle = true;
double adjacentRange = 3.5;
double obstacleHeightThre = 0.2;
double groundHeightThre = 0.1;
double costHeightThre = 0.1;
double costScore = 0.02;
bool useCost = false;
const int laserCloudStackNum = 1;
int laserCloudCount = 0;
int pointPerPathThre = 2;
double minRelZ = -0.5;
double maxRelZ = 0.25;
double maxSpeed = 1.0;
double dirWeight = 0.02;
double dirThre = 90.0;
bool dirToVehicle = false;
double pathScale = 1.0;
double minPathScale = 0.75;
double pathScaleStep = 0.25;
bool pathScaleBySpeed = true;
double minPathRange = 1.0;
double pathRangeStep = 0.5;
bool pathRangeBySpeed = true;
bool pathCropByGoal = true;
double goalClearRange = 0.5;
double goalX = 0;
double goalY = 0;
bool useGlobalPlan = true; 

std::vector<std::pair<double, double>> globalPathPoints;
int currentGoalIndex = 1; 
double goalUpdateDistance = 2.0; 
bool newGlobalPath = false; 

std::pair<double, double> lastGlobalPathEndPoint = std::make_pair(NAN, NAN);

float joySpeed = 0;
float joyDir = 0;

const int pathNum = 343;
const int groupNum = 7;
float gridVoxelSize = 0.02;
float searchRadius = 0.45;
float gridVoxelOffsetX = 3.2;
float gridVoxelOffsetY = 4.5;
const int gridVoxelNumX = 161;
const int gridVoxelNumY = 451;
const int gridVoxelNum = gridVoxelNumX * gridVoxelNumY;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStack[laserCloudStackNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[groupNum];

int pathList[pathNum] = {0};
float endDirPathList[pathNum] = {0};
int clearPathList[36 * pathNum] = {0};
float pathPenaltyList[36 * pathNum] = {0};
float clearPathPerGroupScore[36 * groupNum] = {0};
std::vector<int> correspondences[gridVoxelNum];

bool newLaserCloud = false;
bool newTerrainCloud = false;

double odomTime = 0;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter, terrainDwzFilter;

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  odomTime = odom->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odom->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odom->pose.pose.position.z;
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
  if (!useTerrainAnalysis) {
    laserCloud->clear();
    pcl::fromROSMsg(*laserCloud2, *laserCloud);

    pcl::PointXYZI point;
    laserCloudCrop->clear();
    int laserCloudSize = laserCloud->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      point = laserCloud->points[i];

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
      if (dis < adjacentRange) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        laserCloudCrop->push_back(point);
      }
    }

    laserCloudDwz->clear();
    laserDwzFilter.setInputCloud(laserCloudCrop);
    laserDwzFilter.filter(*laserCloudDwz);

    newLaserCloud = true;
  }
}

void terrainCloudHandler(const sensor_msgs::PointCloud2ConstPtr& terrainCloud2)
{
  if (useTerrainAnalysis) {
    terrainCloud->clear();
    pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

    pcl::PointXYZI point;
    terrainCloudCrop->clear();
    int terrainCloudSize = terrainCloud->points.size();
    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud->points[i];

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
      if (dis < adjacentRange && (point.intensity > obstacleHeightThre || useCost)) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        terrainCloudCrop->push_back(point);
      }
    }

    terrainCloudDwz->clear();
    terrainDwzFilter.setInputCloud(terrainCloudCrop);
    terrainDwzFilter.filter(*terrainCloudDwz);

    newTerrainCloud = true;
  }
}

void globalPathHandler(const visualization_msgs::MarkerArray::ConstPtr& globalPath)
{
  std::vector<std::pair<int, std::pair<double, double>>> tempPoints;
  for (const auto& marker : globalPath->markers) {
    if (marker.type == visualization_msgs::Marker::SPHERE) {
      double x = marker.pose.position.x;
      double y = marker.pose.position.y;
      tempPoints.push_back(std::make_pair(marker.id, std::make_pair(x, y)));
    }
  }
  

  if (tempPoints.empty()) return;
  std::sort(tempPoints.begin(), tempPoints.end(), 
    [](const std::pair<int, std::pair<double, double>>& a, 
       const std::pair<int, std::pair<double, double>>& b) {
      return a.first < b.first;
    });
  
  std::pair<double, double> newEndPoint = tempPoints.back().second;
  bool isNewPath = false;
  if (std::isnan(lastGlobalPathEndPoint.first) || 
      std::abs(newEndPoint.first - lastGlobalPathEndPoint.first) > 0.01 ||
      std::abs(newEndPoint.second - lastGlobalPathEndPoint.second) > 0.01) {
    isNewPath = true;
    lastGlobalPathEndPoint = newEndPoint;
  }
  if (isNewPath) {
    globalPathPoints.clear();
    for (const auto& point : tempPoints) {
      globalPathPoints.push_back(point.second);
    }
    
    currentGoalIndex = 1;
    if (currentGoalIndex >= globalPathPoints.size()) {
      currentGoalIndex = globalPathPoints.size() - 1;
    }
    
    goalX = globalPathPoints[currentGoalIndex].first;
    goalY = globalPathPoints[currentGoalIndex].second;
    ROS_INFO("Received new global path with %ld points. Setting initial goal to point %d: (%.2f, %.2f)", 
             globalPathPoints.size(), currentGoalIndex, goalX, goalY);
  }
  
  newGlobalPath = isNewPath;
}

void simpleGoalHandler(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
  goalX = goal->pose.position.x;
  goalY = goal->pose.position.y;
  globalPathPoints.clear();
  currentGoalIndex = 0;
  
  ROS_INFO("Received new simple goal: (%.2f, %.2f)", goalX, goalY);
}

void publishGoalMarker(ros::Publisher& pub, double goalX, double goalY)
{
  visualization_msgs::Marker goalMarker;
  goalMarker.header.frame_id = "map"; 
  goalMarker.header.stamp = ros::Time::now();
  goalMarker.ns = "goal";
  goalMarker.id = 0;
  goalMarker.type = visualization_msgs::Marker::SPHERE;
  goalMarker.action = visualization_msgs::Marker::ADD;
  goalMarker.pose.position.x = goalX;
  goalMarker.pose.position.y = goalY;
  goalMarker.pose.position.z = 1.0; 
  goalMarker.pose.orientation.x = 0.0;
  goalMarker.pose.orientation.y = 0.0;
  goalMarker.pose.orientation.z = 0.0;
  goalMarker.pose.orientation.w = 1.0;
  goalMarker.scale.x = 0.8; 
  goalMarker.scale.y = 0.8;
  goalMarker.scale.z = 0.8;
  goalMarker.color.r = 1.0; 
  goalMarker.color.g = 0.6;
  goalMarker.color.b = 0.6;
  goalMarker.color.a = 1.0; 
  goalMarker.lifetime = ros::Duration(0.5);
  
  pub.publish(goalMarker);
}

int readPlyHeader(FILE *filePtr)
{
  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(filePtr, "%s", str);
    if (val != 1) {
      printf ("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(filePtr, "%d", &pointNum);
      if (val != 1) {
        printf ("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  return pointNum;
}

void readStartPaths()
{
  string fileName = pathFolder + "/startPaths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZ point;
  int val1, val2, val3, val4, groupID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    if (groupID >= 0 && groupID < groupNum) {
      startPaths[groupID]->push_back(point);
    }
  }

  fclose(filePtr);
}

void readPathList()
{
  string fileName = pathFolder + "/pathList.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  if (pathNum != readPlyHeader(filePtr)) {
    printf ("\nIncorrect path number, exit.\n\n");
    exit(1);
  }

  int val1, val2, val3, val4, val5, pathID, groupID;
  float endX, endY, endZ;
  for (int i = 0; i < pathNum; i++) {
    val1 = fscanf(filePtr, "%f", &endX);
    val2 = fscanf(filePtr, "%f", &endY);
    val3 = fscanf(filePtr, "%f", &endZ);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
      pathList[pathID] = groupID;
      endDirPathList[pathID] = 2.0 * atan2(endY, endX) * 180 / PI;
    }
  }

  fclose(filePtr);
}

void readCorrespondences()
{
  string fileName = pathFolder + "/correspondences.txt";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    printf ("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  int val1, gridVoxelID, pathID;
  for (int i = 0; i < gridVoxelNum; i++) {
    val1 = fscanf(filePtr, "%d", &gridVoxelID);
    if (val1 != 1) {
      printf ("\nError reading input files, exit.\n\n");
        exit(1);
    }

    while (1) {
      val1 = fscanf(filePtr, "%d", &pathID);
      if (val1 != 1) {
        printf ("\nError reading input files, exit.\n\n");
          exit(1);
      }

      if (pathID != -1) {
        if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 && pathID < pathNum) {
          correspondences[gridVoxelID].push_back(pathID);
        }
      } else {
        break;
      }
    }
  }

  fclose(filePtr);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localPlanner");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("pathFolder", pathFolder);
  nhPrivate.getParam("vehicleLength", vehicleLength);
  nhPrivate.getParam("vehicleWidth", vehicleWidth);
  nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
  nhPrivate.getParam("twoWayDrive", twoWayDrive);
  nhPrivate.getParam("laserVoxelSize", laserVoxelSize);
  nhPrivate.getParam("terrainVoxelSize", terrainVoxelSize);
  nhPrivate.getParam("useTerrainAnalysis", useTerrainAnalysis);
  nhPrivate.getParam("checkObstacle", checkObstacle);
  nhPrivate.getParam("adjacentRange", adjacentRange);
  nhPrivate.getParam("obstacleHeightThre", obstacleHeightThre);
  nhPrivate.getParam("groundHeightThre", groundHeightThre);
  nhPrivate.getParam("costHeightThre", costHeightThre);
  nhPrivate.getParam("costScore", costScore);
  nhPrivate.getParam("useCost", useCost);
  nhPrivate.getParam("pointPerPathThre", pointPerPathThre);
  nhPrivate.getParam("minRelZ", minRelZ);
  nhPrivate.getParam("maxRelZ", maxRelZ);
  nhPrivate.getParam("maxSpeed", maxSpeed);
  nhPrivate.getParam("dirWeight", dirWeight);
  nhPrivate.getParam("dirThre", dirThre);
  nhPrivate.getParam("dirToVehicle", dirToVehicle);
  nhPrivate.getParam("pathScale", pathScale);
  nhPrivate.getParam("minPathScale", minPathScale);
  nhPrivate.getParam("pathScaleStep", pathScaleStep);
  nhPrivate.getParam("pathScaleBySpeed", pathScaleBySpeed);
  nhPrivate.getParam("minPathRange", minPathRange);
  nhPrivate.getParam("pathRangeStep", pathRangeStep);
  nhPrivate.getParam("pathRangeBySpeed", pathRangeBySpeed);
  nhPrivate.getParam("pathCropByGoal", pathCropByGoal);
  nhPrivate.getParam("goalClearRange", goalClearRange);
  nhPrivate.getParam("goalX", goalX);
  nhPrivate.getParam("goalY", goalY);
  nhPrivate.getParam("useGlobalPlan", useGlobalPlan); 

  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>
                                ("/state_estimation", 5, odometryHandler);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>
                                  ("/registered_scan", 5, laserCloudHandler);

  ros::Subscriber subTerrainCloud = nh.subscribe<sensor_msgs::PointCloud2>
                                    ("/terrain_map", 5, terrainCloudHandler);
                                    
  ros::Subscriber subGoal;
  if (useGlobalPlan) {
    subGoal = nh.subscribe<visualization_msgs::MarkerArray>
              ("/rrt_planned_path", 5, globalPathHandler);
    ROS_INFO("Local planner running in GLOBAL PLAN mode");
  } else {
    subGoal = nh.subscribe<geometry_msgs::PoseStamped>
              ("/move_base_simple/goal", 5, simpleGoalHandler);
    ROS_INFO("Local planner running in SIMPLE GOAL mode");
  }

  ros::Publisher pubPath = nh.advertise<nav_msgs::Path> ("/path", 5);
  ros::Publisher pubGoalMarker = nh.advertise<visualization_msgs::Marker>("/goal_marker", 5);
  nav_msgs::Path path;

  printf ("\nReading path files.\n");

  joySpeed = maxSpeed / maxSpeed;
  if (joySpeed < 0) joySpeed = 0;
  else if (joySpeed > 1.0) joySpeed = 1.0;

  for (int i = 0; i < laserCloudStackNum; i++) {
    laserCloudStack[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  for (int i = 0; i < groupNum; i++) {
    startPaths[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
  for (int i = 0; i < gridVoxelNum; i++) {
    correspondences[i].resize(0);
  }

  laserDwzFilter.setLeafSize(laserVoxelSize, laserVoxelSize, laserVoxelSize);
  terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

  readStartPaths();
  readPathList();
  readCorrespondences();

  printf ("\nInitialization complete.\n\n");

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (useGlobalPlan && !globalPathPoints.empty()) {
      double distToGoal = sqrt(pow(vehicleX - goalX, 2) + pow(vehicleY - goalY, 2));
      if (distToGoal < goalUpdateDistance && currentGoalIndex < globalPathPoints.size() - 1) {
        currentGoalIndex++;
        if (currentGoalIndex >= globalPathPoints.size()) currentGoalIndex = globalPathPoints.size() - 1;
        goalX = globalPathPoints[currentGoalIndex].first;
        goalY = globalPathPoints[currentGoalIndex].second;
        ROS_INFO("Updating goal to point %d: (%.2f, %.2f)", currentGoalIndex, goalX, goalY);
      }
    }
    publishGoalMarker(pubGoalMarker, goalX, goalY);
    if (newLaserCloud || newTerrainCloud) {
      if (newLaserCloud) {
        newLaserCloud = false;

        laserCloudStack[laserCloudCount]->clear();
        *laserCloudStack[laserCloudCount] = *laserCloudDwz;
        laserCloudCount = (laserCloudCount + 1) % laserCloudStackNum;

        plannerCloud->clear();
        for (int i = 0; i < laserCloudStackNum; i++) {
          *plannerCloud += *laserCloudStack[i];
        }
      }

      if (newTerrainCloud) {
        newTerrainCloud = false;

        plannerCloud->clear();
        *plannerCloud = *terrainCloudDwz;
      }

      float sinVehicleRoll = sin(vehicleRoll);
      float cosVehicleRoll = cos(vehicleRoll);
      float sinVehiclePitch = sin(vehiclePitch);
      float cosVehiclePitch = cos(vehiclePitch);
      float sinVehicleYaw = sin(vehicleYaw);
      float cosVehicleYaw = cos(vehicleYaw);

      pcl::PointXYZI point;
      plannerCloudCrop->clear();
      int plannerCloudSize = plannerCloud->points.size();
      for (int i = 0; i < plannerCloudSize; i++) {
        float pointX1 = plannerCloud->points[i].x - vehicleX;
        float pointY1 = plannerCloud->points[i].y - vehicleY;
        float pointZ1 = plannerCloud->points[i].z - vehicleZ;

        point.x = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
        point.y = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
        point.z = pointZ1;
        point.intensity = plannerCloud->points[i].intensity;

        float dis = sqrt(point.x * point.x + point.y * point.y);
        if (dis < adjacentRange && ((point.z > minRelZ && point.z < maxRelZ) || useTerrainAnalysis)) {
          plannerCloudCrop->push_back(point);
        }
      }

      float pathRange = adjacentRange;
      if (pathRangeBySpeed) pathRange = adjacentRange * joySpeed;
      if (pathRange < minPathRange) pathRange = minPathRange;
      float relativeGoalDis = adjacentRange;

      if (true) {
        float relativeGoalX = ((goalX - vehicleX) * cosVehicleYaw + (goalY - vehicleY) * sinVehicleYaw);
        float relativeGoalY = (-(goalX - vehicleX) * sinVehicleYaw + (goalY - vehicleY) * cosVehicleYaw);

        relativeGoalDis = sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
        joyDir = atan2(relativeGoalY, relativeGoalX) * 180 / PI;

        if (!twoWayDrive) {
          if (joyDir > 90.0) joyDir = 90.0;
          else if (joyDir < -90.0) joyDir = -90.0;
        }
      }

      bool pathFound = false;
      float defPathScale = pathScale;
      if (pathScaleBySpeed) pathScale = defPathScale * joySpeed;
      if (pathScale < minPathScale) pathScale = minPathScale;

      while (pathScale >= minPathScale && pathRange >= minPathRange) {
        for (int i = 0; i < 36 * pathNum; i++) {
          clearPathList[i] = 0;
          pathPenaltyList[i] = 0;
        }
        for (int i = 0; i < 36 * groupNum; i++) {
          clearPathPerGroupScore[i] = 0;
        }

        float minObsAngCW = -180.0;
        float minObsAngCCW = 180.0;
        float diameter = sqrt(vehicleLength / 2.0 * vehicleLength / 2.0 + vehicleWidth / 2.0 * vehicleWidth / 2.0);
        float angOffset = atan2(vehicleWidth, vehicleLength) * 180.0 / PI;
        int plannerCloudCropSize = plannerCloudCrop->points.size();
        for (int i = 0; i < plannerCloudCropSize; i++) {
          float x = plannerCloudCrop->points[i].x / pathScale;
          float y = plannerCloudCrop->points[i].y / pathScale;
          float h = plannerCloudCrop->points[i].intensity;
          float dis = sqrt(x * x + y * y);

          if (dis < pathRange / pathScale && (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal) && checkObstacle) {
            for (int rotDir = 0; rotDir < 36; rotDir++) {
              float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
              float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
              if (angDiff > 180.0) {
                angDiff = 360.0 - angDiff;
              }
              if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                  ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
                continue;
              }

              float x2 = cos(rotAng) * x + sin(rotAng) * y;
              float y2 = -sin(rotAng) * x + cos(rotAng) * y;

              float scaleY = x2 / gridVoxelOffsetX + searchRadius / gridVoxelOffsetY 
                             * (gridVoxelOffsetX - x2) / gridVoxelOffsetX;

              int indX = int((gridVoxelOffsetX + gridVoxelSize / 2 - x2) / gridVoxelSize);
              int indY = int((gridVoxelOffsetY + gridVoxelSize / 2 - y2 / scaleY) / gridVoxelSize);
              if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 && indY < gridVoxelNumY) {
                int ind = gridVoxelNumY * indX + indY;
                int blockedPathByVoxelNum = correspondences[ind].size();
                for (int j = 0; j < blockedPathByVoxelNum; j++) {
                  if (h > obstacleHeightThre || !useTerrainAnalysis) {
                    clearPathList[pathNum * rotDir + correspondences[ind][j]]++;
                  } else {
                    if (pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] < h && h > groundHeightThre) {
                      pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] = h;
                    }
                  }
                }
              }
            }
          }

          if (dis < diameter / pathScale && (fabs(x) > vehicleLength / pathScale / 2.0 || fabs(y) > vehicleWidth / pathScale / 2.0) && 
              (h > obstacleHeightThre || !useTerrainAnalysis)) {
            float angObs = atan2(y, x) * 180.0 / PI;
            if (angObs > 0) {
              if (minObsAngCCW > angObs - angOffset) minObsAngCCW = angObs - angOffset;
              if (minObsAngCW < angObs + angOffset - 180.0) minObsAngCW = angObs + angOffset - 180.0;
            } else {
              if (minObsAngCW < angObs + angOffset) minObsAngCW = angObs + angOffset;
              if (minObsAngCCW > 180.0 + angObs - angOffset) minObsAngCCW = 180.0 + angObs - angOffset;
            }
          }
        }

        if (minObsAngCW > 0) minObsAngCW = 0;
        if (minObsAngCCW < 0) minObsAngCCW = 0;

        for (int i = 0; i < 36 * pathNum; i++) {
          int rotDir = int(i / pathNum);
          float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
          if (angDiff > 180.0) {
            angDiff = 360.0 - angDiff;
          }
          if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
              ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
            continue;
          }

          if (clearPathList[i] < pointPerPathThre) {
            float penaltyScore = 1.0 - pathPenaltyList[i] / costHeightThre;
            if (penaltyScore < costScore) penaltyScore = costScore;

            float dirDiff = fabs(joyDir - endDirPathList[i % pathNum] - (10.0 * rotDir - 180.0));
            if (dirDiff > 360.0) {
              dirDiff -= 360.0;
            }
            if (dirDiff > 180.0) {
              dirDiff = 360.0 - dirDiff;
            }

            float rotDirW;
            if (rotDir < 18) rotDirW = fabs(fabs(rotDir - 9) + 1);
            else rotDirW = fabs(fabs(rotDir - 27) + 1);
            float score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * rotDirW * rotDirW * rotDirW * rotDirW * penaltyScore;
            if (score > 0) {
              clearPathPerGroupScore[groupNum * rotDir + pathList[i % pathNum]] += score;
            }
          }
        }

        float maxScore = 0;
        int selectedGroupID = -1;
        for (int i = 0; i < 36 * groupNum; i++) {
          int rotDir = int(i / groupNum);
          float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
          float rotDeg = 10.0 * rotDir;
          if (rotDeg > 180.0) rotDeg -= 360.0;
          if (maxScore < clearPathPerGroupScore[i] && ((rotAng * 180.0 / PI > minObsAngCW && rotAng * 180.0 / PI < minObsAngCCW) || 
              (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive))) {
            maxScore = clearPathPerGroupScore[i];
            selectedGroupID = i;
          }
        }

        if (selectedGroupID >= 0) {
          int rotDir = int(selectedGroupID / groupNum);
          float rotAng = (10.0 * rotDir - 180.0) * PI / 180;

          selectedGroupID = selectedGroupID % groupNum;
          int selectedPathLength = startPaths[selectedGroupID]->points.size();
          path.poses.resize(selectedPathLength);
          for (int i = 0; i < selectedPathLength; i++) {
            float x = startPaths[selectedGroupID]->points[i].x;
            float y = startPaths[selectedGroupID]->points[i].y;
            float z = startPaths[selectedGroupID]->points[i].z;
            float dis = sqrt(x * x + y * y);

            if (dis <= pathRange / pathScale && dis <= relativeGoalDis / pathScale) {
              path.poses[i].pose.position.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
              path.poses[i].pose.position.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
              path.poses[i].pose.position.z = pathScale * z;
            } else {
              path.poses.resize(i);
              break;
            }
          }

          path.header.stamp = ros::Time().fromSec(odomTime);
          path.header.frame_id = "vehicle";
          pubPath.publish(path);
        }

        if (selectedGroupID < 0) {
          if (pathScale >= minPathScale + pathScaleStep) {
            pathScale -= pathScaleStep;
            pathRange = adjacentRange * pathScale / defPathScale;
          } else {
            pathRange -= pathRangeStep;
          }
        } else {
          pathFound = true;
          break;
        }
      }
      pathScale = defPathScale;

      if (!pathFound) {
        path.poses.resize(1);
        path.poses[0].pose.position.x = 0;
        path.poses[0].pose.position.y = 0;
        path.poses[0].pose.position.z = 0;

        path.header.stamp = ros::Time().fromSec(odomTime);
        path.header.frame_id = "vehicle";
        pubPath.publish(path);
      }
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}