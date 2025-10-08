/*
*	File: rrt_star_interactive_node.cpp
*	---------------
*   Created by Rjl on 2025.10
*/
#include <ros/ros.h>
#include <global_path_planner/rrtStarOctomap.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h> 
#include <thread>
#include <mutex>

using std::cout;
using std::endl;

bool hasOdom = false;
std::vector<double> currentPosition {0, 0, 0};
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    currentPosition[0] = odom->pose.pose.position.x;
    currentPosition[1] = odom->pose.pose.position.y;
    currentPosition[2] = odom->pose.pose.position.z;
    hasOdom = true;
}

bool newGoalMsg = false;
std::vector<double> goalPoint {0, 0, 1.0};
void clickedPointCB(const geometry_msgs::PoseStamped::ConstPtr& cp){
    goalPoint[0] = cp->pose.position.x;
    goalPoint[1] = cp->pose.position.y;
    goalPoint[2] = 1.0; // set height to be 1.0 m
    newGoalMsg = true;
}

nav_msgs::Path global_path;
ros::Publisher goalVisPub;
ros::Publisher globalpathPub; 
bool initGoal = false;
visualization_msgs::Marker goalMarker;

void publishGoalVis(){
    ros::Rate r(10);
    while (ros::ok()){
        if (initGoal){
            goalVisPub.publish(goalMarker);
        }
        r.sleep();
    }
}   

int main(int argc, char** argv){
    ros::init(argc, argv, "RRT*_test_node");
    ros::NodeHandle nh;

    ros::Subscriber odomSub = nh.subscribe("/state_estimation", 1000, odomCallback);
    ros::Subscriber clickedPointSub = nh.subscribe("/move_base_simple/goal", 1000, clickedPointCB);
    
    ros::Publisher posePub = nh.advertise<geometry_msgs::PoseStamped>("/trajectory_pose", 1000);
    goalVisPub = nh.advertise<visualization_msgs::Marker>("/goal_position", 1000);
    globalpathPub = nh.advertise<nav_msgs::Path>("/rrt_star/global_path", 10);
    std::thread goalVisWorker_ = std::thread(publishGoalVis);
    
    const int N = 3; // dimension
    globalPlanner::rrtStarOctomap<N> rrtStarPlanner (nh);
    cout << rrtStarPlanner << endl;

    int countLoop = 0;
    ros::Rate r(10);
    while (ros::ok()){
        cout << "----------------------------------------------------" << endl;
        cout << "[Planner Node]: Request No. " << countLoop+1 << endl;
        cout << "[Planner Node]: Waiting for goal point..." << endl;
        while (ros::ok()){
            if (newGoalMsg){
                std::vector<double> goal = goalPoint;
                rrtStarPlanner.updateGoal(goal);
                global_path.poses.clear();
                newGoalMsg = false;
                cout << "[Planner Node]: Goal point set. (" << goal[0] << " " << goal[1] << " " << goal[2] << ")" << endl;
                
                initGoal = true;
                goalMarker.header.frame_id = "map";
                goalMarker.header.stamp = ros::Time();
                goalMarker.ns = "goal_vis";
                goalMarker.id = 0;
                goalMarker.type = visualization_msgs::Marker::SPHERE;
                goalMarker.action = visualization_msgs::Marker::ADD;
                goalMarker.pose.position.x = goal[0];
                goalMarker.pose.position.y = goal[1];
                goalMarker.pose.position.z = goal[2];
                goalMarker.lifetime = ros::Duration(0.5);
                goalMarker.scale.x = 0.4;
                goalMarker.scale.y = 0.4;
                goalMarker.scale.z = 0.4;
                goalMarker.color.a = 0.7;
                goalMarker.color.r = 0.2;
                goalMarker.color.g = 1.0;
                goalMarker.color.b = 0.2;
                break;
            }
            ros::spinOnce();
            r.sleep();
        }
        
        cout << "[Planner Node]: Waiting for current position from odometry..." << endl;
        while (ros::ok() && !hasOdom) {
            ros::spinOnce();
            r.sleep();
        }
        std::vector<double> start = currentPosition;
        rrtStarPlanner.updateStart(start);
        cout << "[Planner Node]: Start point from odometry. (" 
             << start[0] << " " << start[1] << " " << start[2] << ")" << endl;

        rrtStarPlanner.makePlan(global_path);
	
        if (!global_path.poses.empty()) {
            cout << "[Planner Node]: Publishing path with " << global_path.poses.size() << " waypoints." << endl;
			global_path.header.frame_id = "map"; 
            global_path.header.stamp = ros::Time::now();
            globalpathPub.publish(global_path);
        } else {
            cout << "[Planner Node]: Generated path is empty!" << endl;
        }
	

        ++countLoop;
        cout << "----------------------------------------------------" << endl;
    }
    goalVisWorker_.join();
    return 0;
}