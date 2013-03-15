#include "src/PathPlanner.h"
#include <Eigen/Dense>

#include <stdio.h>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include "rrt/Path_rrt.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

#define SCAN_DATA_NUM	720

static CURB_Vector_3D robotPosition;
static double robotAngle;// degree
static vector<float> scanData(SCAN_DATA_NUM);
static int scanCnt = 0;

void SendPath(ros::Publisher &cmd_path_pub_, MatrixXd pathSegX, MatrixXd pathSegY, MatrixXd pathSegZ, CURB_Vector_3D rState, double rAngle, double psi)
{
	rrt::Path_rrt path;
	vector<double> vTemp;
	
	vTemp.reserve(pathSegX.rows()*pathSegX.cols());
	for (int i=0; i<pathSegX.rows(); i++)
		for (int j=0; j<pathSegX.cols(); j++)
			vTemp.push_back(pathSegX(i,j));

	path.path_X = vTemp;
	vTemp.clear();


	vTemp.reserve(pathSegY.rows()*pathSegY.cols());
	for (int i=0; i<pathSegY.rows(); i++)
		for (int j=0; j<pathSegY.cols(); j++)
			vTemp.push_back(pathSegY(i,j));
	
	path.path_Y = vTemp;
	vTemp.clear();


	vTemp.reserve(pathSegZ.rows()*pathSegZ.cols());
	for (int i=0; i<pathSegZ.rows(); i++)
		for (int j=0; j<pathSegZ.cols(); j++)	
			vTemp.push_back(pathSegZ(i,j));
	
	path.path_Z = vTemp;
	vTemp.clear();

	vTemp.reserve(2);
	vTemp.push_back(rState.x);  vTemp.push_back(rState.y);
	path.start_state = vTemp;
	
	path.hdg = rAngle;
	path.psi = psi;
	
	// log
	FILE *fOut;
	fOut = fopen("start_state.txt","wt");
	fprintf(fOut,"%f\t%f\t%f\t",rState.x, rState.y, rAngle);
	fclose(fOut);
	
	cmd_path_pub_.publish(path);
}

void StateCallback(const nav_msgs::Odometry& msg)
{
	// update global robot position
	robotPosition.x = msg.pose.pose.position.x;
	robotPosition.y = msg.pose.pose.position.y;
	robotPosition.z = 0;

	geometry_msgs::Quaternion odom_quat = msg.pose.pose.orientation;
	robotAngle = tf::getYaw(odom_quat)*180.0/M_PI;

	ROS_INFO("Move robot : [%f, %f, %f]", robotPosition.x, robotPosition.y, robotAngle);
}

void ScanCallback(const sensor_msgs::LaserScan& msg)
{
	FILE *scanTest;
	char ptr[1024];
	
	sprintf(ptr, "data/odo/odo%d.txt",scanCnt);
	scanTest = fopen(ptr,"wt");
	fprintf(scanTest,"%f\t%f\t%f\t",robotPosition.x, robotPosition.y, robotAngle);
	fclose(scanTest);
	
	sprintf(ptr, "data/scan/scan%d.txt",scanCnt++);
	scanTest = fopen(ptr,"wt");
	for (int i=0; i < SCAN_DATA_NUM; i++) {
		scanData[i] = msg.ranges[i];
		fprintf(scanTest,"%lf\n",msg.ranges[i]);
	}
	fclose(scanTest);
}

int main(int argc, char **argv)
{
	// init the ROS node
	ros::init(argc, argv, "robot_pathplanner");
	ros::NodeHandle nh_;
	ros::Publisher cmd_path_pub_ = nh_.advertise<rrt::Path_rrt>("/rrt/path", 100);
	ros::Subscriber cmd_state_sub = nh_.subscribe( "/pose", 0, StateCallback);
	ros::Subscriber cmd_scan_sub = nh_.subscribe( "/scan", 0, ScanCallback);
	
	usleep(500000);

	// init the path planner
	CPathPlanner PathPlanner;

	// environment variable
	int iNumObstacles = 0;
	double stAngle = 90.0;
	double loop_time = 0.25;
	CURB_Vector_3D startNode = CURB_Vector_3D(0.0,0.0,0.0);
	CURB_Vector_3D endNode = CURB_Vector_3D(0.0,3.0,0.0);
	
	// set initial (global) robot position
	robotPosition = startNode;
	robotAngle = stAngle;
	
	// condition variable
	setting_t P;
	P.L = .03;		// RRT tree extension length
	P.SZR = .4;		// Safety Zone of Radius
	P.SZH = .03;	// Safety Zone of Height
	P.d = .02;		// smoothing Length of Cubic Bezier Spiral
	P.ds = .01;		// smoothing Length of Simple Cubic Bezier Curve
	P.kappa = 3.5;	// maximum curvature of the path
	P.INC = .04;	// Increment of d
	P.Ind = .01; 	// Indicator
	PathPlanner.SetCondition(P);
	
	// World setting
	worldLine_t world = CreateWorld_Line(iNumObstacles);
	PathPlanner.SetWorld(world);
		
	// Sensor & line extractor : hokuyo - sensor rotation : yaw pitch roll (radian)
	CLRFModel LRF = CLRFModel(CURB_Point_3D(0,0.17,0.840), CURB_Point_3D(0,-20*M_PI/180,0),
	                60.0f, 0.024, deg2rad(180), 0.00436332309619);
	CVirtualLRF vLRF = CVirtualLRF(LRF,0.00f);
	PathPlanner.SetvLRF(vLRF);
	PathPlanner.SetLRF(LRF);
	PathPlanner.SetExtractor(.005f); // merge error threshold

	// db update
	PathPlanner.UpdateExtractorDbMap();

	// Path planning
	PathPlanner.SetStartNode(startNode);
	PathPlanner.SetEndNode(endNode);
	PathPlanner.SetRobotState(startNode, stAngle);
	
	//ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan",nh_); // 일단 중지
	ros::spinOnce();
	PathPlanner.UpdateLRF_ros(scanData);
	PathPlanner.UpdateKnownWorld();
	PathPlanner.UpdateExtractorDbMap();

	ROS_INFO("Generate Path");
	PathPlanner.GenPath();
	ROS_INFO("First path generation is completed");

	PathPlanner.StackCurve();
	PathPlanner.SaveData();
//	SendPath(cmd_path_pub_,PathPlanner.GetPathX(),PathPlanner.GetPathY(),PathPlanner.GetPathZ(),PathPlanner.GetRobotPosition(),PathPlanner.GetRobotAngle(),PathPlanner.GetPsi()); // 일단 중지
	
	ros::Rate loop_rate(1.0/loop_time);
	while(nh_.ok()) {
		//ros::topic::waitForMessage<nav_msgs::Odometry>("/pose",nh_);
		PathPlanner.SetRobotState(robotPosition, robotAngle);
		ros::spinOnce();
		
		//ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan",nh_);
		ros::spinOnce();
		PathPlanner.UpdateLRF_ros(scanData);
		PathPlanner.UpdateKnownWorld();
		PathPlanner.UpdateExtractorDbMap();
				
		// if generated path collide with updated world
//		if(PathPlanner.CollisionCheck()) {
			PathPlanner.StackCurve();
			
			// Generate path from known world
			PathPlanner.SetStartNode(PathPlanner.GetRobotPosition());
			
			ROS_INFO("Generate Path");
			PathPlanner.GenPath();
			PathPlanner.SaveData();
			
			//SendPath(cmd_path_pub_,PathPlanner.GetPathX(),PathPlanner.GetPathY(),PathPlanner.GetPathZ(),PathPlanner.GetRobotPosition(),PathPlanner.GetRobotAngle(),PathPlanner.GetPsi());
//		}
				
		if(PathPlanner.IsArrived()) 
			break;
	}
	
	PathPlanner.StackCurve();
	PathPlanner.SaveData();
	
	ROS_INFO("End of program");
	
	return 0;
}

