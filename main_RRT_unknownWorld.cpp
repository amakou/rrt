#include "src/PathPlanner.h"
//#include "Eigen/Dense"
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

static CURB_Vector_3D robotPosition;
static double robotAngle;// degree
static vector<float> scanData;


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
	vTemp.clear();
	
	path.hdg = rAngle;
	path.psi = psi;
	
	FILE *fOut;
	fOut = fopen("start_state.txt","wt");
	fprintf(fOut,"%f\t",rState.x);   fprintf(fOut,"%f\t",rState.y);   fprintf(fOut,"%f\t",rAngle);  
	fclose(fOut);
	
	cmd_path_pub_.publish(path);
}


void SimStateCallback(const geometry_msgs::PoseConstPtr& msg)
{
	robotPosition.x = msg->position.x;
	robotPosition.y = msg->position.y;
	robotPosition.z = 0;
	
	robotAngle = msg->orientation.z*180.0/M_PI;
	
//	ROS_INFO("Move robot : [%f, %f, %f]", robotPosition.x, robotPosition.y, robotAngle*180.0/M_PI);
}


void StateCallback(const nav_msgs::Odometry& msg)
{
	geometry_msgs::Quaternion odom_quat;
	double qx, qy, qz, qw;
		
	robotPosition.x = msg.pose.pose.position.x;
	robotPosition.y = msg.pose.pose.position.y;
	robotPosition.z = 0;
	
	odom_quat = msg.pose.pose.orientation;
	qx = odom_quat.x;
	qy = odom_quat.y;
	qz = odom_quat.z;
	qw = odom_quat.w;

	robotAngle = tf::getYaw(odom_quat)*180.0/M_PI;
	
//	ROS_INFO("Move robot : [%f, %f, %f]", robotPosition.x, robotPosition.y, robotAngle);
}


static int cnt = 0;

void ScanCallback(const sensor_msgs::LaserScan& msg)
{
//	int scanNum = 512;
	int scanNum = 720;

	scanData.resize(scanNum);
	
//	ROS_INFO("<<< Receive Scan");
	
	FILE *scanTest;


	char ptr[1024];
	
	sprintf(ptr, "data/odo/odo%d.txt",cnt);
	scanTest = fopen(ptr,"wt");
	fprintf(scanTest,"%f\t%f\t%f\t",robotPosition.x, robotPosition.y, robotAngle);
	fclose(scanTest);

	
	sprintf(ptr, "data/scan/scan%d.txt",cnt);
	cnt++;
	scanTest = fopen(ptr,"wt");
	
	for (int i=0; i<scanNum; i++)
	{
		scanData[i] = msg.ranges[i];
		fprintf(scanTest,"%lf\n",msg.ranges[i]);
	}
	
	fclose(scanTest);
}


int main(int argc, char **argv)
{
	/// ***** ROS SETUP ***** ///
	
	 //init the ROS node 
	ros::init(argc, argv, "robot_pathplanner");
	ros::NodeHandle nh_;
	
	ros::Publisher cmd_path_pub_;
	ros::Subscriber cmd_state_sub, cmd_scan_sub;
	
	cmd_path_pub_ = nh_.advertise<rrt::Path_rrt>("/rrt/path", 100);
	cmd_state_sub = nh_.subscribe( "/pose", 0, StateCallback);
	cmd_scan_sub = nh_.subscribe( "/scan", 0, ScanCallback);
	
	usleep(500000);

	/// ***** PATH PLANNER ***** ///
	CPathPlanner PathPlanner;

	/// Initial setting
	CURB_Vector_3D startNode, endNode;
	int iNumObstacles;
	double stAngle, loop_time;
	
	iNumObstacles = 0;
	
	startNode = CURB_Vector_3D(0.0,0.0,0.0);
	endNode = CURB_Vector_3D(0.0,3.0,0.0);
	stAngle = 90.0;
	
	robotPosition = startNode;
	robotAngle = stAngle;
	
	// Condition Setting
	setting_t P;
	
	P.L = .03;	// RRT tree extension length
//	P.SZR = .37;	// Safety Zone of Radius
	P.SZR = .4;	// Safety Zone of Radius
	P.SZH = .03;	// Safety Zone of Height
	P.d = .02;	// smoothing Length of Cubic Bezier Spiral
	P.ds = .01;	// smoothing Length of Simple Cubic Bezier Curve
	P.kappa = 3.5;	// maximum curvature of the path
	P.INC = .04;	// Increment of d
	P.Ind = .01; 	// Indicator
	
	PathPlanner.SetCondition(P);
	
	/// World setting
	worldLine_t world;
	
	world = CreateWorld_Line(iNumObstacles);
	PathPlanner.SetWorld(world);
		
	/// Sensor & line extractor
	CVirtualLRF vLRF;
	CLRFModel LRF;
	

	// hokuyo - sensor rotation : yaw pitch roll (radian)
	LRF = CLRFModel(CURB_Point_3D(0,0.17,0.840),CURB_Point_3D(0,-20*M_PI/180,0),60.0f, 0.024,deg2rad(180),0.00436332309619);  
	vLRF = CVirtualLRF(LRF,0.00f);
			
	PathPlanner.SetvLRF(vLRF);
	PathPlanner.SetLRF(LRF);
	PathPlanner.SetExtractor(.005f); // merge error threshold

	// db update
	PathPlanner.UpdateExtractorDbMap();


	/// Path planning
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

	// 일단 중지
//	SendPath(cmd_path_pub_,PathPlanner.GetPathX(),PathPlanner.GetPathY(),PathPlanner.GetPathZ(),PathPlanner.GetRobotPosition(),PathPlanner.GetRobotAngle(),PathPlanner.GetPsi());
	
	loop_time = 0.25;
	ros::Rate loop_rate(1.0/loop_time);

	while(nh_.ok())
	{
		// Get lrf information and update known world
		//ros::topic::waitForMessage<nav_msgs::Odometry>("/pose",nh_);
		
		PathPlanner.SetRobotState(robotPosition, robotAngle);
		ros::spinOnce();
		
		//ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan",nh_);
		ros::spinOnce();
		PathPlanner.UpdateLRF_ros(scanData);
		PathPlanner.UpdateKnownWorld();
		PathPlanner.UpdateExtractorDbMap();
				
		// if generated path collide with updated world
		if(PathPlanner.CollisionCheck())
		{
			PathPlanner.StackCurve();
			
			// Generate path from known world
			PathPlanner.SetStartNode(PathPlanner.GetRobotPosition());
			
			ROS_INFO("Generate Path");
			PathPlanner.GenPath();
			PathPlanner.SaveData();
	
			PathPlanner.GetPathX();
			
			//SendPath(cmd_path_pub_,PathPlanner.GetPathX(),PathPlanner.GetPathY(),PathPlanner.GetPathZ(),PathPlanner.GetRobotPosition(),PathPlanner.GetRobotAngle(),PathPlanner.GetPsi());
		}
				
		if(PathPlanner.IsArrived()) 
			break;
	}
	
	PathPlanner.StackCurve();
	PathPlanner.SaveData();
	
	ROS_INFO("End of program");
	
	return 0;
}

