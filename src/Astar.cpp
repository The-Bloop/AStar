#include <ros/ros.h>

// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

// Subscribe to a topic with this message type
#include <sensor_msgs/LaserScan.h>
//#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>

// for printing
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <bits/stdc++.h>

// for RAND_MAX
#include <cstdlib>

std::vector<std::vector<int>> Map;
double end_x, end_y;

double start_x = 0;
double start_y = 0;
int start_x_index = 9;
int start_y_index = 9;

double stop_x = 5;
double stop_y = 5;
int stop_x_index = 0;
int stop_y_index = 0;
bool path_established = false;


class Position
{
    public:
    Position* Next;
    int direction;
    double cost;
    int x_index;
    int y_index;
    int fill_value;

    Position()
    {
		cost = std::numeric_limits<double>::max();
		x_index = 0;
		y_index = 0;
		direction = -1;
    }

    ~Position()
    {

    }
};

class PathTree
{
    public:
    Position* Start;

    PathTree()
    {
        std::cout<<"Creating DS"<<std::endl;
        Start = new Position();
        Start->x_index = start_x_index;
        Start->y_index = start_y_index;
        double minval = std::numeric_limits<double>::max();
    }

    ~PathTree()
    {

    }

    void Find_Short_Path()
    {
        double val = Find_Short_Path(*Start,0);
        std::cout<<"Final Score: "<<val<<std::endl;
    }

    double Find_Short_Path(Position & pos, int level)
    {
        double maxval = std::numeric_limits<double>::max();

        if(level > 100)
            return maxval;
        
        double min_score = maxval;
        double min_dir = -1;
        
        int fill_value = Map[pos.x_index][pos.y_index];

        std::cout<<pos.x_index<<", "<<pos.y_index<<", "<<fill_value<<std::endl;

        if((pos.x_index != stop_x_index || pos.y_index != stop_y_index) && fill_value == 0)
        {
        	std::vector<double> scores = {maxval,maxval,maxval,maxval,maxval,maxval,maxval,maxval};
        	std::vector<bool> boundary_bool = CheckBoundaries(pos);
        	pos.Next = new Position();
        	for(int i=0;i<8;i++)
        	{
        		if(boundary_bool[i]!=true)
        		{
        			Position* next = new Position();
        			next->x_index = pos.x_index;
        			next->y_index = pos.y_index;
        			Get_Index(next->x_index, next->y_index, i);
        			scores[i] = Calculate_Cost(next->x_index,next->y_index);
        		}
        	}
        	
        	std::vector<int> direction = Sort_Index(scores);
        	std:sort(scores.begin(), scores.end());

        	for(int i=0;i<8;i++)
        	{
        		if(scores[i] != maxval)
        		{
        			Position* next = new Position();
        			next->x_index = pos.x_index;
        			next->y_index = pos.y_index;
        			Get_Index(next->x_index, next->y_index, direction[i]);
        			double next_score = Find_Short_Path(*next, level+1);
        			double new_score = next_score + scores[i];
        			if(new_score < min_score)
        			{
        				min_score = new_score;
        				min_dir = direction[i];
        				pos.Next = next;
        				if(path_established == true)
        				{
        				    break;
        				}
        			}
        		}
        	}
        	
        	pos.direction = min_dir;
        	return min_score;
        }
        else if(pos.x_index == stop_x_index && pos.y_index == stop_y_index)
        {
            path_established = true;
        	double score = Calculate_Cost(pos.x_index, pos.y_index);
        	return score;
        }
        else
        {
        	return maxval;
        }
    }
    
    std::vector<int> Sort_Index(std::vector<double> scores)
    {
    	std::vector<double> dupe_scores = scores;
    	std::sort(dupe_scores.begin(), dupe_scores.end());
    	std::vector<int> index = {0,1,2,3,4,5,6,7};
    	double maxval = std::numeric_limits<double>::max();
    	for(int i=0;i<8;i++)
    	{
    		for(int j=0;j<8;j++)
    		{
    			if(dupe_scores[j] == maxval)
    			{
    				index[j] = -1;
    			}
    			else if(dupe_scores[j] == scores[i])
    			{
    				index[j] = i;
    				dupe_scores[j] = 0;
    				break;
    			}
    		}
    	}
    	return index;
    	
    }
    
    void Get_Index(int &x, int &y, int move)
    {
    	if(move == 0)
    	{
    		x = x - 1;
    	}
    	else if(move == 1)
    	{
    		x = x - 1;
    		y = y - 1;
    	}
    	else if(move == 2)
    	{
    		y = y - 1;
    	}
    	else if(move == 3)
    	{
    		x = x + 1;
    		y = y - 1;
    	}
    	else if(move == 4)
    	{
    		x = x + 1;
    	}
    	else if(move == 5)
    	{
    		x = x + 1;
    		y = y + 1;
    	}
    	else if(move == 6)
    	{
    		y = y + 1;
    	}
    	else if(move ==7)
    	{
    		x = x - 1;
    		y = y + 1;
    	}
    }

    double Calculate_Cost(int x_index, int y_index)
    {
        if(Map[x_index][y_index] == 0)
        {
            double hx = abs(x_index-stop_x_index);
            double hy = abs(y_index-stop_y_index);
            double h = (hx+hy);// + (1.414 -2) * std::min(hx,hy);
            return h;
        }
        else
            return std::numeric_limits<double>::max();
    }
    
    std::vector<bool> CheckBoundaries(Position &pos)
    {
    	std::vector<bool> boundary_bool = {0,0,0,0,0,0,0,0};
    	
    	if(pos.x_index == 0)
    	{
    		boundary_bool[0] = 1;
    		boundary_bool[1] = 1;
    		boundary_bool[7] = 1;
    	}
    	if(pos.y_index == 0)
    	{
    		boundary_bool[1] = 1;
    		boundary_bool[2] = 1;
    		boundary_bool[3] = 1;
    	}
    	if(pos.x_index == 17)
    	{
    		boundary_bool[3] = 1;
    		boundary_bool[4] = 1;
    		boundary_bool[5] = 1;
    	}
    	if(pos.x_index == 19)
    	{
    		boundary_bool[5] = 1;
    		boundary_bool[6] = 1;
    		boundary_bool[7] = 1;
    	}
    	
    	return boundary_bool;
    }

    void printpos(Position & pos)
    {
        std::cout<<pos.x_index<<", "<<pos.y_index;
    }
    
    void Print_Path()
    {
    	std::cout<<Start->direction<<std::endl;
    	Position* next = Start;
    	while(next->Next != nullptr)
    	{
    	    std::cout<<"Pose: ";
    	    printpos(*next);
    		std::cout<<"; Direction :"<<next->direction<<std::endl;
    		next = next->Next;
    	}
    }

    void Move()
    {
        std::cout<<Start->direction<<std::endl;
    	Position* next = Start;
    	while(next->Next != nullptr)
    	{
    	    std::cout<<"Pose: ";
    	    printpos(*next);
    		std::cout<<"; Direction :"<<next->direction<<std::endl;
    		next = next->Next;
    	}
    }

};

class AStar
{
    private:
    // A ROS node
    ros::NodeHandle n;

    // Listen for scan messages
    //ros::Subscriber scan_sub;

    // Publish position data
    ros::Subscriber pose_sub;
    double x = 0;

    //ros::Publisher drive_pub;

    double bot_angle;
    geometry_msgs::Point bot_pose;
    double old_bot_angle;
    geometry_msgs::Point old_bot_pose;
    ros::Publisher drive_pub;


    public:

    AStar()
    {
        n = ros::NodeHandle("~");
        std::string drive_topic;
        std::string pose_topic;
        n.getParam("odom_topic",drive_topic);
        n.getParam("pose_topic",pose_topic);
        n.getParam("start_pose_x",start_x);
        n.getParam("start_pose_y",start_y);
        n.getParam("end_pose_x",stop_x);
        n.getParam("end_pose_y",stop_y);

        old_bot_angle = 0;
        old_bot_pose.x = 0;
        old_bot_pose.y = 0;
        old_bot_pose.z = 0;

        GetMapIndex(stop_x,stop_y,stop_x_index,stop_y_index);
        GetMapIndex(start_x,start_y,start_x_index,start_y_index);

        pose_sub = n.subscribe(pose_topic, 1, &AStar::PoseChange_CallBack, this);

        PathTree path_tree;
        drive_pub = n.advertise<nav_msgs::Odometry>(drive_topic, 10);

        if(Map[start_x_index][start_y_index] != 1 && Map[stop_x_index][stop_y_index] != 1)
        {
            std::cout<<"Start : "<<start_x_index<<", "<<start_y_index<<std::endl;
            std::cout<<"End : "<<stop_x_index<<", "<<stop_y_index<<std::endl;


            if(path_established)
                std::cout<<"Goal Reached"<<std::endl;

            path_tree.Find_Short_Path();
//            path_tree.Print_Path();
        }
        else
            std::cout<<"Set Better Pose"<<std::endl;
    }

    void Calculate_Cost(int x_index, int y_index, double & g, double & h)
    {
        g = sqrt(pow((x_index,start_x_index),2) + pow((y_index,start_y_index),2));
        h = sqrt(pow((x_index,stop_x_index),2) + pow((y_index,stop_y_index),2));
    }

    void PoseChange_CallBack(const nav_msgs::Odometry & msg)
    {
        double q3 = msg.pose.pose.orientation.z;
        double q0 = msg.pose.pose.orientation.w;
        double angle = atan2((2*(q3*q0)), (1 - (2 * (q3*q3))));
        if(CheckPoseChange(msg, angle))
        {
            std::cout<<"Blah!!!"<<std::endl;
            bot_angle = angle;
            bot_pose.x = msg.pose.pose.position.x;
            bot_pose.y = msg.pose.pose.position.y;
            bot_pose.z = msg.pose.pose.position.z;
        }
    }

    bool CheckPoseChange(const nav_msgs::Odometry & msg, double angle)
    {
        if (msg.pose.pose.position.x==old_bot_pose.x && msg.pose.pose.position.y==old_bot_pose.y && msg.pose.pose.position.z==old_bot_pose.z && angle == old_bot_angle)
            return false;
        else
        {
            old_bot_pose.x = msg.pose.pose.position.x;
            old_bot_pose.y = msg.pose.pose.position.y;
            old_bot_pose.z = msg.pose.pose.position.z;
            old_bot_angle = angle;
            return true;
        }
    }
    
    bool CheckMouseMove(const nav_msgs::Odometry & msg, double angle)
    {
        return false;
    }

    void GetMapIndex(double x, double y, int & x_index, int & y_index)
    {
        x_index = int(9-x);
        y_index = int(8+y);

        if(x_index > 19)
        {
            x_index = 19;
        }
        if(x_index < 0)
        {
            x_index = 0;
        }
        if(y_index > 17)
        {
            y_index = 17;
        }
        if(y_index < 0)
        {
            y_index = 0;
        }
    }
};

void CreateDS()
{

}


int main(int argc, char ** argv) 
{
    ros::init(argc, argv, "pathplan");

    Map = {{0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0},
           {0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0},
           {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
           {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
           {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
           {0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0},
           {0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0},
           {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0},
           {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1},
           {0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1},
           {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1},
           {0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0},
           {0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0},
           {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
           {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
           {0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
           {0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0},
           {0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0},
           {0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0},
           {0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1}};

    AStar e;
    ros::spin();
    return 0;
}
