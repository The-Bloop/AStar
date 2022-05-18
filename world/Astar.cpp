#include <ros/ros.h>

// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

// Subscribe to a topic with this message type
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>

// for printing
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <bits/stdc++.h>

// for RAND_MAX
#include <cstdlib>

using namespace std;

std::vector<std::vector<int>> Map;
double end_x, end_y;

double start_x = 0;
double start_y = 0;
int start_x_index = 9;
int start_y_index = 9;

double stop_x = 0;
double stop_y = 0;
int stop_x_index = 0;
int stop_y_index = 9;


class Position
{
    public:
    Position* Left;
    Position* Right;
    Position* Top;
    Position* Bottom;
    Position* TopLeft;
    Position* BottomLeft;
    Position* TopRight;
    Position* BottomRight;
    double cost;
    int x_index;
    int y_index;
    int fill_value;

    Position()
    {

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
        std::cout<<Start->x_index<<", "<<Start->y_index<<std::endl;

        double minval = std::numeric_limits<double>::max();
        int dir = -1;

        /*double sum = Calculate_Cost(Start->x_index,Start->y_index);

        std::cout<<sum<<std::endl;*/

        double val = Find_Short_Path(*Start, -1);

        std::cout<<val<<" "<<dir<<std::endl;
    }

    void printpos(Position & pos)
    {
        std::cout<<pos.x_index<<", "<<pos.y_index<<std::endl;
    }

    ~PathTree()
    {

    }

    double Find_Short_Path(Position & pos, int dir)
    {
        double maxval = std::numeric_limits<double>::max();
        //std::cout<<pos.x_index<<", "<<pos.y_index<<std::endl;

        if((pos.x_index != stop_x_index || pos.y_index != stop_y_index))
        {
            if( Map[pos.y_index][pos.x_index] == 0)
            {
                //if(dir == -1)
                    //std::cout<<pos.x_index<<", "<<pos.y_index<<", "<<dir<<std::endl;

                std::vector<double> path_costs = {maxval,maxval,maxval,maxval,maxval,maxval,maxval,maxval};
                std::vector<int> dir_vector = {0,1,2,3,4,5,6,7};

                Position * Left = new Position();
                Position * Right = new Position();
                Position * Top = new Position();
                Position * Bottom = new Position();
                Position * TopLeft = new Position();
                Position * BottomLeft = new Position();
                Position * TopRight = new Position();
                Position * BottomRight = new Position();

                if(pos.x_index > 0)
                {
                    if(dir != 1)
                    {
                        Left->x_index = pos.x_index - 1;
                        Left->y_index = pos.y_index;
                        double value = Calculate_Cost(Left->x_index,Left->y_index);
                        //std::cout<<Left->x_index<<", "<<Left->y_index<<std::endl;
                        path_costs[0] = value;

                    }
                    else
                    {
                        dir_vector[0] = -1;
                    }
                    if(pos.y_index > 0)
                    {
                        if(dir != 5)
                        {
                            TopRight->x_index = pos.x_index - 1;
                            TopRight->y_index = pos.y_index - 1;
                            double value = Calculate_Cost(TopRight->x_index,TopRight->y_index);
                            //std::cout<<TopLeft->x_index<<", "<<TopLeft->y_index<<std::endl;
                            path_costs[6] = value;
                        }
                        else
                        {
                            dir_vector[6] = -1;
                        }

                    }

                    if(pos.y_index < 17)
                    {
                        if(dir != 7)
                        {
                            TopLeft->x_index = pos.x_index - 1;
                            TopLeft->y_index = pos.y_index + 1;
                            double value = Calculate_Cost(TopLeft->x_index,TopLeft->y_index);
                            //std::cout<<BottomLeft->x_index<<", "<<BottomLeft->y_index<<std::endl;
                            path_costs[4] = value;
                        }
                        else
                        {
                            dir_vector[4] = -1;
                        }
                    }
                }

                if(pos.x_index < 17)
                {
                    if(dir != 0)
                    {
                        Right->x_index = pos.x_index + 1;
                        Right->y_index = pos.y_index;
                        double value = Calculate_Cost(Right->x_index,Right->y_index);
                        //std::cout<<Right->x_index<<", "<<Right->y_index<<std::endl;
                        path_costs[1] = value;

                    }
                    else
                    {
                        dir_vector[1] = -1;
                    }
                    if(pos.y_index > 0)
                    {
                        if(dir != 4)
                        {
                            BottomRight->x_index = pos.x_index + 1;
                            BottomRight->y_index = pos.y_index - 1;
                            double value = Calculate_Cost(BottomRight->x_index,BottomRight->y_index);
                            //std::cout<<TopRight->x_index<<", "<<TopRight->y_index<<std::endl;
                            path_costs[7] = value;
                        }
                        else
                        {
                            dir_vector[7] = -1;
                        }

                    }
                    if(pos.y_index < 17)
                    {
                        if(dir != 6)
                        {
                            BottomLeft->x_index = pos.x_index + 1;
                            BottomLeft->y_index = pos.y_index + 1;
                            double value = Calculate_Cost(BottomLeft->x_index,BottomLeft->y_index);
                            //std::cout<<BottomRight->x_index<<", "<<BottomRight->y_index<<std::endl;
                            path_costs[5] = value;
                        }
                        else
                        {
                            dir_vector[5] = -1;
                        }

                    }
                }

                if(pos.y_index > 0)
                {
                    if(dir != 3)
                    {
                        Top->x_index = pos.x_index;
                        Top->y_index = pos.y_index - 1;
                        double value = Calculate_Cost(Top->x_index,Top->y_index);
                        //std::cout<<Top->x_index<<", "<<Top->y_index<<std::endl;
                        path_costs[2] = value;

                    }
                    else
                    {
                        dir_vector[2] = -1;
                    }
                }
                if(pos.y_index < 17)
                {
                    if(dir != 2)
                    {
                        Bottom->x_index = pos.x_index;
                        Bottom->y_index = pos.y_index + 1;
                        double value = Calculate_Cost(Bottom->x_index,Bottom->y_index);
                        //std::cout<<Top->x_index<<", "<<Top->y_index<<std::endl;
                        path_costs[3] = value;
                    }
                    else
                    {
                        dir_vector[3] = -1;
                    }
                }


                std::vector<double> dupe_path_costs = path_costs;
                auto it = path_costs.begin();
                std::sort(it,it + 8);

                std::vector<int> sorted_dir;

                for(int i = 0;i < 8;i++)
                {
                    for(int j = 0;j < 8;j++)
                    {
                        if(dupe_path_costs[j] == path_costs[i])
                        {
                            sorted_dir.push_back(dir_vector[j]);
                            break;
                        }
                    }
                    /*if(dir == -1)
                    {
                        std::cout<<sorted_dir[i]<<std::endl;
                        std::cout<<path_costs[i]<<std::endl;
                    }*/
                }


                std::vector<double> total_value = {maxval,maxval,maxval,maxval,maxval,maxval,maxval,maxval};


                for(int i = 0;i < 8; i++)
                {
                    //std::cout<<sorted_dir[i]<<std::endl;
                    if(sorted_dir[i] == 0)
                    {
                        double val = Find_Short_Path(*Left, 0);
                        //std::cout<<"Blocked:0"<<std::endl;
                        if(val + path_costs[i] < maxval)
                        {
                            pos.Left = Left;
                            //maxval = val + path_costs[i];
                            //std::cout<<Left->x_index<<", "<<Left->y_index<<std::endl;
                            //std::cout<<(val + path_costs[i])<<std::endl;
                            total_value[i] = val + path_costs[i];
                        }
                        else
                        {
                            //break;
                        }
                    }
                    else if(sorted_dir[i] == 1)
                    {
                        double val = Find_Short_Path(*Right, 1);
                        if(val + path_costs[i] < maxval)
                        {
                            pos.Right = Right;
                            //maxval = val + path_costs[i];
                            //std::cout<<Right->x_index<<", "<<Right->y_index<<std::endl;
                            //std::cout<<(val + path_costs[i])<<std::endl;
                            total_value[i] = val + path_costs[i];
                        }
                        else
                        {
                            //break;
                        }
                    }
                    else if(sorted_dir[i] == 2)
                    {
                        double val = Find_Short_Path(*Top, 2);
                        if(val + path_costs[i] < maxval)
                        {
                            pos.Top = Top;
                            //maxval = val + path_costs[i];
                            //std::cout<<Top->x_index<<", "<<Top->y_index<<std::endl;
                            //std::cout<<(val + path_costs[i])<<std::endl;
                            total_value[i] = val + path_costs[i];
                        }
                        else
                        {
                            //break;
                        }
                    }
                    else if(sorted_dir[i] == 3)
                    {
                        double val = Find_Short_Path(*Bottom, 3);
                        if(val + path_costs[i] < maxval)
                        {
                            pos.Bottom = Bottom;
                            //maxval = val + path_costs[i];
                            //std::cout<<Bottom->x_index<<", "<<Bottom->y_index<<std::endl;
                            //std::cout<<(val + path_costs[i])<<std::endl;
                            total_value[i] = val + path_costs[i];
                        }
                        else
                        {
                            //break;
                        }
                    }
                    else if(sorted_dir[i] == 4)
                    {
                        double val = Find_Short_Path(*TopLeft, 4);
                        if(val + path_costs[i] < maxval)
                        {
                            pos.TopLeft = TopLeft;
                            //maxval = val + path_costs[i];
                            //std::cout<<TopLeft->x_index<<", "<<TopLeft->y_index<<std::endl;
                            //std::cout<<(val + path_costs[i])<<std::endl;
                            total_value[i] = val + path_costs[i];
                        }
                        else
                        {
                            //break;
                        }
                    }
                    else if(sorted_dir[i] == 5)
                    {
                        double val = Find_Short_Path(*BottomLeft, 5);
                        if(val + path_costs[i] < maxval)
                        {
                            pos.BottomLeft = BottomLeft;
                            //maxval = val + path_costs[i];
                            //std::cout<<BottomLeft->x_index<<", "<<BottomLeft->y_index<<std::endl;
                            //std::cout<<(val + path_costs[i])<<std::endl;
                            total_value[i] = val + path_costs[i];
                        }
                        else
                        {
                            //break;
                        }
                    }
                    else if(sorted_dir[i] == 6)
                    {
                        double val = Find_Short_Path(*TopRight, 6);
                        if(val + path_costs[i] < maxval)
                        {
                            pos.TopRight = TopRight;
                            //maxval = val + path_costs[i];
                            //std::cout<<TopRight->x_index<<", "<<TopRight->y_index<<std::endl;
                            //std::cout<<(val + path_costs[i])<<std::endl;
                            total_value[i] = val + path_costs[i];
                        }
                        else
                        {
                            //break;
                        }
                    }
                    else if(sorted_dir[i] == 7)
                    {

                        double val = Find_Short_Path(*BottomRight, 7);
                        if(val + path_costs[i] < maxval)
                        {
                            pos.BottomRight = BottomRight;
                            //maxval = val + path_costs[i];
                            //std::cout<<BottomRight->x_index<<", "<<BottomRight->y_index<<std::endl;
                            //std::cout<<(val + path_costs[i])<<std::endl;
                            total_value[i] = val + path_costs[i];
                        }
                        else
                        {
                            //break;
                        }
                    }

                }
            
                double minval = std::numeric_limits<double>::max();
                for(int i = 0;i < 8; i++)
                {
                    if(total_value[i] < minval)
                    {
                        minval = total_value[i];
                    }
                }

                return minval;
            
            }
            else
            {
                //std::cout<<"Blocked"<<std::endl;
                return std::numeric_limits<double>::max();
            }

            //
        }
        else
        {
            std::cout<<"Stop Reached"<<std::endl;
            return 0;
        }
    }

    void MoveRight(Position& pos)
    {
        
    }

    void MoveTop(Position& pos)
    {
        
    }

    void MoveBottom(Position& pos)
    {
        
    }

    double Calculate_Cost(int x_index, int y_index)
    {
        double gx = x_index-start_x_index;
        double gx2 = pow(gx,2);
        double gy = y_index-start_y_index;
        double gy2 = pow(gy,2);
        double g2 = gx2 + gy2;
        double g = sqrt(g2);

        double hx = x_index-stop_x_index;
        double hx2 = pow(hx,2);
        double hy = y_index-stop_y_index;
        double hy2 = pow(hy,2);
        double h2 = hx2 + hy2;
        double h = sqrt(h2);

        double sum = g+h;
        return sum;
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
    //ros::Subscriber pose_sub;
    double x = 0;

    //ros::Publisher drive_pub;

    double bot_angle;
    geometry_msgs::Point bot_pose;

    public:

    AStar()
    {
        n = ros::NodeHandle("~");
        std::string drive_topic;
        std::string pose_topic;
        n.getParam("odom_topic",drive_topic);
        n.getParam("pose_topic",pose_topic);
        n.getParam("end_pose_x",stop_x);
        n.getParam("end_pose_y",stop_y);
        //
        GetMapIndex(stop_x,stop_y,stop_x_index,stop_y_index);
        std::cout<<"End : "<<stop_x_index<<", "<<stop_y_index<<endl;
        //std::cout<<drive_topic<<std::endl;
        PathTree path_tree;
    }

    void Calculate_Cost(int x_index, int y_index, double & g, double & h)
    {
        g = sqrt(pow((x_index,start_x_index),2) + pow((y_index,start_y_index),2));
        h = sqrt(pow((x_index,stop_x_index),2) + pow((y_index,stop_y_index),2));
    }

    /*void PoseChange_CallBack(const nav_msgs::Odometry & msg)
    {
        double q3 = msg.pose.pose.orientation.z;
        double q0 = msg.pose.pose.orientation.w;
        bot_angle = atan2((2*(q3*q0)), (1 - (2 * (q3*q3))));
        bot_pose.x = msg.pose.pose.position.x;
        bot_pose.y = msg.pose.pose.position.y;
        bot_pose.z = msg.pose.pose.position.z;
    }*/

    void GetMapIndex(double x, double y, int & x_index, int & y_index)
    {
        x_index = int(9+x);
        y_index = int(9-y);
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