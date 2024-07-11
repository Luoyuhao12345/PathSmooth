#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>


using namespace std;


float dt = 2;
bool path_received = false;
nav_msgs::Path global_path = nav_msgs::Path();
geometry_msgs::PoseStamped goal_msg = geometry_msgs::PoseStamped();

vector<float> xs;
vector<float> ys;
vector<float> yaws;

ros::Publisher global_path_pub;
ros::Publisher goal_pub;

void read_map()
{
    float data1,data2,data3;
    string package_path = ros::package::getPath("fixed_path_planner");
    string sub_path = "/map/backyard.txt";
    string map_path = package_path+sub_path;
    ROS_INFO("the map path: %s", map_path.c_str());
    FILE* fsp=fopen(map_path.c_str(), "r");
    while(fscanf(fsp,"%f%f%f",&data1,&data2,&data3)!=EOF)
    {
        xs.push_back(data1);
        ys.push_back(data2);
        yaws.push_back(data3);
    }
    fclose(fsp);
    cout<<"map has been read"<<endl;
}


void set_ref_path()
{
    string frame_string = "/map";
    int n = xs.size();
    ros::Time current_time = ros::Time::now();
    global_path.header.stamp = current_time;
    global_path.header.frame_id = frame_string;
    // 路径
    for (int i = 0; i < n; i++)
    {
        geometry_msgs::PoseStamped current_point = geometry_msgs::PoseStamped();
        current_point.pose.position.x = xs[i];
        current_point.pose.position.y = ys[i];
        current_point.header.frame_id = frame_string;
        current_point.header.stamp = current_time;
        global_path.poses.push_back(current_point);
    }
    // 目标点
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.header.frame_id = "map";
    goal_msg.pose.position.x = xs[n-1];
    goal_msg.pose.position.y = ys[n-1];
    path_received = true;
    cout<<"ref_path init"<<endl;
}

void global_planner(const ros::TimerEvent& event)
{
    if(!path_received) return;
    global_path_pub.publish(global_path);
    goal_pub.publish(goal_msg);
    cout<<"pub map"<<endl;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_global_planner");
    ros::NodeHandle nh;
    ros::Timer timer = nh.createTimer(ros::Duration(dt), global_planner);

    global_path_pub = nh.advertise<nav_msgs::Path>("/my_global_planner", 10);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    read_map();
    set_ref_path();

    ros::spin();
    return 0;
}

