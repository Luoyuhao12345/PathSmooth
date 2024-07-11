#include "smoother.h"

std::vector<geometry_msgs::PoseStamped> path_trim(const nav_msgs::Path::ConstPtr& path){
    std::vector<geometry_msgs::PoseStamped> result;
    if (path->poses.empty()) {
        return result;
    }
    result.push_back(path->poses.front());

    size_t prevIdx = 0;
    for (size_t i = 1; i < path->poses.size(); ++i) {
        const geometry_msgs::PoseStamped& prevPose = path->poses[prevIdx];
        const geometry_msgs::PoseStamped& currPose = path->poses[i];
        double distance = std::hypot(currPose.pose.position.x - prevPose.pose.position.x,
                                        currPose.pose.position.y - prevPose.pose.position.y);
        if (distance >= 0.1) {
            result.push_back(currPose);
            prevIdx = i;
        }
    }
result.push_back(path->poses.back());
return result;
}

nav_msgs::Path Smoother::smoothPath(const nav_msgs::Path::ConstPtr& path){
    int iterations = 0;
    int maxIterations = 100;
    std::vector<geometry_msgs::PoseStamped> poses = path_trim(path);
    int pathLength = poses.size();
    // std::cout<<"size:"<<pathLength<<std::endl;
    while (iterations < maxIterations){
        for (int i = 2; i < pathLength - 2; ++i){
            //后面2个点，当前点，前面2个点
            Vector2D xim2(poses[i-2].pose.position.x, poses[i-2].pose.position.y);
            Vector2D xim1(poses[i-1].pose.position.x, poses[i-1].pose.position.y);
            Vector2D xi(poses[i].pose.position.x, poses[i].pose.position.y);
            Vector2D xip1(poses[i+1].pose.position.x, poses[i+1].pose.position.y);
            Vector2D xip2(poses[i+2].pose.position.x, poses[i+2].pose.position.y);
            Vector2D correction;
            correction = correction - smoothnessTerm(xim2, xim1, xi, xip1, xip2);
            xi = xi + alpha*correction;

            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = xi.getX();
            pose.pose.position.y = xi.getY();
            poses[i] = pose;
        }
        iterations++;
    }
    nav_msgs::Path smooth_path;
    smooth_path.header = path->header;
    smooth_path.poses = poses;
    return smooth_path;
}


Vector2D Smoother::smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2) {
  return wSmoothness * (xim2 - 4 * xim1 + 6 * xi - 4 * xip1 + xip2);
}