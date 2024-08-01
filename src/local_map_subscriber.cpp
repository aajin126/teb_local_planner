#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <vector>
#include <algorithm>
#include <nav_msgs/OccupancyGrid.h>

#include "teb_local_planner/sdt_dead_reckoning.h"
#include "teb_local_planner/local_map_subscriber.h"


LocalMapSubscriber* local_map_subscriber = nullptr;

LocalMapSubscriber::LocalMapSubscriber() : nh_("~"), costmap_sub_(nh_.subscribe("/move_base/local_costmap/costmap", 1, &LocalMapSubscriber::costmapCallback, this))
{
    distance_field_ = nullptr;
    ROS_INFO("LocalMapSubscriber initialized.");
}

LocalMapSubscriber::~LocalMapSubscriber()
{
    delete[] distance_field_;
}

const std::vector<unsigned char>& LocalMapSubscriber::getCostmapData() const
{
    return costmap_data_;
}

unsigned int LocalMapSubscriber::getWidth() const
{
    return width_;
}

unsigned int LocalMapSubscriber::getHeight() const
{
    return height_;
}

const float* LocalMapSubscriber::getDistanceField() const
{
    return distance_field_;
}

bool LocalMapSubscriber::isDataReady() const
{
    return data_ready_;
}

void LocalMapSubscriber::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    width_ = msg->info.width;
    height_ = msg->info.height;
    unsigned char threshold = 50; // 예시 값

    costmap_data_.resize(width_ * height_);
    std::copy(msg->data.begin(), msg->data.end(), costmap_data_.begin());

    delete[] distance_field_; // 이전 거리 필드 삭제
    distance_field_ = new float[width_ * height_];
    sdt_dead_reckoning(width_, height_, threshold, costmap_data_.data(), distance_field_);
    data_ready_ = true; // Set data_ready_ to true once data is updated
    ROS_INFO("Distance field updated.");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_map_subscriber");
    LocalMapSubscriber lms;
    local_map_subscriber = &lms; // Set global pointer to the instance
    ROS_INFO("Local map subscriber node started.");
    ros::spin();
    return 0;
}