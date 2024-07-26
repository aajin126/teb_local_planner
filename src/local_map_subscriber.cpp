#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <sensor_msgs/Image.h>
#include <vector>

#include "teb_local_planner/sdt_dead_reckoning.h"

// Singleton or global access
class LocalMapSubscriber
{
public:
    LocalMapSubscriber() : nh_("~"), costmap_sub_(nh_.subscribe("local_costmap", 1, &LocalMapSubscriber::costmapCallback, this))
    {
        distance_field_ = nullptr;
    }

    ~LocalMapSubscriber()
    {
        delete[] distance_field_;
    }

    // Accessor methods
    const std::vector<unsigned char>& getCostmapData() const { return costmap_data_; }
    unsigned int getWidth() const { return width_; }
    unsigned int getHeight() const { return height_; }

    // Calculate distance field if necessary
    const float* getDistanceField() const { return distance_field_; }

private:
    void costmapCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        width_ = msg->width;
        height_ = msg->height;
        unsigned char threshold = 50; // Example value
        costmap_data_.resize(width_ * height_);
        std::copy(msg->data.begin(), msg->data.end(), costmap_data_.begin());

        delete[] distance_field_; // Delete old distance field if it exists
        distance_field_ = new float[width_ * height_];
        sdt_dead_reckoning(width_, height_, threshold, costmap_data_.data(), distance_field_);
    }

    ros::NodeHandle nh_;
    ros::Subscriber costmap_sub_;
    std::vector<unsigned char> costmap_data_;
    unsigned int width_, height_;
    float* distance_field_; // Pointer to store distance field
};

// Global pointer to access LocalMapSubscriber instance
LocalMapSubscriber* local_map_subscriber = nullptr;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_map_subscriber");
    LocalMapSubscriber lms;
    local_map_subscriber = &lms; // Set global pointer to the instance
    ros::spin();
    return 0;
}