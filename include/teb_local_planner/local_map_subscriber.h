#ifndef LOCAL_MAP_SUBSCRIBER_H
#define LOCAL_MAP_SUBSCRIBER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vector>

class LocalMapSubscriber
{
public:
    LocalMapSubscriber();
    ~LocalMapSubscriber();

    // Accessor methods
    const std::vector<unsigned char>& getCostmapData() const;
    unsigned int getWidth() const;
    unsigned int getHeight() const;
    const float* getDistanceField() const;

private:
    void costmapCallback(const sensor_msgs::Image::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber costmap_sub_;
    std::vector<unsigned char> costmap_data_;
    unsigned int width_, height_;
    float* distance_field_; // Pointer to store distance field
};

// Global pointer to access LocalMapSubscriber instance
extern LocalMapSubscriber* local_map_subscriber;

#endif // LOCAL_MAP_SUBSCRIBER_H