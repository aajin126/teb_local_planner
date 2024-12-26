#include "ros/ros.h"
#include <teb_local_planner/teb_config.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/visualization.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <random>
#include <vector>
#include <cmath>
#include <visualization_msgs/Marker.h>

class NarrowPassageDetector
{
public:
    NarrowPassageDetector(double thre, const costmap_2d::Costmap2D* costmap, unsigned int num_samples, ros::Publisher& marker_pub)
        : thre_(thre), costmap_(costmap), num_samples_(num_samples), marker_pub_(marker_pub)
    {
    }

    std::vector<std::pair<geometry_msgs::Point, double>> detectNarrowPassages(const geometry_msgs::Pose& robot_pose)
    {
        std::vector<std::pair<geometry_msgs::Point, double>> narrow_passages;

        // Generate Sobol-like samples within the local costmap bounds
        std::vector<geometry_msgs::Point> samples = generateSamples();

        // Visualize sampled points
        visualizeSamples(samples);

        for (const auto& sample : samples)
        {
            // Compute Medial Ball size for the sample point
            double medial_radius = findMedialBallRadius(sample);

            // Define a threshold for identifying narrow passages
            if (medial_radius < thre_)
            {
                narrow_passages.emplace_back(sample, medial_radius);
            }

            // Visualize medial ball
            visualizeMedialBall(sample, medial_radius);
        }

        return narrow_passages;
    }

private:
    double thre_;
    const costmap_2d::Costmap2D* costmap_;
    unsigned int num_samples_;
    ros::Publisher& marker_pub_;

    std::vector<geometry_msgs::Point> generateSamples()
    {
        std::vector<geometry_msgs::Point> samples;

        // Get bounds of the local costmap
        double origin_x = costmap_->getOriginX();
        double origin_y = costmap_->getOriginY();
        double width = costmap_->getSizeInMetersX();
        double height = costmap_->getSizeInMetersY();

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis_x(origin_x, origin_x + width);
        std::uniform_real_distribution<> dis_y(origin_y, origin_y + height);

        for (unsigned int i = 0; i < num_samples_; ++i)
        {
            geometry_msgs::Point sample;
            sample.x = dis_x(gen);
            sample.y = dis_y(gen);
            samples.push_back(sample);
        }

        return samples;
    }

    double findMedialBallRadius(const geometry_msgs::Point& point)
    {
        unsigned int mx, my;

        // 점이 costmap 내부에 있는지 확인
        if (!costmap_->worldToMap(point.x, point.y, mx, my))
        {
            ROS_WARN("Point (%.2f, %.2f) is outside the costmap bounds.", point.x, point.y);
            return 0.0; // 맵 밖이면 반경 0 반환
        }

        const double max_radius = std::min(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY()) / 2.0;
        const double step_size = 0.1; // 반경 증가 간격
        double radius = 0.0;

        while (radius <= max_radius)
        {
            bool obstacle_found = false;

            // 현재 반경에서 10도 간격으로 원형 경계 샘플링
            for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 18)
            {
                double sample_x = point.x + radius * std::cos(angle);
                double sample_y = point.y + radius * std::sin(angle);

                unsigned int sx, sy;
                if (costmap_->worldToMap(sample_x, sample_y, sx, sy))
                {
                    // 장애물 레이어에서만 확인
                    if (costmap_->getCost(sx, sy) == costmap_2d::LETHAL_OBSTACLE)
                    {
                        obstacle_found = true;
                        break;
                    }
                }
                else
                {
                    // 원이 맵 밖으로 나가면 장애물로 간주
                    obstacle_found = true;
                    break;
                }
            }

        if (obstacle_found)
        {
            return radius; // 장애물에 닿으면 반경 반환
        }

        radius += step_size; // 반경 증가
    }

    // 최대 반경 초과 시 제한
    return max_radius;
    }

    void visualizeSamples(const std::vector<geometry_msgs::Point>& samples)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "samples";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        for (const auto& sample : samples)
        {
            geometry_msgs::Point p;
            p.x = sample.x;
            p.y = sample.y;
            p.z = 0.0;
            marker.points.push_back(p);
        }

        marker_pub_.publish(marker);
    }

    void visualizeMedialBall(const geometry_msgs::Point& center, double radius)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "medial_balls";
        marker.id = center.x * 1000 + center.y * 1000; // Unique ID for each sphere
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = center.x;
        marker.pose.position.y = center.y;
        marker.pose.position.z = 0.0;
        marker.scale.x = radius * 2;
        marker.scale.y = radius * 2;
        marker.scale.z = 0.1;

        marker.color.a = 0.5;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        marker_pub_.publish(marker);
    }
};

// Global variable to store the latest Odometry message
nav_msgs::Odometry::ConstPtr latest_odom_msg;

// Odometry callback function
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    latest_odom_msg = msg;
    ROS_INFO_THROTTLE(5, "Odometry data updated. Current position: x=%.2f, y=%.2f", 
                      msg->pose.pose.position.x, msg->pose.pose.position.y);
}

geometry_msgs::Pose getRobotPoseFromOdom(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    return odom_msg->pose.pose;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "narrow_passage_detector");
    ros::NodeHandle nh;

    // Load parameters and initialize
    double threshold = 0.7; // Example robot radius threshold
    unsigned int num_samples = 10; // Number of detection samples

    // Create the Costmap2DROS object
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    costmap_2d::Costmap2DROS local_costmap("local_costmap", tf_buffer);

    // Wait for Costmap initialization
    while (!local_costmap.isCurrent())
    {
        ROS_WARN_THROTTLE(2, "Waiting for local costmap to initialize...");
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    ROS_INFO("Local costmap initialized.");

    costmap_2d::Costmap2D* costmap = local_costmap.getCostmap();

    // Subscribe to Odometry topic
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, odomCallback);

    // Create Marker publisher
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    // Create the NarrowPassageDetector
    NarrowPassageDetector detector(threshold, costmap, num_samples, marker_pub);

    ros::Rate rate(10); // Loop rate: 10Hz
    while (ros::ok())
    {
        if (latest_odom_msg)
        {
            geometry_msgs::Pose robot_pose = getRobotPoseFromOdom(latest_odom_msg);

            // Detect narrow passages
            std::vector<std::pair<geometry_msgs::Point, double>> narrow_passages = detector.detectNarrowPassages(robot_pose);

            // Output the results
            for (const auto& passage : narrow_passages)
            {
                ROS_INFO("Narrow passage detected at (x=%.2f, y=%.2f) with medial radius %.2f", 
                         passage.first.x, passage.first.y, passage.second);
            }
        }
        else
        {
            ROS_WARN_THROTTLE(5, "Waiting for Odometry data...");
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
