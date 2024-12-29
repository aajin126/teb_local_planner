#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <random>
#include <cmath>

class NarrowPassageDetector
{
public:
    NarrowPassageDetector(double thre, const nav_msgs::OccupancyGrid::ConstPtr& costmap, unsigned int num_samples, ros::Publisher& marker_pub)
        : thre_(thre), costmap_(costmap), num_samples_(num_samples), marker_pub_(marker_pub)
    {
    }

    std::vector<std::pair<geometry_msgs::Point, double>> detectNarrowPassages()
    {
        std::vector<std::pair<geometry_msgs::Point, double>> narrow_passages;

        std::vector<geometry_msgs::Point> samples = generateSamples();
        visualizeSamples(samples);

        for (const auto& sample : samples)
        {
            double medial_radius = findMedialBallRadius(sample);

            if (medial_radius < thre_ && medial_radius >= 0.25)
            {
                narrow_passages.emplace_back(sample, medial_radius);
                visualizeMedialBall(sample, medial_radius);
            }
        }

        return narrow_passages;
    }

private:
    double thre_;
    nav_msgs::OccupancyGrid::ConstPtr costmap_;
    unsigned int num_samples_;
    ros::Publisher& marker_pub_;

    std::vector<geometry_msgs::Point> generateSamples()
    {
        std::vector<geometry_msgs::Point> samples;

        double origin_x = costmap_->info.origin.position.x;
        double origin_y = costmap_->info.origin.position.y;
        double resolution = costmap_->info.resolution;
        unsigned int width = costmap_->info.width;
        unsigned int height = costmap_->info.height;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis_x(0, width - 1);
        std::uniform_int_distribution<> dis_y(0, height - 1);

        while (samples.size() < num_samples_)
        {
            int mx = dis_x(gen);
            int my = dis_y(gen);
            int index = my * width + mx;

            if (costmap_->data[index] == 0) // FREE_SPACE only
            {
                geometry_msgs::Point sample;
                sample.x = origin_x + mx * resolution;
                sample.y = origin_y + my * resolution;
                samples.push_back(sample);
            }
        }

        return samples;
    }

    double findMedialBallRadius(const geometry_msgs::Point& point)
{
    double max_radius = std::min(costmap_->info.width, costmap_->info.height) * costmap_->info.resolution / 2.0;
    double step_size = 0.1;
    double radius = 0.2;
    geometry_msgs::Point center = point;

    std::vector<geometry_msgs::Point> boundary_points;
    std::vector<geometry_msgs::Point> fixed_points; // 고정된 점들을 저장하는 리스트
    bool boundary_reached = false;

    while (radius <= max_radius)
    {
        boundary_reached = false;
        boundary_points.clear(); // 매 루프마다 경계 점들을 초기화

        // 원 위의 점들을 살펴봄
        for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 36)
        {
            double sample_x = center.x + radius * std::cos(angle);
            double sample_y = center.y + radius * std::sin(angle);

            geometry_msgs::Point sample_point;
            sample_point.x = sample_x;
            sample_point.y = sample_y;
            sample_point.z = 0.0;  // 기본값 설정

            // 고정된 점이 아니라면 경계 점을 확인
            if (isObstacleOrUnknown(sample_x, sample_y) && std::find(fixed_points.begin(), fixed_points.end(), sample_point) == fixed_points.end())
            {
                boundary_points.push_back(sample_point);
                boundary_reached = true;
            }
        }

        // 두 개 이상의 점이 경계에 닿았다면, 원을 더 이상 확장하지 않고 중심을 이동시킴
        if (boundary_reached && boundary_points.size() > 1)
        {
            // 경계 점들을 고정시킴
            fixed_points.insert(fixed_points.end(), boundary_points.begin(), boundary_points.end());

            // 원 중심을 고정된 점들로부터 벡터를 계산하여 이동
            for (const auto& fixed_point : fixed_points)
            {
                // 고정된 점과 center 사이의 벡터 계산
                double vector_x = fixed_point.x - center.x;
                double vector_y = fixed_point.y - center.y;

                // 벡터의 반대방향으로 이동 (벡터의 방향을 반전)
                double move_x = center.x - vector_x;
                double move_y = center.y - vector_y;

                // 중심을 이동
                center.x = move_x;
                center.y = move_y;
            }

            // 최종적으로 반지름을 반환
            return radius;
        }

        // 한 점만 장애물에 닿은 경우에는 원의 반지름을 계속 확장
        radius += step_size;
    }

    return max_radius;
}

    bool isObstacleOrUnknown(double x, double y)
    {
        int mx = static_cast<int>((x - costmap_->info.origin.position.x) / costmap_->info.resolution);
        int my = static_cast<int>((y - costmap_->info.origin.position.y) / costmap_->info.resolution);

        if (mx < 0 || my < 0 || mx >= static_cast<int>(costmap_->info.width) || my >= static_cast<int>(costmap_->info.height))
        {
            return true; // Out of bounds
        }

        int index = my * costmap_->info.width + mx;
        return costmap_->data[index] == 100 || costmap_->data[index] == -1; // LETHAL_OBSTACLE or NO_INFORMATION
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
        marker.id = static_cast<int>(center.x * 1000 + center.y * 1000);
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 0.02;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        // Draw the medial ball centered at the new position
        for (double angle = 0; angle <= 2 * M_PI; angle += M_PI / 36)
        {
            geometry_msgs::Point p;
            p.x = center.x + radius * std::cos(angle);
            p.y = center.y + radius * std::sin(angle);
            p.z = 0.0;
            marker.points.push_back(p);
        }

        ros::Duration lifetime(1.0);
        marker.lifetime = lifetime;

        marker_pub_.publish(marker);
    }
};

nav_msgs::OccupancyGrid::ConstPtr latest_costmap_msg;

void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    latest_costmap_msg = msg;
    ROS_INFO_THROTTLE(5, "Costmap data updated.");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "narrow_passage_detector");
    ros::NodeHandle nh;

    double threshold = 0.6;
    unsigned int num_samples = 5;

    ros::Subscriber costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/local_costmap/costmap", 5, costmapCallback);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 5);

    ros::Rate rate(10);
    while (ros::ok())
    {
        if (latest_costmap_msg)
        {
            NarrowPassageDetector detector(threshold, latest_costmap_msg, num_samples, marker_pub);
            std::vector<std::pair<geometry_msgs::Point, double>> narrow_passages = detector.detectNarrowPassages();

            for (const auto& passage : narrow_passages)
            {
                ROS_INFO("Narrow passage detected at (x=%.2f, y=%.2f) with medial radius %.2f",
                         passage.first.x, passage.first.y, passage.second);
            }
        }
        else
        {
            ROS_WARN_THROTTLE(5, "Waiting for costmap data...");
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
