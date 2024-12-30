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
            double medial_radius = findMedialBallRadius(sample).first;
            geometry_msgs::Point final_center = findMedialBallRadius(sample).second;

            if (medial_radius < thre_ && medial_radius >= 0.25)
            {
                narrow_passages.emplace_back(final_center, medial_radius);
                visualizeMedialBall(final_center, medial_radius);
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

    double calculateAngle(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& center)
    {
        double vector1_x = p1.x - center.x;
        double vector1_y = p1.y - center.y;
        double vector2_x = p2.x - center.x;
        double vector2_y = p2.y - center.y;

        double dot_product = vector1_x * vector2_x + vector1_y * vector2_y;
        double magnitude1 = std::sqrt(vector1_x * vector1_x + vector1_y * vector1_y);
        double magnitude2 = std::sqrt(vector2_x * vector2_x + vector2_y * vector2_y);

        if (magnitude1 > 0 && magnitude2 > 0)
        {
            double cosine_angle = dot_product / (magnitude1 * magnitude2);

            // 클램프를 사용하지 않고 범위를 제한
            if (cosine_angle < -1.0)
                cosine_angle = -1.0;
            else if (cosine_angle > 1.0)
                cosine_angle = 1.0;

            return std::acos(cosine_angle); // 각도 (라디안)
        }
        return 0.0;
    }

    std::pair<double, geometry_msgs::Point> findMedialBallRadius(const geometry_msgs::Point& point)
    {
        double max_radius = std::min(costmap_->info.width, costmap_->info.height) * costmap_->info.resolution / 2.0;
        double step_size = 0.01;
        double radius = 0.2;
        geometry_msgs::Point center = point;

        while (radius <= max_radius)
        {
            std::vector<geometry_msgs::Point> boundary_points;

            // 원의 경계 점 계산
            for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 36)
            {
                double sample_x = center.x + radius * std::cos(angle);
                double sample_y = center.y + radius * std::sin(angle);

                if (isObstacleOrUnknown(sample_x, sample_y))
                {
                    geometry_msgs::Point boundary_point;
                    boundary_point.x = sample_x;
                    boundary_point.y = sample_y;
                    boundary_point.z = 0.0; // 기본값 설정
                    boundary_points.push_back(boundary_point);
                }
            }

            // 경계 점이 발견되지 않으면 반지름 증가
            if (boundary_points.empty())
            {
                radius += step_size;
                continue;
            }

            // 경계 점이 두 개 이상일 경우 종료 조건
            if (boundary_points.size() > 1)
            {
                const auto& last_boundary_point = boundary_points.back();
                double angle = calculateAngle(boundary_points.front(), last_boundary_point, center);

                if (angle > M_PI / 2.0)
                {
                    return {radius, center};
                }
                
            }

            // 중심 이동 계산
            const auto& boundary_point = boundary_points.front();

            double vector_x = boundary_point.x - center.x;
            double vector_y = boundary_point.y - center.y;

            double magnitude = std::sqrt(vector_x * vector_x + vector_y * vector_y);
            if (magnitude > 0)
            {
                center.x -= (vector_x / magnitude) * step_size;
                center.y -= (vector_y / magnitude) * step_size;
            }

            // 반지름 증가
            radius += step_size;

        }

        return {radius, center}; // 최대 반지름에 도달한 경우 반환
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
