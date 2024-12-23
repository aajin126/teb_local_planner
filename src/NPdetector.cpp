#include <ros/ros.h>
#include <teb_local_planner/teb_config.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/visualization.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <random>
#include <vector>
#include <cmath>

class NarrowPassageDetector
{
public:
    NarrowPassageDetector(double thre, const costmap_2d::Costmap2D* costmap, unsigned int num_samples)
        : thre_(thre), costmap_(costmap), num_samples_(num_samples)
    {
    }

    std::vector<std::pair<geometry_msgs::Point, double>> detectNarrowPassages(const geometry_msgs::Pose& robot_pose)
    {
        std::vector<std::pair<geometry_msgs::Point, double>> narrow_passages;
        
        // Generate Sobol-like samples within the local costmap bounds
        std::vector<geometry_msgs::Point> samples = generateSamples();

        for (const auto& sample : samples)
        {
            // Compute Medial Ball size for the sample point
            double medial_radius = findMedialBallRadius(sample);

            // Define a threshold for identifying narrow passages
            if (medial_radius < thre_) 
            {
                narrow_passages.emplace_back(sample, medial_radius);
            }
        }

        return narrow_passages;
    }

private:
    double thre_;
    const costmap_2d::Costmap2D* costmap_;
    unsigned int num_samples_;

    std::vector<geometry_msgs::Point> generateSamples()
    {
        std::vector<geometry_msgs::Point> samples;

        // Get bounds of the local costmap
        // Create points using a uniform distribution for x and y coordinates.
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
        costmap_->worldToMap(point.x, point.y, mx, my);

        // Check all surrounding cells for obstacles
        double min_distance = std::numeric_limits<double>::max();

        for (unsigned int i = 0; i < costmap_->getSizeInCellsX(); ++i)
        {
            for (unsigned int j = 0; j < costmap_->getSizeInCellsY(); ++j)
            {
                if (costmap_->getCost(i, j) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
                {
                    double wx, wy;
                    costmap_->mapToWorld(i, j, wx, wy);
                    double distance = std::hypot(wx - point.x, wy - point.y);
                    min_distance = std::min(min_distance, distance);
                }
            }
        }

        return min_distance;
    }
};

int main(int argc, char** argv)
{   ros::init(argc, argv, "narrow_passage_detector");
    ros::NodeHandle nh;

    // Load parameters and initialize
    double thre = 0.7; // Example robot radius
    unsigned int num_samples = 10; // Number of samples for detection

    // Initialize tf2 Buffer and TransformListener
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // Create the Costmap2DROS object
    costmap_2d::Costmap2DROS* local_costmap = new costmap_2d::Costmap2DROS("local_costmap", tf_buffer);

    costmap_2d::Costmap2D* costmap = local_costmap->getCostmap();

    // Get the robot's current pose from the Transform Listener
    geometry_msgs::Pose robot_pose;

    try
    {
        geometry_msgs::TransformStamped transform_stamped = tf_buffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
        robot_pose.position.x = transform_stamped.transform.translation.x;
        robot_pose.position.y = transform_stamped.transform.translation.y;
        robot_pose.orientation = transform_stamped.transform.rotation;
    }
    catch (tf2::TransformException& ex)
    {
        ROS_ERROR("%s", ex.what());
        return -1;
    }

    // Create the NarrowPassageDetector with the robot's pose and costmap
    NarrowPassageDetector detector(thre, costmap, num_samples);

    // Detect narrow passages
    std::vector<std::pair<geometry_msgs::Point, double>> narrow_passages = detector.detectNarrowPassages(robot_pose);

    // Output the results
    for (const auto& passage : narrow_passages)
    {
        ROS_INFO("Narrow passage at (%.2f, %.2f) with medial radius %.2f", passage.first.x, passage.first.y, passage.second);
    }

    delete local_costmap;
    return 0;
}
