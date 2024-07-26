#include <geometry_msgs/Pose.h>
#include <cmath>
#include <vector>

#include "teb_local_planner/sdt_dead_reckoning.h"


void pushPoseAwayFromObstacle(geometry_msgs::Pose& pose, const std::vector<unsigned char>& costmap_data, unsigned int width, unsigned int height)
{
    unsigned char threshold = 50; // 예시 값
    float* distance_field = new float[width * height];
    sdt_dead_reckoning(width, height, threshold, costmap_data.data(), distance_field);

    int xi = static_cast<int>(pose.position.x);
    int yi = static_cast<int>(pose.position.y);

    if (xi >= 0 && xi < width && yi >= 0 && yi < height)
    {
        float distance = distance_field[xi + yi * width];
        if (distance != -1)
        {
            float dx = xi - pose.position.x;
            float dy = yi - pose.position.y;
            float angle = atan2(dy, dx);
            float move_distance = 0.5 * distance; // Move 50% of the distance away from the obstacle

            pose.position.x += move_distance * cos(angle);
            pose.position.y += move_distance * sin(angle);
        }
    }

    delete[] distance_field;
}