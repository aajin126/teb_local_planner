#include "teb_local_planner/sdt_dead_reckoning.h"
#include <stdlib.h>
#include <math.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <limits>

#define SDT_DEAD_RECKONING_IMPLEMENTATION

void sdt_dead_reckoning(unsigned int width, unsigned int height, unsigned char threshold, const unsigned char* image, float* distance_field) {
    // The internal buffers have a 1px padding around them so we can avoid border checks in the loops below
    unsigned int padded_width = width + 2;
    unsigned int padded_height = height + 2;

    // px and py store the corresponding border point for each pixel (just p in the paper, here x and y
    // are separated into px and py).
    int* px = (int*)malloc(padded_width * padded_height * sizeof(px[0]));
    int* py = (int*)malloc(padded_width * padded_height * sizeof(py[0]));
    float* padded_distance_field = (float*)malloc(padded_width * padded_height * sizeof(padded_distance_field[0]));

    // Create macros as local shorthands to access the buffers. Push (and later restore) any previous macro definitions so we
    // don't overwrite any macros of the user. The names are similar to the names used in the paper so you can use the pseudo-code
    // in the paper as reference.
    #pragma push_macro("I")
    #pragma push_macro("D")
    #pragma push_macro("PX")
    #pragma push_macro("PY")
    #pragma push_macro("LENGTH")
    // image is unpadded so x and y are in the range 0..width-1 and 0..height-1
    #define I(x, y) (image[(x) + (y) * width] > threshold)
    // The internal buffers are padded x and y are in the range 0..padded_width-1 and 0..padded_height-1
    #define D(x, y) padded_distance_field[(x) + (y) * (padded_width)]
    #define PX(x, y) px[(x) + (y) * padded_width]
    #define PY(x, y) py[(x) + (y) * padded_width]
    // We use a macro instead of the hypotf() function because it's a major performance boost (~26ms down to ~17ms)
    #define LENGTH(x, y) sqrtf((x)*(x) + (y)*(y))

    // Initialize internal buffers
    for(unsigned int y = 0; y < padded_height; ++y) {
        for(unsigned int x = 0; x < padded_width; ++x) {
            D(x, y) = INFINITY;
            PX(x, y) = -1;
            PY(x, y) = -1;
        }
    }

    // Initialize immediate interior and exterior elements
    // We iterate over the unpadded image and skip the outermost pixels of it (because we look 1px into each direction)
    for(unsigned int y = 1; y < height-1; ++y) {
        for(unsigned int x = 1; x < width-1; ++x) {
            int on_immediate_interior_or_exterior = (
                I(x-1, y) != I(x, y)  ||  I(x+1, y) != I(x, y)  ||
                I(x, y-1) != I(x, y)  ||  I(x, y+1) != I(x, y)
            );
            if ( I(x, y) && on_immediate_interior_or_exterior ) {
                // The internal buffers have a 1px padding so we need to add 1 to the coordinates of the unpadded image
                D(x+1, y+1) = 0;
                PX(x+1, y+1) = x+1;
                PY(x+1, y+1) = y+1;
            }
        }
    }

    // Horizontal (dx), vertical (dy) and diagonal (dxy) distances between pixels
    const float dx = 1.0, dy = 1.0, dxy = 1.4142135623730950488 /* sqrtf(2) */;

    // Perform the first pass
    // We iterate over the padded internal buffers but skip the outermost pixel because we look 1px into each direction
    for(unsigned int y = 1; y < padded_height-1; ++y) {
        for(unsigned int x = 1; x < padded_width-1; ++x) {
            if ( D(x-1, y-1) + dxy < D(x, y) ) {
                PX(x, y) = PX(x-1, y-1);
                PY(x, y) = PY(x-1, y-1);
                D(x, y) = LENGTH(x - PX(x, y), y - PY(x, y));
            }
            if ( D(x, y-1) + dy < D(x, y) ) {
                PX(x, y) = PX(x, y-1);
                PY(x, y) = PY(x, y-1);
                D(x, y) = LENGTH(x - PX(x, y), y - PY(x, y));
            }
            if ( D(x+1, y-1) + dxy < D(x, y) ) {
                PX(x, y) = PX(x+1, y-1);
                PY(x, y) = PY(x+1, y-1);
                D(x, y) = LENGTH(x - PX(x, y), y - PY(x, y));
            }
            if ( D(x-1, y) + dx < D(x, y) ) {
                PX(x, y) = PX(x-1, y);
                PY(x, y) = PY(x-1, y);
                D(x, y) = LENGTH(x - PX(x, y), y - PY(x, y));
            }
        }
    }

    // Perform the final pass
    for(unsigned int y = padded_height-2; y >= 1; --y) {
        for(unsigned int x = padded_width-2; x >= 1; --x) {
            if ( D(x+1, y) + dx < D(x, y) ) {
                PX(x, y) = PX(x+1, y);
                PY(x, y) = PY(x+1, y);
                D(x, y) = LENGTH(x - PX(x, y), y - PY(x, y));
            }
            if ( D(x-1, y+1) + dxy < D(x, y) ) {
                PX(x, y) = PX(x-1, y+1);
                PY(x, y) = PY(x-1, y+1);
                D(x, y) = LENGTH(x - PX(x, y), y - PY(x, y));
            }
            if ( D(x, y+1) + dy < D(x, y) ) {
                PX(x, y) = PX(x, y+1);
                PY(x, y) = PY(x, y+1);
                D(x, y) = LENGTH(x - PX(x, y), y - PY(x, y));
            }
            if ( D(x+1, y+1) + dx < D(x, y) ) {
                PX(x, y) = PX(x+1, y+1);
                PY(x, y) = PY(x+1, y+1);
                D(x, y) = LENGTH(x - PX(x, y), y - PY(x, y));
            }
        }
    }

    // Set the proper sign for inside and outside and write the result into the output distance field
    for(unsigned int y = 0; y < height; ++y) {
        for(unsigned int x = 0; x < width; ++x) {
            float sign = I(x, y) ? -1 : 1;
            distance_field[x + y*width] = D(x+1, y+1) * sign;
        }
    }

    // Restore macros and free internal buffers
    #pragma pop_macro("I")
    #pragma pop_macro("D")
    #pragma pop_macro("PX")
    #pragma pop_macro("PY")
    #pragma pop_macro("LENGTH")

    free(padded_distance_field);
    free(px);
    free(py);
}

DistanceFieldUpdater::DistanceFieldUpdater(ros::NodeHandle& nh) : nh_(nh), map_received_(false), map_width_(0), map_height_(0) {
    costmap_sub_ = nh_.subscribe("/move_base/global_costmap/costmap", 1, &DistanceFieldUpdater::costmapCallback, this);
    robot_distance_pub_ = nh_.advertise<sensor_msgs::Image>("/distance_map", 1);
    ROS_INFO("DistanceFieldUpdater initialized");
}

float DistanceFieldUpdater::getDistanceAt(double x, double y) const {
    if (!map_received_) {
        ROS_WARN("Map not received yet. Returning large distance.");
        return std::numeric_limits<float>::max();
    }

    int grid_x = static_cast<int>(x / map_resolution_);
    int grid_y = static_cast<int>(y / map_resolution_);

    ROS_INFO("grid_x: %d, grid_y: %d, map_width: %d, map_height: %d", grid_x, grid_y, map_width_, map_height_);

    return distance_field_[grid_y * map_width_ + grid_x];
}

bool DistanceFieldUpdater::isDataReady() const {
    return map_received_;
}

const float* DistanceFieldUpdater::getDistanceField() const {
    return distance_field_.data();
}

unsigned int DistanceFieldUpdater::getWidth() const {
    return map_width_;
}

unsigned int DistanceFieldUpdater::getHeight() const {
    return map_height_;
}

void DistanceFieldUpdater::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_width_ = msg->info.width;
    map_height_ = msg->info.height;
    map_resolution_ = msg->info.resolution;
    map_received_ = true;

    distance_field_.resize(map_width_ * map_height_);

    std::vector<unsigned char> unsigned_costmap_data(msg->data.size());
    for (size_t i = 0; i < msg->data.size(); ++i) {
        unsigned_costmap_data[i] = static_cast<unsigned char>(msg->data[i]);
    }
    sdt_dead_reckoning(map_width_, map_height_, 0, unsigned_costmap_data.data(), distance_field_.data());

    ROS_DEBUG("Costmap received and distance field updated.");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_field_updater");
    ros::NodeHandle nh;

    DistanceFieldUpdater map_subscriber_(nh);

    ros::spin();
    return 0;
}
