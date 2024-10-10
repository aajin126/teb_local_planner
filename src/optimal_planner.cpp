/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/optimal_planner.h>

// g2o custom edges and vertices for the TEB planner
#include <teb_local_planner/g2o_types/edge_velocity.h>
#include <teb_local_planner/g2o_types/edge_velocity_obstacle_ratio.h>
#include <teb_local_planner/g2o_types/edge_acceleration.h>
#include <teb_local_planner/g2o_types/edge_kinematics.h>
#include <teb_local_planner/g2o_types/edge_time_optimal.h>
#include <teb_local_planner/g2o_types/edge_shortest_path.h>
#include <teb_local_planner/g2o_types/edge_obstacle.h>
#include <teb_local_planner/g2o_types/edge_dynamic_obstacle.h>
#include <teb_local_planner/g2o_types/edge_via_point.h>
#include <teb_local_planner/g2o_types/edge_prefer_rotdir.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layered_costmap.h>
#include <teb_local_planner/pose_se2.h>

#include "teb_local_planner/sdt_dead_reckoning.h"

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <teb_local_planner/TrajectoryMsg.h>

#include <vector>
#include <memory>
#include <limits>

namespace teb_local_planner
{

// ============== Implementation ===================

TebOptimalPlanner::TebOptimalPlanner() : cfg_(NULL), obstacles_(NULL), via_points_(NULL), cost_(HUGE_VAL), prefer_rotdir_(RotType::none),
                                         initialized_(false), optimized_(false)
{    
}
  
TebOptimalPlanner::TebOptimalPlanner(const TebConfig& cfg, ObstContainer* obstacles, TebVisualizationPtr visual, const ViaPointContainer* via_points)
{
  initialize(cfg, obstacles, visual, via_points);

  ros::NodeHandle nh;
  map_subscriber_ = std::make_shared<DistanceFieldUpdater>(nh);
}

TebOptimalPlanner::~TebOptimalPlanner()
{
  clearGraph();
  // free dynamically allocated memory
  //if (optimizer_) 
  //  g2o::Factory::destroy();
  //g2o::OptimizationAlgorithmFactory::destroy();
  //g2o::HyperGraphActionLibrary::destroy();
}

void TebOptimalPlanner::initialize(const TebConfig& cfg, ObstContainer* obstacles, TebVisualizationPtr visual, const ViaPointContainer* via_points)
{
  ROS_DEBUG("TEB initialize");    
  // init optimizer (set solver and block ordering settings)
  optimizer_ = initOptimizer();
  
  cfg_ = &cfg;
  obstacles_ = obstacles;
  via_points_ = via_points;
  cost_ = HUGE_VAL;
  prefer_rotdir_ = RotType::none;
  setVisualization(visual);
  
  vel_start_.first = true;
  vel_start_.second.linear.x = 0;
  vel_start_.second.linear.y = 0;
  vel_start_.second.angular.z = 0;

  vel_goal_.first = true;
  vel_goal_.second.linear.x = 0;
  vel_goal_.second.linear.y = 0;
  vel_goal_.second.angular.z = 0;
}


void TebOptimalPlanner::setVisualization(TebVisualizationPtr visualization)
{
  visualization_ = visualization;
}

void TebOptimalPlanner::visualize()
{
  if (!visualization_)
    return;
 
  visualization_->publishLocalPlanAndPoses(teb_);
  
  if (teb_.sizePoses() > 0)
    visualization_->publishRobotFootprintModel(teb_.Pose(0), *cfg_->robot_model);
  
  if (cfg_->trajectory.publish_feedback)
    visualization_->publishFeedbackMessage(*this, *obstacles_);
 
}


/*
 * registers custom vertices and edges in g2o framework
 */
void TebOptimalPlanner::registerG2OTypes()
{
  g2o::Factory* factory = g2o::Factory::instance();
  factory->registerType("VERTEX_POSE", new g2o::HyperGraphElementCreator<VertexPose>);
  factory->registerType("VERTEX_TIMEDIFF", new g2o::HyperGraphElementCreator<VertexTimeDiff>);

  factory->registerType("EDGE_TIME_OPTIMAL", new g2o::HyperGraphElementCreator<EdgeTimeOptimal>);
  factory->registerType("EDGE_SHORTEST_PATH", new g2o::HyperGraphElementCreator<EdgeShortestPath>);
  factory->registerType("EDGE_VELOCITY", new g2o::HyperGraphElementCreator<EdgeVelocity>);
  factory->registerType("EDGE_VELOCITY_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeVelocityHolonomic>);
  factory->registerType("EDGE_ACCELERATION", new g2o::HyperGraphElementCreator<EdgeAcceleration>);
  factory->registerType("EDGE_ACCELERATION_START", new g2o::HyperGraphElementCreator<EdgeAccelerationStart>);
  factory->registerType("EDGE_ACCELERATION_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationGoal>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomic>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_START", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicStart>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicGoal>);
  factory->registerType("EDGE_KINEMATICS_DIFF_DRIVE", new g2o::HyperGraphElementCreator<EdgeKinematicsDiffDrive>);
  factory->registerType("EDGE_KINEMATICS_CARLIKE", new g2o::HyperGraphElementCreator<EdgeKinematicsCarlike>);
  factory->registerType("EDGE_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeObstacle>);
  factory->registerType("EDGE_INFLATED_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeInflatedObstacle>);
  factory->registerType("EDGE_DYNAMIC_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeDynamicObstacle>);
  factory->registerType("EDGE_VIA_POINT", new g2o::HyperGraphElementCreator<EdgeViaPoint>);
  factory->registerType("EDGE_PREFER_ROTDIR", new g2o::HyperGraphElementCreator<EdgePreferRotDir>);
  return;
}

/*
 * initialize g2o optimizer. Set solver settings here.
 * Return: pointer to new SparseOptimizer Object.
 */
boost::shared_ptr<g2o::SparseOptimizer> TebOptimalPlanner::initOptimizer()
{
  // Call register_g2o_types once, even for multiple TebOptimalPlanner instances (thread-safe)
  static boost::once_flag flag = BOOST_ONCE_INIT;
  boost::call_once(&registerG2OTypes, flag);  

  // allocating the optimizer
  boost::shared_ptr<g2o::SparseOptimizer> optimizer = boost::make_shared<g2o::SparseOptimizer>();
  std::unique_ptr<TEBLinearSolver> linear_solver(new TEBLinearSolver()); // see typedef in optimization.h
  linear_solver->setBlockOrdering(true);
  std::unique_ptr<TEBBlockSolver> block_solver(new TEBBlockSolver(std::move(linear_solver)));
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

  optimizer->setAlgorithm(solver);
  
  optimizer->initMultiThreading(); // required for >Eigen 3.1
  
  return optimizer;
}


bool TebOptimalPlanner::optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards,
                                    double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{ // look for code 
  if (cfg_->optim.optimization_activate==false) 
    return false;
  
  bool success = false;
  optimized_ = false;
  
  double weight_multiplier = 1.0;

  // TODO(roesmann): we introduced the non-fast mode with the support of dynamic obstacles
  //                (which leads to better results in terms of x-y-t homotopy planning).
  //                 however, we have not tested this mode intensively yet, so we keep
  //                 the legacy fast mode as default until we finish our tests.
  bool fast_mode = !cfg_->obstacles.include_dynamic_obstacles;
  
  for(int i=0; i<iterations_outerloop; ++i)
  {
    if (cfg_->trajectory.teb_autosize)
    {
      ROS_DEBUG("AUTO RESIZE");
      //teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples);
      teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples, fast_mode);

    }

    success = buildGraph(weight_multiplier);
    ROS_DEBUG("SUCCESSFULLY BUILD GRAPH");
    if (!success) 
    {
        clearGraph();
        return false;
    }
    success = optimizeGraph(iterations_innerloop, false);
    ROS_DEBUG("SUCCESSFULLY OPTIMIZE GRAPH");
    if (!success) 
    {
        clearGraph();
        return false;
    }
    optimized_ = true;
    
    if (compute_cost_afterwards && i==iterations_outerloop-1) // compute cost vec only in the last iteration
    {
        computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost);
        ROS_INFO_STREAM("Obstacle cost: " << obst_cost_scale << ", Via-point cost: " << viapoint_cost_scale);
    }
  
     
    clearGraph();

    weight_multiplier *= cfg_->optim.weight_adapt_factor;
  }

  return true;
}

void TebOptimalPlanner::setVelocityStart(const geometry_msgs::Twist& vel_start)
{
  vel_start_.first = true;
  vel_start_.second.linear.x = vel_start.linear.x;
  vel_start_.second.linear.y = vel_start.linear.y;
  vel_start_.second.angular.z = vel_start.angular.z;
}

void TebOptimalPlanner::setVelocityGoal(const geometry_msgs::Twist& vel_goal)
{
  vel_goal_.first = true;
  vel_goal_.second = vel_goal;
}

bool TebOptimalPlanner::plan(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec, const std::vector<geometry_msgs::PoseStamped>& initial_plan, double inscribed_radius, double circumscribed_radius, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{    
  ROS_DEBUG("TEB plan with initial_plan 1 ");

  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");

  if (!teb_.isInit())
  {
    ROS_DEBUG("initialize_trajectory_to_goal");
    teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.global_plan_overwrite_orientation,
      cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);

    ROS_DEBUG("finish initialize");
    
    int look_ahead_idx = cfg_->trajectory.feasibility_check_no_poses;

    if (look_ahead_idx < 0 || look_ahead_idx >= teb().sizePoses())
        look_ahead_idx = teb().sizePoses() - 1;
        
    for (int i=0; i <= look_ahead_idx; ++i)
    {           
        if (i<look_ahead_idx)
        {
            double delta_rot = g2o::normalize_theta(g2o::normalize_theta(teb().Pose(i+1).theta()) - g2o::normalize_theta(teb().Pose(i).theta()));

            Eigen::Vector2d delta_dist = teb().Pose(i+1).position()-teb().Pose(i).position();

            if(fabs(delta_rot) > cfg_->trajectory.min_resolution_collision_check_angular || delta_dist.norm() > inscribed_radius)
            {
                int n_additional_samples = std::max(std::ceil(fabs(delta_rot) / cfg_->trajectory.min_resolution_collision_check_angular), std::ceil(delta_dist.norm() / inscribed_radius)) - 1;

                ROS_DEBUG("initial planning -> additional_samples number : %d", n_additional_samples);
                PoseSE2 intermediate_pose = teb().Pose(i);
                ROS_DEBUG("pose %d turn into intermediate_pose 0", i);

                std::vector<PoseSE2> intermediate_poses; // 중간 포즈를 저장할 임시 리스트

                for (int step = 0; step < n_additional_samples; ++step)
                {
                  intermediate_pose.position() = intermediate_pose.position() + delta_dist / (n_additional_samples + 1.0);
                  intermediate_pose.theta() = g2o::normalize_theta(intermediate_pose.theta() +
                                                                 delta_rot / (n_additional_samples + 1.0));
                  double intermediate_cost = costmap_model->footprintCost(intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta(),
                                                                          footprint_spec, inscribed_radius, circumscribed_radius);
                
                  visualization_->visualizeIntermediatePoint(intermediate_pose); 
                  intermediate_poses.push_back(intermediate_pose);

                  if (intermediate_cost == -1)
                  {
                    visualization_->publishInfeasibleRobotPose(intermediate_pose, *cfg_->robot_model, footprint_spec);
                  }
                }

                ROS_DEBUG("teb size : %d", teb().sizePoses());
                ROS_DEBUG("intermediate poses size : %d", intermediate_poses.size());

                for(int g = 0; g < teb().sizePoses(); g++)
                {
                  ROS_INFO("pose %d,  x : %f, y : %f", g, teb().Pose(g).x(), teb().Pose(g).y());
                }

                // Insert the new pose and time difference at the position
                for (int k = 0; k < intermediate_poses.size(); k++)
                {
                  teb().insertPose(i + k + 1, intermediate_poses[k]);
                  teb().insertTimeDiff(i + k + 1, cfg_->trajectory.dt_ref);
                }

                for(int g = 0; g < teb().sizePoses(); g++)
                {
                  ROS_INFO("pose %d,  x : %f, y : %f", g, teb().Pose(g).x(), teb().Pose(g).y());
                  double pose_cost = costmap_model->footprintCost(teb().Pose(g).x(), teb().Pose(g).y(), teb().Pose(g).theta(),footprint_spec, inscribed_radius, circumscribed_radius);
                  ROS_INFO("Pose cost : %lf", pose_cost);
                }
            
                ROS_DEBUG("FINISH INSERTING NEW POSES");

                ROS_DEBUG("teb size : %d", teb_.sizePoses());
                
                // Implement optimization part when adding intermediate pose 
                for(int i = 0; i < intermediate_poses.size(); i++)
                {
                    ROS_INFO("Footprint position x : %f, y : %f", intermediate_poses[i].x(), intermediate_poses[i].y());
                }
            
                //std::cout << "Press Enter to continue..." << std::endl;
                //std::cin.get();

                ROS_INFO("optimizeTEB with intermediate pose"); 

                optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
              
                for(int g = 0; g < teb().sizePoses(); g++)
                {
                  ROS_INFO("pose %d,  x : %f, y : %f", g, teb().Pose(g).x(), teb().Pose(g).y());
                  double pose_cost = costmap_model->footprintCost(teb().Pose(g).x(), teb().Pose(g).y(), teb().Pose(g).theta(),
                                                                        footprint_spec, inscribed_radius, circumscribed_radius);
                  ROS_INFO("Pose cost : %lf", pose_cost);
                }

                //std::cout << "Press Enter to continue..." << std::endl;
                //std::cin.get();
            }
        }
    }
  }
  else // warm start
  {
    ROS_DEBUG("Warm starting");
    PoseSE2 start_(initial_plan.front().pose);
    PoseSE2 goal_(initial_plan.back().pose);
    
    if (teb_.sizePoses()>0
        && (goal_.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist
        && fabs(g2o::normalize_theta(goal_.theta() - teb_.BackPose().theta())) < cfg_->trajectory.force_reinit_new_goal_angular) // actual warm start!
    {
        teb_.updateAndPruneTEB(start_, goal_, cfg_->trajectory.min_samples); // update TEB

      int look_ahead_idx = cfg_->trajectory.feasibility_check_no_poses;

      if (look_ahead_idx < 0 || look_ahead_idx >= teb().sizePoses())
        look_ahead_idx = teb().sizePoses() - 1;

      for (int i=0; i <= look_ahead_idx; ++i)
      {           
          if (i<look_ahead_idx)
          {
            double delta_rot = g2o::normalize_theta(g2o::normalize_theta(teb().Pose(i+1).theta()) - g2o::normalize_theta(teb().Pose(i).theta()));

            Eigen::Vector2d delta_dist = teb().Pose(i+1).position()-teb().Pose(i).position();

            if(fabs(delta_rot) > cfg_->trajectory.min_resolution_collision_check_angular || delta_dist.norm() > inscribed_radius)
            {
                int n_additional_samples = std::max(std::ceil(fabs(delta_rot) / cfg_->trajectory.min_resolution_collision_check_angular), std::ceil(delta_dist.norm() / inscribed_radius)) - 1;

                ROS_DEBUG("initial planning -> additional_samples number : %d", n_additional_samples);
                PoseSE2 intermediate_pose = teb().Pose(i);
                ROS_DEBUG("pose %d turn into intermediate_pose 0", i);

                std::vector<PoseSE2> intermediate_poses; // 중간 포즈를 저장할 임시 리스트

                for (int step = 0; step < n_additional_samples; ++step)
                {
                  intermediate_pose.position() = intermediate_pose.position() + delta_dist / (n_additional_samples + 1.0);
                  intermediate_pose.theta() = g2o::normalize_theta(intermediate_pose.theta() +
                                                                 delta_rot / (n_additional_samples + 1.0));
                  double intermediate_cost = costmap_model->footprintCost(intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta(),
                                                                          footprint_spec, inscribed_radius, circumscribed_radius);
                
                  visualization_->visualizeIntermediatePoint(intermediate_pose); 
                  intermediate_poses.push_back(intermediate_pose);

                  if (intermediate_cost == -1)
                  {
                    visualization_->publishInfeasibleRobotPose(intermediate_pose, *cfg_->robot_model, footprint_spec);
                  }
                }

                ROS_DEBUG("teb size : %d", teb().sizePoses());
                ROS_DEBUG("intermediate poses size : %d", intermediate_poses.size());

                for(int g = 0; g < teb().sizePoses(); g++)
                {
                  ROS_INFO("pose %d,  x : %f, y : %f", g, teb().Pose(g).x(), teb().Pose(g).y());
                }

                // Insert the new pose and time difference at the position
                for (int k = 0; k < intermediate_poses.size(); k++)
                {
                  teb().insertPose(i + k + 1, intermediate_poses[k]);
                  teb().insertTimeDiff(i + k + 1, cfg_->trajectory.dt_ref);
                }

                for(int g = 0; g < teb().sizePoses(); g++)
                {
                  ROS_INFO("pose %d,  x : %f, y : %f", g, teb().Pose(g).x(), teb().Pose(g).y());
                  double pose_cost = costmap_model->footprintCost(teb().Pose(g).x(), teb().Pose(g).y(), teb().Pose(g).theta(),footprint_spec, inscribed_radius, circumscribed_radius);

                  ROS_INFO("Pose cost : %lf", pose_cost);
                }
            
                ROS_DEBUG("FINISH INSERTING NEW POSES");

                ROS_DEBUG("teb size : %d", teb_.sizePoses());
                
                // Implement optimization part when adding intermediate pose 
                for(int i = 0; i < intermediate_poses.size(); i++)
                {
                    ROS_INFO("Footprint position x : %f, y : %f", intermediate_poses[i].x(), intermediate_poses[i].y());
                }
            
                //std::cout << "Press Enter to continue..." << std::endl;
                //std::cin.get();

                ROS_INFO("optimizeTEB with intermediate pose"); 

                optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
              
                for(int g = 0; g < teb().sizePoses(); g++)
                {
                  ROS_INFO("pose %d,  x : %f, y : %f", g, teb().Pose(g).x(), teb().Pose(g).y());
                  double pose_cost = costmap_model->footprintCost(teb().Pose(g).x(), teb().Pose(g).y(), teb().Pose(g).theta(),
                                                                        footprint_spec, inscribed_radius, circumscribed_radius);
                  ROS_INFO("Pose cost : %lf", pose_cost);
                }

                //std::cout << "Press Enter to continue..." << std::endl;
                //std::cin.get();
            }
          }
      }
        
    }
    
    else // goal too far away -> reinit
    { 
      ROS_DEBUG("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");

      teb_.clearTimedElasticBand();
      teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.global_plan_overwrite_orientation,
        cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
      
      int look_ahead_idx = cfg_->trajectory.feasibility_check_no_poses;

      if (look_ahead_idx < 0 || look_ahead_idx >= teb().sizePoses())
          look_ahead_idx = teb().sizePoses() - 1;
        
      for (int i=0; i <= look_ahead_idx; ++i)
      {           
          if (i<look_ahead_idx)
          {
            double delta_rot = g2o::normalize_theta(g2o::normalize_theta(teb().Pose(i+1).theta()) - g2o::normalize_theta(teb().Pose(i).theta()));

            Eigen::Vector2d delta_dist = teb().Pose(i+1).position()-teb().Pose(i).position();

            if(fabs(delta_rot) > cfg_->trajectory.min_resolution_collision_check_angular || delta_dist.norm() > inscribed_radius)
            {
                int n_additional_samples = std::max(std::ceil(fabs(delta_rot) / cfg_->trajectory.min_resolution_collision_check_angular), std::ceil(delta_dist.norm() / inscribed_radius)) - 1;

                ROS_DEBUG("initial planning -> additional_samples number : %d", n_additional_samples);
                PoseSE2 intermediate_pose = teb().Pose(i);
                ROS_DEBUG("pose %d turn into intermediate_pose 0", i);
            }
          }
      }

    }
  }
  if (start_vel)
    setVelocityStart(*start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)
  
  ROS_DEBUG("TEB optimization start");
  // now optimize
  return optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
}

bool TebOptimalPlanner::plan(const std::vector<geometry_msgs::PoseStamped>& initial_plan, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{    
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  if (!teb_.isInit())
  {
    teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.global_plan_overwrite_orientation,
      cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
  }
  else // warm start
  {
    PoseSE2 start_(initial_plan.front().pose);
    PoseSE2 goal_(initial_plan.back().pose);
    if (teb_.sizePoses()>0
        && (goal_.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist
        && fabs(g2o::normalize_theta(goal_.theta() - teb_.BackPose().theta())) < cfg_->trajectory.force_reinit_new_goal_angular) // actual warm start!
      teb_.updateAndPruneTEB(start_, goal_, cfg_->trajectory.min_samples); // update TEB
    else // goal too far away -> reinit
    {
      ROS_DEBUG("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      teb_.clearTimedElasticBand();
      teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.global_plan_overwrite_orientation,
        cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
    }
  }
  if (start_vel)
    setVelocityStart(*start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)
  
  // now optimize
  return optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
}


bool TebOptimalPlanner::plan(const tf::Pose& start, const tf::Pose& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{
  ROS_DEBUG("TEB plan with initial_plan 2 ");
  PoseSE2 start_(start);
  PoseSE2 goal_(goal);
  return plan(start_, goal_, start_vel);
}

bool TebOptimalPlanner::plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{	
  ROS_DEBUG("TEB plan with initial_plan 3 ");
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  if (!teb_.isInit())
  {
    // init trajectory
    teb_.initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion); // 0 intermediate samples, but dt=1 -> autoResize will add more samples before calling first optimization
  }
  else // warm start
  {
    if (teb_.sizePoses() > 0
        && (goal.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist
        && fabs(g2o::normalize_theta(goal.theta() - teb_.BackPose().theta())) < cfg_->trajectory.force_reinit_new_goal_angular) // actual warm start!
      teb_.updateAndPruneTEB(start, goal, cfg_->trajectory.min_samples);
    else // goal too far away -> reinit
    {
      ROS_DEBUG("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      teb_.clearTimedElasticBand();
      teb_.initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
    }
  }
  if (start_vel)
    setVelocityStart(*start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)
      
  // now optimize
  return optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
}


bool TebOptimalPlanner::buildGraph(double weight_multiplier)
{
  ROS_DEBUG("Build Graph");
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
  {
    ROS_WARN("Cannot build graph, because it is not empty. Call graphClear()!");
    return false;
  }

  optimizer_->setComputeBatchStatistics(cfg_->recovery.divergence_detection_enable);

  if (is_reoptimization_active) 
  { // 재최적화가 활성화된 경우 
    AddTEBVertices(redundant_indices.front() - 1, redundant_indices.back()); 
  }

  else 
  { // 전체 TEB 포즈 추가 
    AddTEBVertices(); 
  }  
  
  // add Edges (local cost functions)
  if (cfg_->obstacles.legacy_obstacle_association)
    AddEdgesObstaclesLegacy(weight_multiplier);
  else
  {
    AddEdgesObstacles(weight_multiplier);
    ROS_DEBUG("SUCCESSFULLY ADD EDGES OBSTACLES");
  }
  
  if (cfg_->obstacles.include_dynamic_obstacles)
    AddEdgesDynamicObstacles();
  
  AddEdgesViaPoints();
  
  AddEdgesVelocity();
  
  AddEdgesAcceleration();

  AddEdgesTimeOptimal();	

  AddEdgesShortestPath();
  
  if (cfg_->robot.min_turning_radius == 0 || cfg_->optim.weight_kinematics_turning_radius == 0)
    AddEdgesKinematicsDiffDrive(); // we have a differential drive robot
  else
    AddEdgesKinematicsCarlike(); // we have a carlike robot since the turning radius is bounded from below.

  AddEdgesPreferRotDir();

  if (cfg_->optim.weight_velocity_obstacle_ratio > 0)
    AddEdgesVelocityObstacleRatio();

  // 엣지 비용 출력
  for (const auto& edge : optimizer_->activeEdges()) {
    double edge_cost = edge->chi2(); // 현재 엣지의 비용 계산
    auto* edge_obstacle = dynamic_cast<EdgeObstacle*>(edge);
    auto* edge_viapoint = dynamic_cast<EdgeViaPoint*>(edge);
    auto* edge_velocity = dynamic_cast<EdgeVelocity*>(edge);
    auto* edge_acceleration = dynamic_cast<EdgeAcceleration*>(edge);
    auto* edge_timeoptimal = dynamic_cast<EdgeTimeOptimal*>(edge);
    auto* edge_shortestpath = dynamic_cast<EdgeShortestPath*>(edge);

    // 각 엣지 유형에 따라 로그 출력
    if (edge_obstacle) {
        ROS_INFO_STREAM("Obstacle Edge cost: " << edge_cost);
    } else if (edge_viapoint) {
        ROS_INFO_STREAM("Via Point Edge cost: " << edge_cost);
    } else if (edge_velocity) {
        ROS_INFO_STREAM("Velocity Edge cost: " << edge_cost);
    } else if (edge_acceleration) {
        ROS_INFO_STREAM("Acceleration Edge cost: " << edge_cost);
    } else if (edge_timeoptimal) {
        ROS_INFO_STREAM("Time Optimal Edge cost: " << edge_cost);
    } else if (edge_shortestpath) {
        ROS_INFO_STREAM("Shortest Path Edge cost: " << edge_cost);
    }
}

    
  return true;  
}

bool TebOptimalPlanner::optimizeGraph(int no_iterations,bool clear_after)
{
  if (cfg_->robot.max_vel_x<0.01)
  {
    ROS_WARN("optimizeGraph(): Robot Max Velocity is smaller than 0.01m/s. Optimizing aborted...");
    if (clear_after) clearGraph();
    return false;	
  }
  
  if (!teb_.isInit() || teb_.sizePoses() < cfg_->trajectory.min_samples)
  {
    ROS_WARN("optimizeGraph(): TEB is empty or has too less elements. Skipping optimization.");
    if (clear_after) clearGraph();
    return false;	
  }
  
  optimizer_->setVerbose(cfg_->optim.optimization_verbose);
  optimizer_->initializeOptimization();

  int iter = optimizer_->optimize(no_iterations);

  // Save Hessian for visualization
  //  g2o::OptimizationAlgorithmLevenberg* lm = dynamic_cast<g2o::OptimizationAlgorithmLevenberg*> (optimizer_->solver());
  //  lm->solver()->saveHessian("~/MasterThesis/Matlab/Hessian.txt");

  if(!iter)
  {
	ROS_ERROR("optimizeGraph(): Optimization failed! iter=%i", iter);
	return false;
  }

  if (clear_after) clearGraph();	
    
  return true;
}

void TebOptimalPlanner::clearGraph()
{
  // clear optimizer states
  if (optimizer_)
  {
    // we will delete all edges but keep the vertices.
    // before doing so, we will delete the link from the vertices to the edges.
    auto& vertices = optimizer_->vertices();
    for(auto& v : vertices)
      v.second->edges().clear();

    optimizer_->vertices().clear();  // necessary, because optimizer->clear deletes pointer-targets (therefore it deletes TEB states!)
    optimizer_->clear();
  }
}

void TebOptimalPlanner::AddTEBVertices()
{
  // add vertices to graph
  ROS_DEBUG_COND(cfg_->optim.optimization_verbose, "Adding TEB vertices ...");
  unsigned int id_counter = 0; // used for vertices ids
  obstacles_per_vertex_.resize(teb_.sizePoses());
  auto iter_obstacle = obstacles_per_vertex_.begin();

  ROS_DEBUG("TEB VERTEX SIZE : %d", teb_.sizePoses());

  for (int i=0; i<teb_.sizePoses(); ++i)
  {
    teb_.PoseVertex(i)->setId(id_counter++);
    optimizer_->addVertex(teb_.PoseVertex(i));
    if (teb_.sizeTimeDiffs()!=0 && i<teb_.sizeTimeDiffs())
    {
      teb_.TimeDiffVertex(i)->setId(id_counter++);
      optimizer_->addVertex(teb_.TimeDiffVertex(i));
    }
    iter_obstacle->clear();
    (iter_obstacle++)->reserve(obstacles_->size());
  }
}

void TebOptimalPlanner::AddTEBVertices(int start_idx, int end_idx)
{
    // 그래프에 vertex 추가
    ROS_DEBUG_COND(cfg_->optim.optimization_verbose, "Adding limited TEB vertices ...");
    unsigned int id_counter = 0; // vertex ID 카운터
    obstacles_per_vertex_.resize(end_idx - start_idx + 1); // 해당 구간 크기로 조정
    auto iter_obstacle = obstacles_per_vertex_.begin();
   
    ROS_DEBUG("TEB VERTEX SIZE : %d", end_idx - start_idx + 1);
   
    for (int i = start_idx; i <= end_idx; ++i)
    {
        teb_.PoseVertex(i)->setId(id_counter++);
        optimizer_->addVertex(teb_.PoseVertex(i));
       
        if (teb_.sizeTimeDiffs() != 0 && i < teb_.sizeTimeDiffs())
        {
            teb_.TimeDiffVertex(i)->setId(id_counter++);
            optimizer_->addVertex(teb_.TimeDiffVertex(i));
        }
       
        iter_obstacle->clear();
        (iter_obstacle++)->reserve(obstacles_->size());
    }
}



void TebOptimalPlanner::AddEdgesObstacles(double weight_multiplier)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==nullptr )
    return; // if weight equals zero skip adding edges!
    
  
  bool inflated = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);
  
  Eigen::Matrix<double,2,2> information_inflated;
  information_inflated(0,0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1,1) = cfg_->optim.weight_inflation;
  information_inflated(0,1) = information_inflated(1,0) = 0;

  // Convert the matrices to strings
  std::stringstream ss_info;
  ss_info << information;
  std::stringstream ss_info_inflated;
  ss_info_inflated << information_inflated;

  // Print the matrices using ROS_DEBUG
  ROS_DEBUG("Information Matrix:\n%s", ss_info.str().c_str());
  ROS_DEBUG("Information Inflated Matrix:\n%s", ss_info_inflated.str().c_str());


  auto iter_obstacle = obstacles_per_vertex_.begin();

  auto create_edge = [inflated, &information, &information_inflated, this] (int index, const Obstacle* obstacle) {
    if (inflated)
    {
      EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
      dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
      dist_bandpt_obst->setInformation(information_inflated);
      dist_bandpt_obst->setParameters(*cfg_, obstacle);
      optimizer_->addEdge(dist_bandpt_obst);
    }
    else
    {
      EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
      dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
      dist_bandpt_obst->setInformation(information);
      dist_bandpt_obst->setParameters(*cfg_, obstacle);
      optimizer_->addEdge(dist_bandpt_obst);
    };
  };
    
  // iterate all teb points, skipping the last and, if the EdgeVelocityObstacleRatio edges should not be created, the first one too
  const int first_vertex = cfg_->optim.weight_velocity_obstacle_ratio == 0 ? 1 : 0;
  for (int i = first_vertex; i < teb_.sizePoses() - 1; ++i)
  {  
      double left_min_dist = std::numeric_limits<double>::max();
      double right_min_dist = std::numeric_limits<double>::max();
      ObstaclePtr left_obstacle;
      ObstaclePtr right_obstacle;
      
      const Eigen::Vector2d pose_orient = teb_.Pose(i).orientationUnitVec();
      
      // iterate obstacles
      for (const ObstaclePtr& obst : *obstacles_)
      {
        // we handle dynamic obstacles differently below
        if(cfg_->obstacles.include_dynamic_obstacles && obst->isDynamic())
          continue;

          // calculate distance to robot model
          double dist = cfg_->robot_model->calculateDistance(teb_.Pose(i), obst.get());
          
          // force considering obstacle if really close to the current pose
        if (dist < cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_force_inclusion_factor)
          {
              iter_obstacle->push_back(obst);
              continue;
          }
          // cut-off distance
          if (dist > cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_cutoff_factor)
            continue;
          
          // determine side (left or right) and assign obstacle if closer than the previous one
          if (cross2d(pose_orient, obst->getCentroid() - teb_.Pose(i).position()) > 0) // left
          {
              if (dist < left_min_dist)
              {
                  left_min_dist = dist;
                  left_obstacle = obst;
              }
          }
          else
          {
              if (dist < right_min_dist)
              {
                  right_min_dist = dist;
                  right_obstacle = obst;
              }
          }
      }   
      
      if (left_obstacle)
      {
        iter_obstacle->push_back(left_obstacle);
      }
      if (right_obstacle)
      {
        iter_obstacle->push_back(right_obstacle);
      }
      visualization_->visualizeObstacle(teb_.Pose(i), left_obstacle, right_obstacle);
      //std::cout << "Press Enter to continue..." << std::endl;
      //std::cin.get();

      // continue here to ignore obstacles for the first pose, but use them later to create the EdgeVelocityObstacleRatio edges
      if (i == 0)
      {
        ++iter_obstacle;
        continue;
      }

      // create obstacle edges
      for (const ObstaclePtr obst : *iter_obstacle)
        create_edge(i, obst.get());
      ++iter_obstacle;
  }
}


void TebOptimalPlanner::AddEdgesObstaclesLegacy(double weight_multiplier)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==nullptr)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information; 
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);
    
  Eigen::Matrix<double,2,2> information_inflated;
  information_inflated(0,0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1,1) = cfg_->optim.weight_inflation;
  information_inflated(0,1) = information_inflated(1,0) = 0;
  
  bool inflated = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;
    
  for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    if (cfg_->obstacles.include_dynamic_obstacles && (*obst)->isDynamic()) // we handle dynamic obstacles differently below
      continue; 
    
    int index;
    
    if (cfg_->obstacles.obstacle_poses_affected >= teb_.sizePoses())
      index =  teb_.sizePoses() / 2;
    else
      index = teb_.findClosestTrajectoryPose(*(obst->get()));
     
    
    // check if obstacle is outside index-range between start and goal
    if ( (index <= 1) || (index > teb_.sizePoses()-2) ) // start and goal are fixed and findNearestBandpoint finds first or last conf if intersection point is outside the range
	    continue; 
        
    if (inflated)
    {
        EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
        dist_bandpt_obst->setInformation(information_inflated);
        dist_bandpt_obst->setParameters(*cfg_, obst->get());
        optimizer_->addEdge(dist_bandpt_obst);
    }
    else
    {
        EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(*cfg_, obst->get());
        optimizer_->addEdge(dist_bandpt_obst);
    }

    for (int neighbourIdx=0; neighbourIdx < floor(cfg_->obstacles.obstacle_poses_affected/2); neighbourIdx++)
    {
      if (index+neighbourIdx < teb_.sizePoses())
      {
            if (inflated)
            {
                EdgeInflatedObstacle* dist_bandpt_obst_n_r = new EdgeInflatedObstacle;
                dist_bandpt_obst_n_r->setVertex(0,teb_.PoseVertex(index+neighbourIdx));
                dist_bandpt_obst_n_r->setInformation(information_inflated);
                dist_bandpt_obst_n_r->setParameters(*cfg_, obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_r);
            }
            else
            {
                EdgeObstacle* dist_bandpt_obst_n_r = new EdgeObstacle;
                dist_bandpt_obst_n_r->setVertex(0,teb_.PoseVertex(index+neighbourIdx));
                dist_bandpt_obst_n_r->setInformation(information);
                dist_bandpt_obst_n_r->setParameters(*cfg_, obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_r);
            }
      }
      if ( index - neighbourIdx >= 0) // needs to be casted to int to allow negative values
      {
            if (inflated)
            {
                EdgeInflatedObstacle* dist_bandpt_obst_n_l = new EdgeInflatedObstacle;
                dist_bandpt_obst_n_l->setVertex(0,teb_.PoseVertex(index-neighbourIdx));
                dist_bandpt_obst_n_l->setInformation(information_inflated);
                dist_bandpt_obst_n_l->setParameters(*cfg_, obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_l);
            }
            else
            {
                EdgeObstacle* dist_bandpt_obst_n_l = new EdgeObstacle;
                dist_bandpt_obst_n_l->setVertex(0,teb_.PoseVertex(index-neighbourIdx));
                dist_bandpt_obst_n_l->setInformation(information);
                dist_bandpt_obst_n_l->setParameters(*cfg_, obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_l);
            }
      }
    } 
    
  }
}


void TebOptimalPlanner::AddEdgesDynamicObstacles(double weight_multiplier)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==NULL )
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,2,2> information;
  information(0,0) = cfg_->optim.weight_dynamic_obstacle * weight_multiplier;
  information(1,1) = cfg_->optim.weight_dynamic_obstacle_inflation;
  information(0,1) = information(1,0) = 0;
  
  for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    if (!(*obst)->isDynamic())
      continue;

    // Skip first and last pose, as they are fixed
    double time = teb_.TimeDiff(0);
    for (int i=1; i < teb_.sizePoses() - 1; ++i)
    {
      EdgeDynamicObstacle* dynobst_edge = new EdgeDynamicObstacle(time);
      dynobst_edge->setVertex(0,teb_.PoseVertex(i));
      dynobst_edge->setInformation(information);
      dynobst_edge->setParameters(*cfg_, obst->get());
      optimizer_->addEdge(dynobst_edge);
      time += teb_.TimeDiff(i); // we do not need to check the time diff bounds, since we iterate to "< sizePoses()-1".
    }
  }
}

void TebOptimalPlanner::AddEdgesViaPoints()
{
  ROS_DEBUG("ADD EDGES VIA POINTS");
  if (cfg_->optim.weight_viapoint==0 || via_points_==NULL || via_points_->empty() )
    return; // if weight equals zero skip adding edges!

  int start_pose_idx = 0;
  ROS_DEBUG("START ADDING VIA POINTS");
  int n = teb_.sizePoses();
  if (n<3) // we do not have any degrees of freedom for reaching via-points
    return;
  
  for (ViaPointContainer::const_iterator vp_it = via_points_->begin(); vp_it != via_points_->end(); ++vp_it)
  {
    int index = teb_.findClosestTrajectoryPose(*vp_it, NULL, start_pose_idx);
    ROS_INFO("index : %d", index);
    if (cfg_->trajectory.via_points_ordered)
      start_pose_idx = index+2; // skip a point to have a DOF inbetween for further via-points
     
    // check if point conicides with goal or is located behind it
    if ( index > n-2 ) 
      index = n-2; // set to a pose before the goal, since we can move it away!
    // check if point coincides with start or is located before it
    if ( index < 1)
    {
      if (cfg_->trajectory.via_points_ordered)
      {
        index = 1; // try to connect the via point with the second (and non-fixed) pose. It is likely that autoresize adds new poses inbetween later.
      }
      else
      {
        ROS_DEBUG("TebOptimalPlanner::AddEdgesViaPoints(): skipping a via-point that is close or behind the current robot pose.");
        continue; // skip via points really close or behind the current robot pose
      }
    }
    Eigen::Matrix<double,1,1> information;
    information.fill(cfg_->optim.weight_viapoint);
    
    EdgeViaPoint* edge_viapoint = new EdgeViaPoint;
    edge_viapoint->setVertex(0,teb_.PoseVertex(index));
    edge_viapoint->setInformation(information);
    edge_viapoint->setParameters(*cfg_, &(*vp_it));
    optimizer_->addEdge(edge_viapoint);   
  }
}

void TebOptimalPlanner::AddEdgesVelocity()
{
  if (cfg_->robot.max_vel_y == 0) // non-holonomic robot
  {
    if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_theta==0)
      return; // if weight equals zero skip adding edges!

    int n = teb_.sizePoses();
    Eigen::Matrix<double,2,2> information;
    information(0,0) = cfg_->optim.weight_max_vel_x;
    information(1,1) = cfg_->optim.weight_max_vel_theta;
    information(0,1) = 0.0;
    information(1,0) = 0.0;

    for (int i=0; i < n - 1; ++i)
    {
      EdgeVelocity* velocity_edge = new EdgeVelocity;
      velocity_edge->setVertex(0,teb_.PoseVertex(i));
      velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
      velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    }
  }
  else // holonomic-robot
  {
    if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_y==0 && cfg_->optim.weight_max_vel_theta==0)
      return; // if weight equals zero skip adding edges!
      
    int n = teb_.sizePoses();
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_max_vel_x;
    information(1,1) = cfg_->optim.weight_max_vel_y;
    information(2,2) = cfg_->optim.weight_max_vel_theta;

    for (int i=0; i < n - 1; ++i)
    {
      EdgeVelocityHolonomic* velocity_edge = new EdgeVelocityHolonomic;
      velocity_edge->setVertex(0,teb_.PoseVertex(i));
      velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
      velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    } 
  }
}

void TebOptimalPlanner::AddEdgesAcceleration()
{
  if (cfg_->optim.weight_acc_lim_x==0  && cfg_->optim.weight_acc_lim_theta==0) 
    return; // if weight equals zero skip adding edges!

  int n = teb_.sizePoses();  
    
  if (cfg_->robot.max_vel_y == 0 || cfg_->robot.acc_lim_y == 0) // non-holonomic robot
  {
    Eigen::Matrix<double,2,2> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_acc_lim_x;
    information(1,1) = cfg_->optim.weight_acc_lim_theta;
    
    // check if an initial velocity should be taken into accound
    if (vel_start_.first)
    {
      EdgeAccelerationStart* acceleration_edge = new EdgeAccelerationStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      EdgeAcceleration* acceleration_edge = new EdgeAcceleration;
      acceleration_edge->setVertex(0,teb_.PoseVertex(i));
      acceleration_edge->setVertex(1,teb_.PoseVertex(i+1));
      acceleration_edge->setVertex(2,teb_.PoseVertex(i+2));
      acceleration_edge->setVertex(3,teb_.TimeDiffVertex(i));
      acceleration_edge->setVertex(4,teb_.TimeDiffVertex(i+1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
    
    // check if a goal velocity should be taken into accound
    if (vel_goal_.first)
    {
      EdgeAccelerationGoal* acceleration_edge = new EdgeAccelerationGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(n-2));
      acceleration_edge->setVertex(1,teb_.PoseVertex(n-1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex( teb_.sizeTimeDiffs()-1 ));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }  
  }
  else // holonomic robot
  {
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_acc_lim_x;
    information(1,1) = cfg_->optim.weight_acc_lim_y;
    information(2,2) = cfg_->optim.weight_acc_lim_theta;
    
    // check if an initial velocity should be taken into accound
    if (vel_start_.first)
    {
      EdgeAccelerationHolonomicStart* acceleration_edge = new EdgeAccelerationHolonomicStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      EdgeAccelerationHolonomic* acceleration_edge = new EdgeAccelerationHolonomic;
      acceleration_edge->setVertex(0,teb_.PoseVertex(i));
      acceleration_edge->setVertex(1,teb_.PoseVertex(i+1));
      acceleration_edge->setVertex(2,teb_.PoseVertex(i+2));
      acceleration_edge->setVertex(3,teb_.TimeDiffVertex(i));
      acceleration_edge->setVertex(4,teb_.TimeDiffVertex(i+1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
    
    // check if a goal velocity should be taken into accound
    if (vel_goal_.first)
    {
      EdgeAccelerationHolonomicGoal* acceleration_edge = new EdgeAccelerationHolonomicGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(n-2));
      acceleration_edge->setVertex(1,teb_.PoseVertex(n-1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex( teb_.sizeTimeDiffs()-1 ));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }  
  }
}



void TebOptimalPlanner::AddEdgesTimeOptimal()
{
  if (cfg_->optim.weight_optimaltime==0) 
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_optimaltime);

  for (int i=0; i < teb_.sizeTimeDiffs(); ++i)
  {
    EdgeTimeOptimal* timeoptimal_edge = new EdgeTimeOptimal;
    timeoptimal_edge->setVertex(0,teb_.TimeDiffVertex(i));
    timeoptimal_edge->setInformation(information);
    timeoptimal_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(timeoptimal_edge);
  }
}

void TebOptimalPlanner::AddEdgesShortestPath()
{
  if (cfg_->optim.weight_shortest_path==0)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_shortest_path);

  for (int i=0; i < teb_.sizePoses()-1; ++i)
  {
    EdgeShortestPath* shortest_path_edge = new EdgeShortestPath;
    shortest_path_edge->setVertex(0,teb_.PoseVertex(i));
    shortest_path_edge->setVertex(1,teb_.PoseVertex(i+1));
    shortest_path_edge->setInformation(information);
    shortest_path_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(shortest_path_edge);
  }
}



void TebOptimalPlanner::AddEdgesKinematicsDiffDrive()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_forward_drive==0)
    return; // if weight equals zero skip adding edges!
  
  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_forward_drive;
  
  for (int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    EdgeKinematicsDiffDrive* kinematics_edge = new EdgeKinematicsDiffDrive;
    kinematics_edge->setVertex(0,teb_.PoseVertex(i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));      
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }	 
}

void TebOptimalPlanner::AddEdgesKinematicsCarlike()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_turning_radius==0)
    return; // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_turning_radius;
  
  for (int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    EdgeKinematicsCarlike* kinematics_edge = new EdgeKinematicsCarlike;
    kinematics_edge->setVertex(0,teb_.PoseVertex(i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));      
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }  
}


void TebOptimalPlanner::AddEdgesPreferRotDir()
{
  //TODO(roesmann): Note, these edges can result in odd predictions, in particular
  //                we can observe a substantional mismatch between open- and closed-loop planning
  //                leading to a poor control performance.
  //                At the moment, we keep these functionality for oscillation recovery:
  //                Activating the edge for a short time period might not be crucial and
  //                could move the robot to a new oscillation-free state.
  //                This needs to be analyzed in more detail!
  if (prefer_rotdir_ == RotType::none || cfg_->optim.weight_prefer_rotdir==0)
    return; // if weight equals zero skip adding edges!

  if (prefer_rotdir_ != RotType::right && prefer_rotdir_ != RotType::left)
  {
    ROS_WARN("TebOptimalPlanner::AddEdgesPreferRotDir(): unsupported RotType selected. Skipping edge creation.");
    return;
  }

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,1,1> information_rotdir;
  information_rotdir.fill(cfg_->optim.weight_prefer_rotdir);
  
  for (int i=0; i < teb_.sizePoses()-1 && i < 3; ++i) // currently: apply to first 3 rotations
  {
    EdgePreferRotDir* rotdir_edge = new EdgePreferRotDir;
    rotdir_edge->setVertex(0,teb_.PoseVertex(i));
    rotdir_edge->setVertex(1,teb_.PoseVertex(i+1));      
    rotdir_edge->setInformation(information_rotdir);
    
    if (prefer_rotdir_ == RotType::left)
        rotdir_edge->preferLeft();
    else if (prefer_rotdir_ == RotType::right)
        rotdir_edge->preferRight();
    
    optimizer_->addEdge(rotdir_edge);
  }
}

void TebOptimalPlanner::AddEdgesVelocityObstacleRatio()
{
  Eigen::Matrix<double,2,2> information;
  information(0,0) = cfg_->optim.weight_velocity_obstacle_ratio;
  information(1,1) = cfg_->optim.weight_velocity_obstacle_ratio;
  information(0,1) = information(1,0) = 0;

  auto iter_obstacle = obstacles_per_vertex_.begin();

  for (int index = 0; index < teb_.sizePoses() - 1; ++index)
  {
    for (const ObstaclePtr obstacle : (*iter_obstacle++))
    {
      EdgeVelocityObstacleRatio* edge = new EdgeVelocityObstacleRatio;
      edge->setVertex(0,teb_.PoseVertex(index));
      edge->setVertex(1,teb_.PoseVertex(index + 1));
      edge->setVertex(2,teb_.TimeDiffVertex(index));
      edge->setInformation(information);
      edge->setParameters(*cfg_, obstacle.get());
      optimizer_->addEdge(edge);
    }
  }
}

bool TebOptimalPlanner::hasDiverged() const
{
  // Early returns if divergence detection is not active
  if (!cfg_->recovery.divergence_detection_enable)
    ROS_DEBUG("divergence detect unable.");
    return false;

  auto stats_vector = optimizer_->batchStatistics();

  // No statistics yet
  if (stats_vector.empty())
    ROS_DEBUG("no divergence statistics.");
    return false;

  // Grab the statistics of the final iteration
  const auto last_iter_stats = stats_vector.back();

  return last_iter_stats.chi2 > cfg_->recovery.divergence_detection_max_chi_squared;
}

void TebOptimalPlanner::computeCurrentCost(double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{ 
  // check if graph is empty/exist  -> important if function is called between buildGraph and optimizeGraph/clearGraph
  bool graph_exist_flag(false);
  if (optimizer_->edges().empty() && optimizer_->vertices().empty())
  {
    // here the graph is build again, for time efficiency make sure to call this function 
    // between buildGraph and Optimize (deleted), but it depends on the application
    buildGraph();	
    optimizer_->initializeOptimization();
  }
  else
  {
    graph_exist_flag = true;
  }
  //초기 추정값(initial guess)을 계산
  optimizer_->computeInitialGuess();
  
  cost_ = 0;

  if (alternative_time_cost)
  {
    cost_ += teb_.getSumOfAllTimeDiffs();
    // TEST we use SumOfAllTimeDiffs() here, because edge cost depends on number of samples, which is not always the same for similar TEBs,
    // since we are using an AutoResize Function with hysteresis.
  }
  
  // now we need pointers to all edges -> calculate error for each edge-type
  // since we aren't storing edge pointers, we need to check every edge
  for (std::vector<g2o::OptimizableGraph::Edge*>::const_iterator it = optimizer_->activeEdges().begin(); it!= optimizer_->activeEdges().end(); it++)
  {
    double cur_cost = (*it)->chi2();

    if (dynamic_cast<EdgeObstacle*>(*it) != nullptr
        || dynamic_cast<EdgeInflatedObstacle*>(*it) != nullptr
        || dynamic_cast<EdgeDynamicObstacle*>(*it) != nullptr)
    {
      cur_cost *= obst_cost_scale;
    }
    else if (dynamic_cast<EdgeViaPoint*>(*it) != nullptr)
    {
      cur_cost *= viapoint_cost_scale;
    }
    else if (dynamic_cast<EdgeTimeOptimal*>(*it) != nullptr && alternative_time_cost)
    {
      continue; // skip these edges if alternative_time_cost is active
    }
    cost_ += cur_cost;
  }

  ROS_DEBUG("cost : %d", cost_);

  // delete temporary created graph
  if (!graph_exist_flag) 
    clearGraph();
}

double TebOptimalPlanner::computeCostForPose(const PoseSE2& pose) {
  /*
    double cost = 0.0;

    // 장애물 비용
    double obstacle_cost = computeObstacleCost(pose.position());
    cost += obstacle_cost * cfg_->optim.obst_cost_scale;  // 스케일링 적용


    // 경유지점 비용
    double viapoint_cost = computeViaPointCost(pose.position());
    cost += viapoint_cost * cfg_->optim.viapoint_cost_scale;  // 스케일링 적용

    return cost;
   */
}


void TebOptimalPlanner::extractVelocity(const PoseSE2& pose1, const PoseSE2& pose2, double dt, double& vx, double& vy, double& omega) const
{
  if (dt == 0)
  {
    vx = 0;
    vy = 0;
    omega = 0;
    return;
  }
  
  Eigen::Vector2d deltaS = pose2.position() - pose1.position();
  
  if (cfg_->robot.max_vel_y == 0) // nonholonomic robot
  {
    Eigen::Vector2d conf1dir( cos(pose1.theta()), sin(pose1.theta()) );
    // translational velocity
    double dir = deltaS.dot(conf1dir);
    vx = (double) g2o::sign(dir) * deltaS.norm()/dt;
    vy = 0;
  }
  else // holonomic robot
  {
    // transform pose 2 into the current robot frame (pose1)
    // for velocities only the rotation of the direction vector is necessary.
    // (map->pose1-frame: inverse 2d rotation matrix)
    double cos_theta1 = std::cos(pose1.theta());
    double sin_theta1 = std::sin(pose1.theta());
    double p1_dx =  cos_theta1*deltaS.x() + sin_theta1*deltaS.y();
    double p1_dy = -sin_theta1*deltaS.x() + cos_theta1*deltaS.y();
    vx = p1_dx / dt;
    vy = p1_dy / dt;    
  }
  
  // rotational velocity
  double orientdiff = g2o::normalize_theta(pose2.theta() - pose1.theta());
  omega = orientdiff/dt;
}

bool TebOptimalPlanner::getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses) const
{
  if (teb_.sizePoses()<2)
  {
    ROS_ERROR("TebOptimalPlanner::getVelocityCommand(): The trajectory contains less than 2 poses. Make sure to init and optimize/plan the trajectory fist.");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }
  look_ahead_poses = std::max(1, std::min(look_ahead_poses, teb_.sizePoses() - 1 - cfg_->trajectory.prevent_look_ahead_poses_near_goal));
  double dt = 0.0;
  for(int counter = 0; counter < look_ahead_poses; ++counter)
  {
    dt += teb_.TimeDiff(counter);
    if(dt >= cfg_->trajectory.dt_ref * look_ahead_poses)  // TODO: change to look-ahead time? Refine trajectory?
    {
        look_ahead_poses = counter + 1;
        break;
    }
  }
  if (dt<=0)
  {	
    ROS_ERROR("TebOptimalPlanner::getVelocityCommand() - timediff<=0 is invalid!");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }
	  
  // Get velocity from the first two configurations
  extractVelocity(teb_.Pose(0), teb_.Pose(look_ahead_poses), dt, vx, vy, omega);
  return true;
}

void TebOptimalPlanner::getVelocityProfile(std::vector<geometry_msgs::Twist>& velocity_profile) const
{
  int n = teb_.sizePoses();
  velocity_profile.resize( n+1 );

  // start velocity 
  velocity_profile.front().linear.z = 0;
  velocity_profile.front().angular.x = velocity_profile.front().angular.y = 0;  
  velocity_profile.front().linear.x = vel_start_.second.linear.x;
  velocity_profile.front().linear.y = vel_start_.second.linear.y;
  velocity_profile.front().angular.z = vel_start_.second.angular.z;
  
  for (int i=1; i<n; ++i)
  {
    velocity_profile[i].linear.z = 0;
    velocity_profile[i].angular.x = velocity_profile[i].angular.y = 0;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), velocity_profile[i].linear.x, velocity_profile[i].linear.y, velocity_profile[i].angular.z);
  }
  
  // goal velocity
  velocity_profile.back().linear.z = 0;
  velocity_profile.back().angular.x = velocity_profile.back().angular.y = 0;  
  velocity_profile.back().linear.x = vel_goal_.second.linear.x;
  velocity_profile.back().linear.y = vel_goal_.second.linear.y;
  velocity_profile.back().angular.z = vel_goal_.second.angular.z;
}

void TebOptimalPlanner::getFullTrajectory(std::vector<TrajectoryPointMsg>& trajectory) const
{
  int n = teb_.sizePoses();
  
  trajectory.resize(n);
  
  if (n == 0)
    return;
     
  double curr_time = 0;
  
  // start
  TrajectoryPointMsg& start = trajectory.front();
  teb_.Pose(0).toPoseMsg(start.pose);
  start.velocity.linear.z = 0;
  start.velocity.angular.x = start.velocity.angular.y = 0;
  start.velocity.linear.x = vel_start_.second.linear.x;
  start.velocity.linear.y = vel_start_.second.linear.y;
  start.velocity.angular.z = vel_start_.second.angular.z;
  start.time_from_start.fromSec(curr_time);
  
  curr_time += teb_.TimeDiff(0);
  
  // intermediate points
  for (int i=1; i < n-1; ++i)
  {
    TrajectoryPointMsg& point = trajectory[i];
    teb_.Pose(i).toPoseMsg(point.pose);
    point.velocity.linear.z = 0;
    point.velocity.angular.x = point.velocity.angular.y = 0;
    double vel1_x, vel1_y, vel2_x, vel2_y, omega1, omega2;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), vel1_x, vel1_y, omega1);
    extractVelocity(teb_.Pose(i), teb_.Pose(i+1), teb_.TimeDiff(i), vel2_x, vel2_y, omega2);
    point.velocity.linear.x = 0.5*(vel1_x+vel2_x);
    point.velocity.linear.y = 0.5*(vel1_y+vel2_y);
    point.velocity.angular.z = 0.5*(omega1+omega2);    
    point.time_from_start.fromSec(curr_time);
    
    curr_time += teb_.TimeDiff(i);
  }
  
  // goal
  TrajectoryPointMsg& goal = trajectory.back();
  teb_.BackPose().toPoseMsg(goal.pose);
  goal.velocity.linear.z = 0;
  goal.velocity.angular.x = goal.velocity.angular.y = 0;
  goal.velocity.linear.x = vel_goal_.second.linear.x;
  goal.velocity.linear.y = vel_goal_.second.linear.y;
  goal.velocity.angular.z = vel_goal_.second.angular.z;
  goal.time_from_start.fromSec(curr_time);
}

/*
Eigen::Vector2d TebOptimalPlanner::pushPoseAwayFromObstacle(PoseSE2& pose, unsigned int width, unsigned int height)
{
    const float* distance_field = map_subscriber_->getDistanceField(); // Use the distance field from the global subscriber

    if (distance_field)
    {
        // intermediate pose 위치를 가져옵니다.
        Eigen::Vector2d position = pose.position();

        int xi = static_cast<int>(position.x());
        int yi = static_cast<int>(position.y());

        float distance = distance_field[xi + yi * width];

        ROS_INFO("Distance : %f", distance);

        if (distance != -1)
        {
            // Calculate the direction vector away from the obstacle
            float dx = xi - position.x();
            float dy = yi - position.y();
            float angle = atan2(dy, dx);
            float move_distance = 0.01 * distance; // Move 50% of the distance away from the obstacle

            // 새 위치를 계산합니다.
            position.x() += move_distance * cos(angle);
            position.y() += move_distance * sin(angle);

            // pose 객체의 위치를 새로운 위치로 업데이트합니다.
            pose.position() = position;

            ROS_INFO("New Position x : %f, y : %f", position.x(), position.y());

            // Visualization and debugging
            visualization_->visualizeIntermediatePoint(pose); 
            
            std::cout << "Press Enter to continue..." << std::endl;
            std::cin.get();
        }
    }
    else
    {
        ROS_DEBUG("Distance field is null");
    }

     return pose.position(); // Return the updated position
}
*/

Eigen::Vector2d TebOptimalPlanner::pushPoseAwayFromObstacle(PoseSE2& pose, unsigned int width, unsigned int height)
{
    // Ensure map subscriber is valid and data is ready
    if (!map_subscriber_ || !map_subscriber_->isDataReady()) {
        ROS_WARN("Map data not available. Cannot push pose away from obstacle.");
        return pose.position(); // Return the current position if no map data
    }

    // Retrieve the current position from the pose
    Eigen::Vector2d position = pose.position();

    // Get the closest obstacle to the current position
    Eigen::Vector2d closest_obstacle = map_subscriber_->getClosestObstacle(position.x(), position.y());

    // Check if the closest obstacle is valid
    if (std::isnan(closest_obstacle.x()) || std::isnan(closest_obstacle.y())) {
        ROS_WARN("Invalid closest obstacle coordinates.");
        return pose.position(); // Return the current position if the closest obstacle is invalid
    }

    // Calculate the direction vector from the current position to the closest obstacle
    Eigen::Vector2d direction_to_obstacle = closest_obstacle - position;

    // Calculate the distance to the closest obstacle
    float distance_to_obstacle = direction_to_obstacle.norm();

    // Calculate the direction vector away from the obstacle
    Eigen::Vector2d direction_away_from_obstacle = -direction_to_obstacle.normalized();

    // Determine how much to move away from the obstacle
    float move_distance = 0.01 * distance_to_obstacle; // Adjust this factor as needed

    // Compute the new position
    Eigen::Vector2d new_position = position + move_distance * direction_away_from_obstacle;

    // Update the pose with the new position
    pose.position() = new_position;

    ROS_INFO("New Position x: %f, y: %f", new_position.x(), new_position.y());

    // Visualization and debugging
    visualization_->visualizeIntermediatePoint(pose);

    return new_position; // Return the updated position
}

void TebOptimalPlanner::printDistanceField()
{
    if (map_subscriber_->isDataReady())
    {
        const float* distance_field = map_subscriber_->getDistanceField();
        if (distance_field)
        {
            unsigned int width = map_subscriber_->getWidth();
            unsigned int height = map_subscriber_->getHeight();

            ROS_INFO("Distance Field:");
            for (unsigned int y = 0; y < height; ++y)
            {
                for (unsigned int x = 0; x < width; ++x)
                {
                    float distance = distance_field[x + y * width];
                    // Print values with a tab for readability
                    printf("%6.2f\t", distance);
                }
                printf("\n");
            }
        }
        else
        {
            ROS_WARN("Distance field is not initialized.");
        }
    }
    else
    {
        ROS_WARN("Map subscriber is not initialized.");
    }
}

Eigen::Vector2d TebOptimalPlanner::processPose(PoseSE2& target_pose)
{

    Eigen::Vector2d modified_pose = target_pose.position(); 
    
    // Check if the map subscriber has received the map data
    if (map_subscriber_->isDataReady())
    {
        ROS_DEBUG("Map subscriber is not null");
        unsigned int width = map_subscriber_->getWidth();
        unsigned int height = map_subscriber_->getHeight();
        const float* distance_field = map_subscriber_->getDistanceField();

        ROS_DEBUG("Width: %u, Height: %u", width, height);

        if (distance_field)
        {
            ROS_DEBUG("Distance field is not null");

            PoseSE2 modifiable_pose = target_pose;

            ROS_DEBUG("Push pose away from obstacle");
            modified_pose = pushPoseAwayFromObstacle(modifiable_pose, width, height);

            ROS_INFO("Modified pose position: [%f, %f]", modifiable_pose.position().x(), modifiable_pose.position().y());
        }
        else
        {
            ROS_DEBUG("Distance field is null");
        }
    }
    else
    {
        ROS_DEBUG("Map subscriber is null or data is not ready");
    }

    return modified_pose; 
}

void TebOptimalPlanner::reOptimizeDuplicatedAndCircularPoses(double proximity_threshold, double circular_threshold)
{
    is_reoptimization_active = true; 

    double accumulated_rotation = 0.0;

    // 포즈들을 순차적으로 확인하면서 중복 및 회전 구간 탐지
    for (int i = 1; i < teb().sizePoses(); ++i)
    {
        // 포즈 간의 거리 계산 (비슷한 위치의 포즈 탐지)
        double dist = std::hypot(teb().Pose(i).x() - teb().Pose(i-1).x(),
                                 teb().Pose(i).y() - teb().Pose(i-1).y());

        // theta(회전 각도) 차이 계산 (회전 경로 탐지)
        double delta_theta = g2o::normalize_theta(teb().Pose(i).theta() - teb().Pose(i-1).theta());
        accumulated_rotation += fabs(delta_theta);

        // 중복된 포즈(가까운 위치) 또는 회전이 360도 이상(2π)인 구간을 찾음
        if (dist < proximity_threshold || accumulated_rotation > circular_threshold)
        {
            redundant_indices.push_back(i); // 중복된 포즈 또는 회전 구간 인덱스 저장
        }
        else if (!redundant_indices.empty())
        {
            // 중복된 구간이 끝났으면 해당 구간을 최적화
            optimizePoseSegment(redundant_indices.front() - 1, redundant_indices.back());
            redundant_indices.clear(); // 리스트 초기화
            accumulated_rotation = 0.0; // 회전 초기화
        }
    }

    // 남은 중복 또는 circular 구간이 있을 경우 최적화
    if (!redundant_indices.empty())
    {
        optimizePoseSegment(redundant_indices.front() - 1, redundant_indices.back());
    }

    is_reoptimization_active = false;
}

// 구간에 대한 최적화를 수행하는 함수 (이전과 동일)
void TebOptimalPlanner::optimizePoseSegment(int start_idx, int end_idx)
{
    std::vector<PoseSE2> pose_segment;
   
    // 포즈를 복사
    for (int i = start_idx; i <= end_idx; ++i)
    {
        pose_segment.push_back(teb().Pose(i));
    }

    optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
   
    // 최적화된 포즈를 기존 경로에 반영
    for (int i = start_idx, j = 0; i <= end_idx; ++i, ++j)
    {
        teb().Pose(i) = pose_segment[j];
    }
}


// orginal func

bool TebOptimalPlanner::isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
                                             double inscribed_radius, double circumscribed_radius, int look_ahead_idx, double feasibility_check_lookahead_distance)
{
  if (look_ahead_idx < 0 || look_ahead_idx >= teb().sizePoses())
    look_ahead_idx = teb().sizePoses() - 1;

  if (feasibility_check_lookahead_distance > 0){
    for (int i=1; i < teb().sizePoses(); ++i){
      double pose_distance=std::hypot(teb().Pose(i).x()-teb().Pose(0).x(), teb().Pose(i).y()-teb().Pose(0).y());
      if(pose_distance > feasibility_check_lookahead_distance){
        look_ahead_idx = i - 1;
        break;
      }
    }
  }

  for (int i=0; i <= look_ahead_idx; ++i)
  {           
    if ( costmap_model->footprintCost(teb().Pose(i).x(), teb().Pose(i).y(), teb().Pose(i).theta(), footprint_spec, inscribed_radius, circumscribed_radius) == -1 )
    {
      if (visualization_)
      {
        visualization_->publishInfeasibleRobotPose(teb().Pose(i), *cfg_->robot_model, footprint_spec);
      }
      return false;
    }
    // Checks if the distance between two poses is higher than the robot radius or the orientation diff is bigger than the specified threshold
    // and interpolates in that case.
    // (if obstacles are pushing two consecutive poses away, the center between two consecutive poses might coincide with the obstacle ;-)!
    if (i<look_ahead_idx)
    {
      double delta_rot = g2o::normalize_theta(g2o::normalize_theta(teb().Pose(i+1).theta()) -
                                              g2o::normalize_theta(teb().Pose(i).theta()));
      Eigen::Vector2d delta_dist = teb().Pose(i+1).position()-teb().Pose(i).position();
      if(fabs(delta_rot) > cfg_->trajectory.min_resolution_collision_check_angular || delta_dist.norm() > inscribed_radius)
      {
        int n_additional_samples = std::max(std::ceil(fabs(delta_rot) / cfg_->trajectory.min_resolution_collision_check_angular), 
                                            std::ceil(delta_dist.norm() / inscribed_radius)) - 1;
        PoseSE2 intermediate_pose = teb().Pose(i);
        for(int step = 0; step < n_additional_samples; ++step)
        {
          ROS_INFO("check for intermediate pose after optimization");

          intermediate_pose.position() = intermediate_pose.position() + delta_dist / (n_additional_samples + 1.0);
          intermediate_pose.theta() = g2o::normalize_theta(intermediate_pose.theta() + 
                                                           delta_rot / (n_additional_samples + 1.0));

          if ( costmap_model->footprintCost(intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta(),
            footprint_spec, inscribed_radius, circumscribed_radius) == -1 )
          {
            if (visualization_) 
            {
              visualization_->publishInfeasibleRobotPose(intermediate_pose, *cfg_->robot_model, footprint_spec);
            }
            return false;
          }
        }
      }
    }
  }
  return true;
}

/*
bool TebOptimalPlanner::isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
                                             double inscribed_radius, double circumscribed_radius, int look_ahead_idx, double feasibility_check_lookahead_distance)
{
  if (look_ahead_idx < 0 || look_ahead_idx >= teb().sizePoses())
    look_ahead_idx = teb().sizePoses() - 1;

  if (feasibility_check_lookahead_distance > 0){
    for (int i=1; i < teb().sizePoses(); ++i){
      double pose_distance=std::hypot(teb().Pose(i).x()-teb().Pose(0).x(), teb().Pose(i).y()-teb().Pose(0).y());
      if(pose_distance > feasibility_check_lookahead_distance){
        look_ahead_idx = i - 1;
        break;
      }
    }
  }

  for (int i=0; i <= look_ahead_idx; ++i)
  {           
    if ( costmap_model->footprintCost(teb().Pose(i).x(), teb().Pose(i).y(), teb().Pose(i).theta(), footprint_spec, inscribed_radius, circumscribed_radius) == -1 )
    {
      if (visualization_)
      {
        visualization_->publishInfeasibleRobotPose(teb().Pose(i), *cfg_->robot_model, footprint_spec);
      }
      return false;
    }
    reOptimizeDuplicatedAndCircularPoses(proximity_threshold, circular_threshold);
    // Checks if the distance between two poses is higher than the robot radius or the orientation diff is bigger than the specified threshold
    // and interpolates in that case.
    // (if obstacles are pushing two consecutive poses away, the center between two consecutive poses might coincide with the obstacle ;-)!

    if (i<look_ahead_idx)
    {
      double delta_rot = g2o::normalize_theta(g2o::normalize_theta(teb().Pose(i+1).theta()) -
                                              g2o::normalize_theta(teb().Pose(i).theta()));
      Eigen::Vector2d delta_dist = teb().Pose(i+1).position()-teb().Pose(i).position();
      
      if(fabs(delta_rot) > cfg_->trajectory.min_resolution_collision_check_angular || delta_dist.norm() > inscribed_radius)
      {
        int n_additional_samples = std::max(std::ceil(fabs(delta_rot) / cfg_->trajectory.min_resolution_collision_check_angular), 
                                            std::ceil(delta_dist.norm() / inscribed_radius)) - 1;
        PoseSE2 intermediate_pose = teb().Pose(i);
        for(int step = 0; step < n_additional_samples; ++step)
        {
          ROS_INFO("check for intermediate pose after optimization");

          intermediate_pose.position() = intermediate_pose.position() + delta_dist / (n_additional_samples + 1.0);
          intermediate_pose.theta() = g2o::normalize_theta(intermediate_pose.theta() + 
                                                           delta_rot / (n_additional_samples + 1.0));

          if ( costmap_model->footprintCost(intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta(),
            footprint_spec, inscribed_radius, circumscribed_radius) == -1 )
          {
            if (visualization_) 
            {
              visualization_->publishInfeasibleRobotPose(intermediate_pose, *cfg_->robot_model, footprint_spec);
            }
            return false;
          }
        }
      }
    }

  }
  return true;
}
*/

/*
// optimizeTEB 한 번 더 시도 
// 문제가 있는 부분만 optimization 실행 필요

bool TebOptimalPlanner::isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
                                             double inscribed_radius, double circumscribed_radius, int look_ahead_idx, double feasibility_check_lookahead_distance)
{
  if (look_ahead_idx < 0 || look_ahead_idx >= teb().sizePoses())
    look_ahead_idx = teb().sizePoses() - 1;
    
  ROS_DEBUG("tebsizePoses: %d", teb().sizePoses());
  ROS_DEBUG("look ahead idx: %d", look_ahead_idx);

  if (feasibility_check_lookahead_distance > 0)
  {
    for (int i=1; i < teb().sizePoses(); ++i)
    {
      double pose_distance=std::hypot(teb().Pose(i).x()-teb().Pose(0).x(), teb().Pose(i).y()-teb().Pose(0).y());
      if(pose_distance > feasibility_check_lookahead_distance)
      {
        look_ahead_idx = i - 1;
        break;
      }
    }
  }

  for (int i=0; i <= look_ahead_idx; ++i)
  {           
    double cost = costmap_model->footprintCost(teb().Pose(i).x(), teb().Pose(i).y(), teb().Pose(i).theta(), footprint_spec, inscribed_radius, circumscribed_radius);
    ROS_DEBUG("Footprint cost at pose %d: %lf", i, cost);
    if (visualization_)
    {
        visualization_->publishRobotPose(teb().Pose(i), *cfg_->robot_model, footprint_spec);
    }

    if (cost == -1)
    {
        if (visualization_)
        {
            visualization_->publishInfeasibleRobotPose(teb().Pose(i), *cfg_->robot_model, footprint_spec);
        }
        return false;
    }
  
    // Checks if the distance between two poses is higher than the robot radius or the orientation diff is bigger than the specified threshold
    // and interpolates in that case.
    // (if obstacles are pushing two consecutive poses away, the center between two consecutive poses might coincide with the obstacle ;-)!
    if (i < look_ahead_idx)
    {
        double delta_rot = g2o::normalize_theta(g2o::normalize_theta(teb().Pose(i + 1).theta()) -
                                                g2o::normalize_theta(teb().Pose(i).theta()));
        ROS_INFO("delta_rot : %lf", delta_rot);
        Eigen::Vector2d delta_dist = teb().Pose(i + 1).position() - teb().Pose(i).position();
        ROS_INFO("delta_dist x : %lf, delta_dist y : %lf", delta_dist.x(), delta_dist.y());

        if (fabs(delta_rot) > cfg_->trajectory.min_resolution_collision_check_angular || delta_dist.norm() > inscribed_radius)
        {
            int n_additional_samples = std::max(std::ceil(fabs(delta_rot) / cfg_->trajectory.min_resolution_collision_check_angular),
                                                std::ceil(delta_dist.norm() / inscribed_radius)) - 1;

            // intermediate pose 전 후의 teb pose visualize                                     
            visualization_->publishRobotPose(teb().Pose(i), *cfg_->robot_model, footprint_spec);
            visualization_->publishRobotPose(teb().Pose(i+1), *cfg_->robot_model, footprint_spec);

            // 프로그램을 일시적으로 멈추고 사용자 입력을 기다림
            // std::cout << "Press Enter to continue..." << std::endl;
            // std::cin.get();

            ROS_DEBUG("additional_samples number : %d", n_additional_samples);
            PoseSE2 intermediate_pose = teb().Pose(i);
            ROS_DEBUG("pose %d turn into intermediate_pose 0", i);

            std::vector<PoseSE2> intermediate_poses; // 중간 포즈를 저장할 임시 리스트

            for (int step = 0; step < n_additional_samples; ++step)
            {
                intermediate_pose.position() = intermediate_pose.position() + delta_dist / (n_additional_samples + 1.0);
                intermediate_pose.theta() = g2o::normalize_theta(intermediate_pose.theta() +
                                                                 delta_rot / (n_additional_samples + 1.0));
                double intermediate_cost = costmap_model->footprintCost(intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta(),
                                                                        footprint_spec, inscribed_radius, circumscribed_radius);
                
                visualization_->visualizeIntermediatePoint(intermediate_pose); 
                intermediate_poses.push_back(intermediate_pose);

                if (intermediate_cost == -1)
                {
                  visualization_->publishInfeasibleRobotPose(intermediate_pose, *cfg_->robot_model, footprint_spec);
                }
            }

            ROS_DEBUG("teb size : %d", teb().sizePoses());
            ROS_DEBUG("intermediate poses size : %d", intermediate_poses.size());

            for(int g = 0; g < teb().sizePoses(); g++)
            {
              ROS_INFO("pose %d,  x : %f, y : %f", g, teb().Pose(g).x(), teb().Pose(g).y());
            }

            // Insert the new pose and time difference at the position
            for (int k = 0; k < intermediate_poses.size(); k++)
            {
              teb().insertPose(i + k + 1, intermediate_poses[k]);
              teb().insertTimeDiff(i + k + 1, cfg_->trajectory.dt_ref);
            }

            for(int g = 0; g < teb().sizePoses(); g++)
            {
              ROS_INFO("pose %d,  x : %f, y : %f", g, teb().Pose(g).x(), teb().Pose(g).y());
              double pose_cost = costmap_model->footprintCost(teb().Pose(g).x(), teb().Pose(g).y(), teb().Pose(g).theta(),
                                                                        footprint_spec, inscribed_radius, circumscribed_radius);
              ROS_INFO("Pose cost : %lf", pose_cost);
            }
            
            ROS_DEBUG("FINISH INSERTING NEW POSES");

            ROS_DEBUG("teb size : %d", teb_.sizePoses());
                
            // Implement optimization part when adding intermediate pose 
            for(int i = 0; i < intermediate_poses.size(); i++)
            {
                ROS_INFO("Footprint position x : %f, y : %f", intermediate_poses[i].x(), intermediate_poses[i].y());
            }
            
            //std::cout << "Press Enter to continue..." << std::endl;
            //std::cin.get();

            ROS_INFO("optimizeTEB with intermediate pose"); 

            optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
              
            for(int g = 0; g < teb().sizePoses(); g++)
            {
              ROS_INFO("pose %d,  x : %f, y : %f", g, teb().Pose(g).x(), teb().Pose(g).y());
              double pose_cost = costmap_model->footprintCost(teb().Pose(g).x(), teb().Pose(g).y(), teb().Pose(g).theta(),
                                                                        footprint_spec, inscribed_radius, circumscribed_radius);
              ROS_INFO("Pose cost : %lf", pose_cost);
            }
            }
        }
    }
  return true;
}

// push poses 

bool TebOptimalPlanner::isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
                                             double inscribed_radius, double circumscribed_radius, int look_ahead_idx, double feasibility_check_lookahead_distance)
{
  if (look_ahead_idx < 0 || look_ahead_idx >= teb().sizePoses())
    look_ahead_idx = teb().sizePoses() - 1;
    
  ROS_DEBUG("tebsizePoses: %d", teb().sizePoses());
  ROS_DEBUG("look ahead idx: %d", look_ahead_idx);
  ROS_INFO("inscribed radius : %d", inscribed_radius);

  if (feasibility_check_lookahead_distance > 0)
  {
    for (int i=1; i < teb().sizePoses(); ++i)
    {
      double pose_distance=std::hypot(teb().Pose(i).x()-teb().Pose(0).x(), teb().Pose(i).y()-teb().Pose(0).y());
      if(pose_distance > feasibility_check_lookahead_distance)
      {
        look_ahead_idx = i - 1;
        break;
      }
    }
  }

  for (int i=0; i <= look_ahead_idx; ++i)
  {    
    // i번째 pose의 cost가 -1이면 push pose 
    double cost = costmap_model->footprintCost(teb().Pose(i).x(), teb().Pose(i).y(), teb().Pose(i).theta(), footprint_spec, inscribed_radius, circumscribed_radius);    
    ROS_INFO("pose %d,  x : %f, y : %f, cost : %lf", i, teb().Pose(i).x(), teb().Pose(i).y(), cost);

    if (cost == -1)
    {
        float distance = map_subscriber_->getDistanceAt(teb().Pose(i).x(), teb().Pose(i).y());
        ROS_INFO("Distance at pose: %f", distance);

        teb().Pose(i).position() = processPose(teb().Pose(i));

        //std::cout << "Press Enter to continue..." << std::endl;
        //std::cin.get();

        if (visualization_)
        {
            visualization_->publishInfeasibleRobotPose(teb().Pose(i), *cfg_->robot_model, footprint_spec);
        }
    }
  
    // Checks if the distance between two poses is higher than the robot radius or the orientation diff is bigger than the specified threshold
    // and interpolates in that case.
    // (if obstacles are pushing two consecutive poses away, the center between two consecutive poses might coincide with the obstacle ;-)!
    if (i < look_ahead_idx)
    {
        double delta_rot = g2o::normalize_theta(g2o::normalize_theta(teb().Pose(i + 1).theta()) -
                                                g2o::normalize_theta(teb().Pose(i).theta()));
        ROS_INFO("delta_rot : %lf", delta_rot);
        Eigen::Vector2d delta_dist = teb().Pose(i + 1).position() - teb().Pose(i).position();
        ROS_INFO("delta_dist x : %lf, delta_dist y : %lf", delta_dist.x(), delta_dist.y());

        if (fabs(delta_rot) > cfg_->trajectory.min_resolution_collision_check_angular || delta_dist.norm() > inscribed_radius)
        {
            int n_additional_samples = std::max(std::ceil(fabs(delta_rot) / cfg_->trajectory.min_resolution_collision_check_angular),
                                                std::ceil(delta_dist.norm() / inscribed_radius)) - 1;

            // intermediate pose 전 후의 teb pose visualize                                     
            visualization_->publishRobotPose(teb().Pose(i), *cfg_->robot_model, footprint_spec);
            visualization_->publishRobotPose(teb().Pose(i+1), *cfg_->robot_model, footprint_spec);


            ROS_DEBUG("additional_samples number : %d", n_additional_samples);
            PoseSE2 intermediate_pose = teb().Pose(i);
            ROS_DEBUG("pose %d turn into intermediate_pose 0", i);

            std::vector<PoseSE2> intermediate_poses; // 중간 포즈를 저장할 임시 리스트

            for (int step = 0; step < n_additional_samples; ++step)
            {
                intermediate_pose.position() = intermediate_pose.position() + delta_dist / (n_additional_samples + 1.0);
                intermediate_pose.theta() = g2o::normalize_theta(intermediate_pose.theta() +
                                                                 delta_rot / (n_additional_samples + 1.0));

                double intermediate_cost = costmap_model->footprintCost(intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta(),
                                                                        footprint_spec, inscribed_radius, circumscribed_radius);
                
                visualization_->visualizeIntermediatePoint(intermediate_pose); 

                intermediate_poses.push_back(intermediate_pose);

                if (intermediate_cost == -1)
                {
                  visualization_->publishInfeasibleRobotPose(intermediate_pose, *cfg_->robot_model, footprint_spec);
                }
            }

            ROS_DEBUG("teb size : %d", teb().sizePoses());
            ROS_DEBUG("intermediate poses size : %d", intermediate_poses.size());

            for(int g = 0; g < teb().sizePoses(); g++)
            {
              ROS_INFO("pose %d,  x : %f, y : %f", g, teb().Pose(g).x(), teb().Pose(g).y());
            }

            // Insert the new pose and time difference at the position
            for (int k = 0; k < intermediate_poses.size(); k++)
            {
              teb().insertPose(i + k + 1, intermediate_poses[k]);
              teb().insertTimeDiff(i + k + 1, cfg_->trajectory.dt_ref);
            }

            ROS_DEBUG("FINISH INSERTING NEW POSES");
            ROS_DEBUG("teb size : %d", teb_.sizePoses());

            for(int g = 0; g < look_ahead_idx; g++)
            {
              ROS_INFO("pose %d,  x : %f, y : %f", g, teb().Pose(g).x(), teb().Pose(g).y());
              double pose_cost = costmap_model->footprintCost(teb().Pose(g).x(), teb().Pose(g).y(), teb().Pose(g).theta(),
                                                                        footprint_spec, inscribed_radius, circumscribed_radius);
              ROS_INFO("Pose cost : %lf", pose_cost);
        
              if (pose_cost == -1)
              {
                float distance = map_subscriber_->getDistanceAt(teb().Pose(g).x(), teb().Pose(g).y());
                ROS_INFO("Distance at pose: %f", distance);

                processPose(teb().Pose(g));

                teb().Pose(g).position() = processPose(teb().Pose(i));
              }
            }

            for(int g = 0; g < teb().sizePoses(); g++)
            {
              ROS_INFO("pose %d,  x : %f, y : %f", g, teb().Pose(g).x(), teb().Pose(g).y());
              double pose_cost = costmap_model->footprintCost(teb().Pose(g).x(), teb().Pose(g).y(), teb().Pose(g).theta(),
                                                                        footprint_spec, inscribed_radius, circumscribed_radius);
              ROS_INFO("Pose cost : %lf", pose_cost);
            }
        }
    }
  }
  return true;
}
*/

} // namespace teb_local_planner
