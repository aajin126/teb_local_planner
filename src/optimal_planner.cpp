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
#include <teb_local_planner/g2o_types/edge_safe_point.h>
#include <teb_local_planner/g2o_types/edge_prefer_rotdir.h>
#include <teb_local_planner/g2o_types/edge_medial_attraction.h>

#include <memory>
#include <limits>

namespace teb_local_planner
{

// ============== Implementation ===================

TebOptimalPlanner::TebOptimalPlanner() : cfg_(NULL), obstacles_(NULL), via_points_(NULL), distance_field_(NULL), costmap_info_(NULL), px_out_(NULL), py_out_(NULL), cost_(HUGE_VAL), prefer_rotdir_(RotType::none),
                                         initialized_(false), optimized_(false)
{    
}
  
TebOptimalPlanner::TebOptimalPlanner(const TebConfig& cfg, ObstContainer* obstacles, TebVisualizationPtr visual, const ViaPointContainer* via_points, const std::vector<float>* distance_field, const DistanceMapInfo* costmap_info, const std::vector<int>* px_out, const std::vector<int>* py_out)
{
  initialize(cfg, obstacles, visual, via_points, distance_field, costmap_info, px_out, py_out);
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

void TebOptimalPlanner::initialize(const TebConfig& cfg, ObstContainer* obstacles, TebVisualizationPtr visual, const ViaPointContainer* via_points, const std::vector<float>* distance_field, const DistanceMapInfo* costmap_info, const std::vector<int>* px_out, const std::vector<int>* py_out)
{    
  // init optimizer (set solver and block ordering settings)
  optimizer_ = initOptimizer();
  
  cfg_ = &cfg;
  obstacles_ = obstacles;
  via_points_ = via_points;
  distance_field_ = distance_field;
  px_out_ = px_out;
  py_out_ = py_out;
  costmap_info_ = costmap_info;
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
  initialized_ = true;
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
  factory->registerType("EDGE_MEDIAL_ATTRACTION", new g2o::HyperGraphElementCreator<EdgeMedialAttraction>);
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
{
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
      //teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples);
      teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples, fast_mode);

    }

    success = buildGraph(weight_multiplier);
    if (!success) 
    {
        clearGraph();
        return false;
    }
    success = optimizeGraph(iterations_innerloop, false);
    if (!success) 
    {
        clearGraph();
        return false;
    }
    optimized_ = true;
    
    if (compute_cost_afterwards && i==iterations_outerloop-1) // compute cost vec only in the last iteration
      computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost);
      
    clearGraph();
    
    weight_multiplier *= cfg_->optim.weight_adapt_factor;
  }

  return true;
}


bool TebOptimalPlanner::adaptiveoptimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards, double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{
  if (cfg_->optim.optimization_activate==false)
    return false;

  bool success = false;
  optimized_ = false;
  ROS_DEBUG("optimizeTEB");
  double weight_multiplier = 1.0;

  bool fast_mode = !cfg_->obstacles.include_dynamic_obstacles;

  for(int i=0; i<iterations_outerloop; ++i)
  {
     {
      size_t M = teb().sizeTimeDiffs();
      ref_timediffs_.assign(M, cfg_->trajectory.dt_ref);
      hyst_timediffs_.assign(M, cfg_->trajectory.dt_hysteresis);
    }

    if (cfg_->trajectory.teb_autosize)
    {
     std::ofstream outFile("/home/glab/bisection_log.txt", std::ios::app);
     if (outFile.is_open())
     {
       outFile << "ref_timediffs_ size: " << ref_timediffs_.size() << "\n";
       outFile << "hyst_timediffs_ size: " << hyst_timediffs_.size() << "\n";
     }
     else
     {
       std::cerr << "File not opened\n";
     }
      teb_.adaptiveautoResize(ref_timediffs_, hyst_timediffs_, cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples, fast_mode);
      //teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples, fast_mode);
    }
    success = adaptivebuildGraph(weight_multiplier);
    if (!success)
    {
        clearGraph();
        return false;
    }
    success = optimizeGraph(iterations_innerloop, false);
    if (!success)
    {
        clearGraph();
        return false;
    }
    optimized_ = true;

    if (compute_cost_afterwards && i==iterations_outerloop-1) // compute cost vec only in the last iteration
      computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost);

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
    { 
      ROS_INFO("warm start");
      teb_.updateAndPruneTEB(start_, goal_, cfg_->trajectory.min_samples); // update TEB
    }
      
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
  PoseSE2 start_(start);
  PoseSE2 goal_(goal);
  return plan(start_, goal_, start_vel);
}

bool TebOptimalPlanner::plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{	
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
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
  {
    ROS_WARN("Cannot build graph, because it is not empty. Call graphClear()!");
    return false;
  }
  optimizer_->setComputeBatchStatistics(cfg_->recovery.divergence_detection_enable);
  
  // add TEB vertices
  AddTEBVertices();

  // add Edges (local cost functions)
  if (cfg_->obstacles.legacy_obstacle_association)
    AddEdgesObstaclesLegacy(weight_multiplier);
  else
    AddEdgesObstacles(weight_multiplier);

  if (cfg_->obstacles.include_dynamic_obstacles)
    AddEdgesDynamicObstacles();

  //AddEdgesMedialAttraction();

  //AddEdgesViaPoints();
  //AddEdgesSafePoints();
  
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
    
  return true;  
}

bool TebOptimalPlanner::adaptivebuildGraph(double weight_multiplier)
{
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
  {
    ROS_WARN("Cannot build graph, because it is not empty. Call graphClear()!");
    return false;
  }
  optimizer_->setComputeBatchStatistics(cfg_->recovery.divergence_detection_enable);
  
  // add TEB vertices
  AddTEBVertices();

  // add Edges (local cost functions)
  if (cfg_->obstacles.legacy_obstacle_association)
    AddEdgesObstaclesLegacy(weight_multiplier);
  else
    AddEdgesObstacles(weight_multiplier);

  if (cfg_->obstacles.include_dynamic_obstacles)
    AddEdgesDynamicObstacles();

  AddEdgesSafePoints();
  
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
  ROS_DEBUG("Add vertices");
  ROS_DEBUG("Add vertices : %d\n", teb_.sizePoses());
  ROS_DEBUG_COND(cfg_->optim.optimization_verbose, "Adding TEB vertices ...");
  unsigned int id_counter = 0; // used for vertices ids
  obstacles_per_vertex_.resize(teb_.sizePoses());
  auto iter_obstacle = obstacles_per_vertex_.begin();
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
        iter_obstacle->push_back(left_obstacle);
      if (right_obstacle)
        iter_obstacle->push_back(right_obstacle);

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
  if (cfg_->optim.weight_viapoint==0 || via_points_==NULL || via_points_->empty() )
    return; // if weight equals zero skip adding edges!

  int start_pose_idx = 0;

  int n = teb_.sizePoses();
  if (n<3) // we do not have any degrees of freedom for reaching via-points
    return;

  for (ViaPointContainer::const_iterator vp_it = via_points_->begin(); vp_it != via_points_->end(); ++vp_it)
  {

    int index = teb_.findClosestTrajectoryPose(*vp_it, NULL, start_pose_idx);
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

void TebOptimalPlanner::AddEdgesSafePoints()
{
  ROS_INFO("AddEgesSafePoints");
  if (cfg_->optim.weight_safepoint==0 || safe_points_.empty() )
    return; // if weight equals zero skip adding edges!

  visualization_->visualizeSafePoints(safe_points_);
  //std::cin.get();
  int start_pose_idx = 0;

  int n = teb_.sizePoses();
  if (n<3) // we do not have any degrees of freedom for reaching safe-points
    return;

  for (SafePointContainer::const_iterator sp_it = safe_points_.begin(); sp_it != safe_points_.end(); ++sp_it)
  {

    int index = teb_.findClosestTrajectoryPose(*sp_it, NULL, start_pose_idx);
    if (cfg_->trajectory.safe_points_ordered)
      start_pose_idx = index+2; // skip a point to have a DOF inbetween for further safe-points

    // check if point conicides with goal or is located behind it
    if ( index > n-2 )
      index = n-2; // set to a pose before the goal, since we can move it away!
    // check if point coincides with start or is located before it
    if ( index < 1)
    {
      if (cfg_->trajectory.safe_points_ordered)
      {
        index = 1; // try to connect the safe point with the second (and non-fixed) pose. It is likely that autoresize adds new poses inbetween later.
      }
      else
      {
        ROS_DEBUG("TebOptimalPlanner::AddEdgesSafePoints(): skipping a safe-point that is close or behind the current robot pose.");
        continue; // skip safe points really close or behind the current robot pose
      }
    }
    Eigen::Matrix<double,1,1> information;
    information.fill(cfg_->optim.weight_safepoint);

    EdgeSafePoint* edge_safepoint = new EdgeSafePoint;
    edge_safepoint->setVertex(0,teb_.PoseVertex(index));
    edge_safepoint->setInformation(information);
    edge_safepoint->setParameters(*cfg_, &(*sp_it));
    optimizer_->addEdge(edge_safepoint);
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

void TebOptimalPlanner::AddEdgesMedialAttraction()
{
  std::vector<std::pair<Eigen::Vector2d, double>> medial_point_list_;

  if (cfg_->optim.weight_medialpoint == 0)
    return;
  ROS_DEBUG("Add medial point");

  int n = teb_.sizePoses();

  if (!distance_field_)
  {
    ROS_ERROR("Distance field is null!");
    return;
  }
  medial_point_storage_.clear();
  medial_point_list_.clear();  // 리스트 초기화

  if (n<3) // we do not have any degrees of freedom for reaching medial-points
    return;

  for (int i = 1; i < n; ++i)
  {
    const VertexPose* pose = teb_.PoseVertex(i);
    const Eigen::Vector2d& pos = pose->position();

    ROS_DEBUG("Compute Medial Point");
    auto result =  findMedialBallCenter(pos, *distance_field_, *costmap_info_);

    medial_point_storage_.emplace_back(result.first);
    const Eigen::Vector2d* medial_ptr = &medial_point_storage_.back();
    double radius = result.second;

    medial_point_list_.emplace_back(*medial_ptr, radius);  // 리스트에 저장
    ROS_DEBUG("Finish Computing Medial Point");

    Eigen::Matrix<double,1,1> information;
    information.fill(cfg_->optim.weight_medialpoint);

    EdgeMedialAttraction* edge_medial_attraction = new EdgeMedialAttraction;
    edge_medial_attraction->setVertex(0, teb_.PoseVertex(i));
    edge_medial_attraction->setInformation(information);
    edge_medial_attraction->setParameters(*cfg_, medial_ptr);
    optimizer_->addEdge(edge_medial_attraction);
  }

  // 전체 리스트 시각화
  visualization_->visualizeMedialBall(medial_point_list_);

  ROS_DEBUG("Finish Add medial point");
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

std::pair<Eigen::Vector2d, double> TebOptimalPlanner::findMedialBallCenter(
  const Eigen::Vector2d& point,
  const std::vector<float>& distance_field,
  const DistanceMapInfo& costmap_info)
{
    //ROS_DEBUG("calculate medial ball center");

    unsigned int map_width = costmap_info.map_width;
    unsigned int map_height = costmap_info.map_height;
    double resolution = costmap_info.resolution;
    double origin_x = costmap_info.origin_x;
    double origin_y = costmap_info.origin_y;

    //ROS_INFO("distance_field size = %lu", distance_field.size());

    Eigen::Vector2d medial_center = performMedialAxisClimb(point, distance_field, map_width, map_height, resolution, origin_x, origin_y);
    int grid_x = static_cast<int>((medial_center.x() - origin_x) / resolution);
    int grid_y = static_cast<int>((medial_center.y() - origin_y) / resolution);
    int idx = grid_x + grid_y * map_width;
    double radius = distance_field[idx]* resolution;
    std_msgs::ColorRGBA blue;
    blue.r = 0.0;
    blue.g = 0.0;
    blue.b = 1.0;
    blue.a = 1.0;
    //ROS_DEBUG("finishing perform climb");
    //visualization_->publishArrow(point, medial_center, blue);
    //visualization_ -> visualizeMedialPoint(medial_center, radius);
    return { medial_center, radius };
}

Eigen::Vector2d TebOptimalPlanner::performMedialAxisClimb(
  const Eigen::Vector2d& start_point,
  const std::vector<float>& distance_field,
  unsigned int map_width, unsigned int map_height,
  double resolution, double origin_x, double origin_y)
{
    int cur_x = static_cast<int>((start_point.x() - origin_x) / resolution);
    int cur_y = static_cast<int>((start_point.y() - origin_y) / resolution);

    int cur_idx = cur_x + cur_y * map_width;
    float cur_dist = distance_field[cur_idx];
    const float threshold = 0.3;

    if (cur_dist * resolution >= threshold)
    {
        return Eigen::Vector2d(origin_x + (cur_x + 0.5) * resolution, origin_y + (cur_y + 0.5) * resolution);
    }

    bool moved = true;
    int max_iterations = 100;
    int iteration = 0;

    while (moved && iteration < max_iterations)
    {
        moved = false;
        float best_dist = cur_dist;
        int best_x = cur_x;
        int best_y = cur_y;

        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx == 0 && dy == 0) continue;

                int nx = cur_x + dx;
                int ny = cur_y + dy;
                if (nx < 0 || ny < 0 || nx >= static_cast<int>(map_width) || ny >= static_cast<int>(map_height))
                    continue;

                int n_idx = nx + ny * map_width;
                float n_dist = distance_field[n_idx];

                if (n_dist * resolution >= threshold) {
                    return Eigen::Vector2d(origin_x + (nx + 0.5) * resolution,
                                           origin_y + (ny + 0.5) * resolution);
                }

                if (n_dist > best_dist) {
                    best_dist = n_dist;
                    best_x = nx;
                    best_y = ny;
                    moved = true;
                }
            }
        }

        if (!moved)
            break;

        cur_x = best_x;
        cur_y = best_y;
        cur_idx = cur_x + cur_y * map_width;
        cur_dist = distance_field[cur_idx];
        //ROS_INFO("  fin_dist    = %.4f", cur_dist);

        iteration++;
    }

    return Eigen::Vector2d(origin_x + (cur_x + 0.5) * resolution,
                           origin_y + (cur_y + 0.5) * resolution);
}


bool TebOptimalPlanner::hasDiverged() const
{
  // Early returns if divergence detection is not active
  if (!cfg_->recovery.divergence_detection_enable)
    return false;

  auto stats_vector = optimizer_->batchStatistics();

  // No statistics yet
  if (stats_vector.empty())
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
     else if (dynamic_cast<EdgeSafePoint*>(*it) != nullptr)
     {
       cur_cost *= viapoint_cost_scale;
     }

    else if (dynamic_cast<EdgeTimeOptimal*>(*it) != nullptr && alternative_time_cost)
    {
      continue; // skip these edges if alternative_time_cost is active
    }
    cost_ += cur_cost;
    ROS_INFO("cost : %d", cost_);
  }

  // delete temporary created graph
  if (!graph_exist_flag) 
    clearGraph();
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


//original
//bool TebOptimalPlanner::isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
//                                             double inscribed_radius, double circumscribed_radius, int look_ahead_idx, double feasibility_check_lookahead_distance)
//{
//  if (look_ahead_idx < 0 || look_ahead_idx >= teb().sizePoses())
//    look_ahead_idx = teb().sizePoses() - 1;
//
//  for (int i=0; i <= look_ahead_idx; ++i)
//  {
//    if ( costmap_model->footprintCost(teb().Pose(i).x(), teb().Pose(i).y(), teb().Pose(i).theta(), footprint_spec, inscribed_radius, circumscribed_radius) == -1 )
//    {
//      if (visualization_)
//      {
//        visualization_->publishInfeasibleRobotPose(teb().Pose(i), *cfg_->robot_model, footprint_spec);
//      }
//      return false;
//    }
//  }
//  return true;
//}


Eigen::Vector2d TebOptimalPlanner::getModifiedPosition(const Eigen::Vector2d pose)
{
  const auto& info = *costmap_info_;
  const auto& df   = *distance_field_;
  const int padded_width = info.map_width + 2;
  const int padded_height = info.map_height + 2;
  float eps = 1e-2;
  // 1. World → grid index
  int grid_x = static_cast<int>((pose.x() - info.origin_x) / info.resolution);
  int grid_y = static_cast<int>((pose.y() - info.origin_y) / info.resolution);

  //2. Check bounds
  if (grid_x < 1 || grid_y < 1 || grid_x >= static_cast<int>(info.map_width) - 1 || grid_y >= static_cast<int>(info.map_height) - 1)
    return Eigen::Vector2d(pose.x(), pose.y());

  // 3. Closest obstacle position from px_, py_ (assumed to be stored in padded form)
  int closest_x = (*px_out_)[(grid_x + 1) + (grid_y + 1) * padded_width];
  int closest_y = (*py_out_)[(grid_x + 1) + (grid_y + 1) * padded_width];

  // 4. Convert back to world coordinates
  double closet_world_x = info.origin_x + (closest_x - 1) * info.resolution;
  double closet_world_y = info.origin_y + (closest_y - 1) * info.resolution;

  Eigen::Vector2d nearest = {closet_world_x, closet_world_y};

  float distance = distanceFieldAt(pose.x(), pose.y());

  Eigen::Vector2d direction;

  direction = (pose - nearest).normalized();

  Eigen::Vector2d new_nearest = pose + (0.25 - distance + eps) * direction;
  std_msgs::ColorRGBA blue;
  blue.r = 0.0;
  blue.g = 0.0;
  blue.b = 1.0;
  blue.a = 1.0;
  //visualization_->publishArrow(pose, new_nearest, blue);
  return new_nearest;
}

std::pair<Eigen::Vector2d, double> TebOptimalPlanner::findPerpMedialAxis(const Eigen::Vector2d& coll_pt,const Eigen::Vector2d& p2, const Eigen::Vector2d& p3, std::ostream& log, double max_iterations)
{
    const auto& df   = *distance_field_;
    const auto& info = *costmap_info_;

    // 1) Compute perpendicular direction n
    Eigen::Vector2d d = p3 - p2;
    Eigen::Vector2d n(d.y(), -d.x());
    if (n.norm() < 1e-6) {
      double v = distanceFieldAt(coll_pt.x(), coll_pt.y()) ;
      return { coll_pt, v };
    }
    n.normalize();
    const double max_dist   = 0.4;

    // 3) Bresenham helper unchanged
    auto bresenhamLine = [&](int x0, int y0, int x1, int y1){
      std::vector<Eigen::Vector2i> cells;
      int dx =  std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
      int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
      int err = dx + dy;
      int x = x0, y = y0;
      while (true) {
        cells.emplace_back(x, y);
        if (x == x1 && y == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
      }
      return cells;
    };

    // 4) Define endpoint at physical distance max_dist
    int cx = int((coll_pt.x() - info.origin_x) / info.resolution);
    int cy = int((coll_pt.y() - info.origin_y) / info.resolution);

    // 4.1) Compute world coordinates of endpoint at max_dist
    Eigen::Vector2d end_pt = coll_pt + n * max_dist;

    // 4.2) Convert endpoint to grid indices
    int ex = int((end_pt.x() - info.origin_x) / info.resolution);
    int ey = int((end_pt.y() - info.origin_y) / info.resolution);

    // 4.4) Generate Bresenham lines in ± directions
    auto plusLine  = bresenhamLine(cx, cy, ex, ey);
    //visualization_->visualizeEndPoints(p2, p3);

    int mx = cx - (ex - cx);
    int my = cy - (ey - cy);
    auto minusLine = bresenhamLine(cx, cy, mx, my);
    Eigen::Vector2d minus_pt(
      info.origin_x + (mx + 0.5) * info.resolution,
      info.origin_y + (my + 0.5) * info.resolution
    );
    //visualization_->visualizeLine({minus_pt.x(),minus_pt.y()}, {end_pt.x(),end_pt.y()});

    // 5) Perform 1D hill-climbing along the line in both directions
//    auto climbOnLine = [&](const std::vector<Eigen::Vector2i>& line) {
//      int cur = 0;
//      double cur_val = df[line[cur].x() + line[cur].y() * info.map_width]*info.resolution;
//      for (int it = 0; it < max_iterations; ++it) {
//        int nxt = cur + 1;
//        if (nxt >= (int)line.size()) break;
//        double nxt_val = df[line[nxt].x() + line[nxt].y() * info.map_width]*info.resolution;
//        if (nxt_val >= max_dist) {
//          cur = nxt; cur_val = nxt_val;
//          break;
//        }
//        if (nxt_val > cur_val) {
//          cur = nxt; cur_val = nxt_val;
//        } else {
//          break;
//        }
//      }
//      return std::make_pair(cur, cur_val);
//    };

    auto climbOnLine = [&](const std::vector<Eigen::Vector2i>& line) {
      int best_idx = 0;
      double best_val = df[line[0].x() + line[0].y() * info.map_width] * info.resolution;
      for (size_t i = 1; i < line.size(); ++i)
      {
          double val = df[line[i].x() + line[i].y() * info.map_width] * info.resolution;
          if (val > best_val)
          {
              best_val = val;
              best_idx = i;
              if (val >= max_dist)
                  break;
          }
      }
      return std::make_pair(best_idx, best_val);
    };

    auto pos = climbOnLine(plusLine);
    auto neg = climbOnLine(minusLine);

    // 6) Choose local maxima
    const auto& chosenLine = (pos.second > neg.second) ? plusLine  : minusLine;
    const auto& chosenRes  = (pos.second > neg.second) ? pos.second : neg.second;
    int chosenIdx = (pos.second > neg.second) ? pos.first  : neg.first;

    // 7) Convert the chosen cell back to world coordinates
    Eigen::Vector2i bc = chosenLine[chosenIdx];
    Eigen::Vector2d best_pt( info.origin_x + (bc.x() + 0.5) * info.resolution, info.origin_y + (bc.y() + 0.5) * info.resolution);

   log << "[Plus Line Distances]\n";
   for (size_t i = 0; i < plusLine.size(); ++i)
   {
     const auto& cell = plusLine[i];
     double val = df[cell.x() + cell.y() * info.map_width] * info.resolution;
     double wx = info.origin_x + (cell.x() + 0.5) * info.resolution;
     double wy = info.origin_y + (cell.y() + 0.5) * info.resolution;
     log << "  idx " << i << ": (" << wx << ", " << wy << ") → dist = " << val << "\n";
   }

   // Log all distance values along minusLine
   log << "[Minus Line Distances]\n";
   for (size_t i = 0; i < minusLine.size(); ++i)
   {
     const auto& cell = minusLine[i];
     double val = df[cell.x() + cell.y() * info.map_width] * info.resolution;
     double wx = info.origin_x + (cell.x() + 0.5) * info.resolution;
     double wy = info.origin_y + (cell.y() + 0.5) * info.resolution;
     log << "  idx " << i << ": (" << wx << ", " << wy << ") → dist = " << val << "\n";
   }

   // Log the chosen result
   std::string chosenDir = (pos.second > neg.second) ? "PLUS" : "MINUS";
   log << "[Chosen Point]\n";
   log << "  Direction: " << chosenDir << "\n";
   log << "  Index: " << chosenIdx << "\n";
   log << "  World Coord: (" << best_pt.x() << ", " << best_pt.y() << ")\n";
   log << "  Distance: " << chosenRes << "\n";

    //visualization_->visualizeEndPoints(p2, p3);
    //visualization_->visualizetwoPoint({best_pt.x(),best_pt.y()}, {coll_pt.x(),coll_pt.y()});
    std_msgs::ColorRGBA green;
    green.r = 0.0;
    green.g = 1.0;
    green.b = 0.0;
    green.a = 1.0;
    //visualization_->publishArrow(coll_pt, best_pt, green);
    //std::cin.get();
    //visualization_->visualizeMedialPoint(best_pt, chosenRes);
   log << "findPerpMedialAxis → (" << best_pt.x() << ", " << best_pt.y()
       << "), dist=" << chosenRes << "\n";

    return { best_pt, chosenRes };
}


// 1. Compute closest boundary
Eigen::Vector2d TebOptimalPlanner::getBoundaryPointFromCollision(const Eigen::Vector2d& pt)
{
    const auto& info = *costmap_info_;
    int gx = int((pt.x() - info.origin_x) / info.resolution);
    int gy = int((pt.y() - info.origin_y) / info.resolution);
    int idx = gy * info.map_width + gx;

    int bx = (*px_out_)[idx] - 1;
    int by = (*py_out_)[idx] - 1;
    return Eigen::Vector2d(info.origin_x + (bx + 0.5) * info.resolution, info.origin_y + (by + 0.5) * info.resolution);
}

// 2. Compute pushing direction
Eigen::Vector2d TebOptimalPlanner::computePushDirection(const Eigen::Vector2d& from, const Eigen::Vector2d& to, double dist)
{
    Eigen::Vector2d vec = to - from;
    if (vec.norm() < 1e-3) return Eigen::Vector2d::Zero();
    return vec.normalized();
}

// 3. Compute Perpendicular Direction
Eigen::Vector2d TebOptimalPlanner::computePerpendicularDirection(const Eigen::Vector2d& p2, const Eigen::Vector2d& p3)
{
    Eigen::Vector2d d = p3 - p2;
    Eigen::Vector2d n(d.y(), -d.x());
    if (n.norm() < 1e-6) return Eigen::Vector2d::Zero();
    return n.normalized();
}

Eigen::Vector2d TebOptimalPlanner::estimateNormal(const Eigen::Vector2d& pt)
{
    const auto& info = *costmap_info_;
    const auto& df = *distance_field_;

    int gx = int((pt.x() - info.origin_x) / info.resolution);
    int gy = int((pt.y() - info.origin_y) / info.resolution);

    if (gx <= 0 || gy <= 0 || gx >= info.map_width - 1 || gy >= info.map_height - 1)
        return Eigen::Vector2d::Zero();  // out of bounds

    // central difference
    double dx = (df[(gx+1) + gy * info.map_width] - df[(gx-1) + gy * info.map_width]) / (2.0 * info.resolution);
    double dy = (df[gx + (gy+1) * info.map_width] - df[gx + (gy-1) * info.map_width]) / (2.0 * info.resolution);

    Eigen::Vector2d grad(dx, dy);
    if (grad.norm() < 1e-6) return Eigen::Vector2d::Zero();  // flat / error

    return grad.normalized();  // boundary normal direction (pointing outward)
}

// 4. bresenham (world → grid)
std::vector<Eigen::Vector2i> TebOptimalPlanner::bresenhamLineWorld(const Eigen::Vector2d& from, const Eigen::Vector2d& to)
{
    const auto& info = *costmap_info_;
    int x0 = int((from.x() - info.origin_x) / info.resolution);
    int y0 = int((from.y() - info.origin_y) / info.resolution);
    int x1 = int((to.x() - info.origin_x) / info.resolution);
    int y1 = int((to.y() - info.origin_y) / info.resolution);

    std::vector<Eigen::Vector2i> cells;
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, x = x0, y = y0;

    while (true)
    {
        cells.emplace_back(x, y);
        if (x == x1 && y == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x += sx; }
        if (e2 <= dx) { err += dx; y += sy; }
    }
    return cells;
}


// 5. Searching for local maxima
std::pair<int, double> TebOptimalPlanner::climbLocalMax(const std::vector<Eigen::Vector2i>& line, double max_dist, double max_iterations)
{
    const auto& info = *costmap_info_;
    const auto& df = *distance_field_;
     int cur = 0;
     double cur_val = df[line[cur].x() + line[cur].y() * info.map_width]*info.resolution;
     for (int it = 0; it < max_iterations; ++it) {
       int nxt = cur + 1;
       if (nxt >= (int)line.size()) break;
       double nxt_val = df[line[nxt].x() + line[nxt].y() * info.map_width]*info.resolution;
       if (nxt_val >= max_dist) {
         cur = nxt; cur_val = nxt_val;
         break;
       }
       if (nxt_val > cur_val) {
         cur = nxt; cur_val = nxt_val;
       } else {
         break;
       }
     }
    return {cur, cur_val};
}


std::pair<Eigen::Vector2d, double> TebOptimalPlanner::findModifiedPose(const Eigen::Vector2d& coll_pt, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3, std::ostream& log, double max_iterations)
{
    const auto& info = *costmap_info_;
    const auto& df = *distance_field_;
    const double max_dist = 0.28;

    ROS_INFO("findmodifiedpose");

    Eigen::Vector2d boundary = getBoundaryPointFromCollision(coll_pt);
    double boundary_val = distanceFieldAt(boundary.x(), boundary.y());
    double coll_val = distanceFieldAt(coll_pt.x(), coll_pt.y());

    log << "[Target Point]\n";
    log << "  World Coord: (" << coll_pt.x() << ", " << coll_pt.y() << ")\n";
    log << "  Distance: " << coll_val << "\n";

    visualization_->visualizetwoPoint({boundary.x(),boundary.y()}, {coll_pt.x(),coll_pt.y()});
    //visualization_->visualizeEndPoints(p2, p3);

    Eigen::Vector2d n;
    if (coll_val == 0.0)
    {
        // case: Boundary value = 0
        n = estimateNormal(coll_pt);
        if (n.norm() == 0.0)
            return {coll_pt, coll_val};
    }
    else if (coll_val < 0.35)
    {
        // case: Boundary value != 0
        n = computePushDirection(coll_pt, boundary, max_dist);
        if (coll_val > 0.0) n = -n;
    }

    Eigen::Vector2d pt_fwd = boundary + n * max_dist;
    auto line = bresenhamLineWorld(boundary, pt_fwd);
    auto result = climbLocalMax(line, max_dist);

    visualization_->visualizeLine({boundary.x(),boundary.y()}, {pt_fwd.x(),pt_fwd.y()});
    //std::cin.get();

    Eigen::Vector2i bc = line[result.first];
    Eigen::Vector2d best_pt(info.origin_x + (bc.x() + 0.5) * info.resolution, info.origin_y + (bc.y() + 0.5) * info.resolution);

    log << "[Line Distances]\n";
    for (size_t i = 0; i < line.size(); ++i)
    {
      const auto& cell = line[i];
      double val = df[cell.x() + cell.y() * info.map_width] * info.resolution;
      double wx = info.origin_x + (cell.x() + 0.5) * info.resolution;
      double wy = info.origin_y + (cell.y() + 0.5) * info.resolution;
      log << "  idx " << i << ": (" << wx << ", " << wy << ") → dist = " << val << "\n";
    }
    log << "  World Coord: (" << best_pt.x() << ", " << best_pt.y() << ")\n";
    log << "  Chosen Distance: " << result.second << "\n";

    std_msgs::ColorRGBA blue;
    blue.r = 0.0;
    blue.g = 0.0;
    blue.b = 1.0;
    blue.a = 1.0;
    visualization_->publishArrow(coll_pt, best_pt, blue);
    //std::cin.get();

    return {best_pt, result.second};

}

// helpers for arc‐length interpolation
double TebOptimalPlanner::normalizeTheta(double ang) {
  while (ang >= M_PI)  ang -= 2*M_PI;
  while (ang < -M_PI)  ang += 2*M_PI;
  return ang;
}

double TebOptimalPlanner::computeArcLength(const PoseSE2& p1, const PoseSE2& p2) {
  // 1) chord length
  double dx = p2.x() - p1.x(), dy = p2.y() - p1.y();
  double c = std::hypot(dx, dy);
  // 2) heading difference
  double dtheta = normalizeTheta(p2.theta() - p1.theta());
  if (std::fabs(dtheta) < 1e-6) return c;
  // 3) radius from chord & angle
  double R = c / (2.0 * std::sin(dtheta * 0.5));
  // 4) arc length
  return std::fabs(R * dtheta);
}

double TebOptimalPlanner::distanceFieldAt(double wx, double wy) const
{
    const auto& info = *costmap_info_;
    const auto& df   = *distance_field_;
    int w = info.map_width;
    int h = info.map_height;
    double res = info.resolution;
    double ox = info.origin_x;
    double oy = info.origin_y;

    int grid_x = static_cast<int>((wx - ox) / res);
    int grid_y = static_cast<int>((wy - oy) / res);
    int idx = grid_x + grid_y * w;

    return df[idx] * res;
}

double TebOptimalPlanner::euclideanDistance(const PoseSE2& p1, const PoseSE2& p2)
{
    double dx = p1.x() - p2.x();
    double dy = p1.y() - p2.y();
    return std::hypot(dx, dy);
}

PoseSE2 TebOptimalPlanner::interpolatePose(const PoseSE2& A, const PoseSE2& B, double frac)
{
  PoseSE2 out;
  out.x() = A.x() + frac * (B.x() - A.x());
  out.y() = A.y() + frac * (B.y() - A.y());

  double delta_theta = g2o::normalize_theta(B.theta() - A.theta());
  out.theta() =g2o::normalize_theta(A.theta() + frac * delta_theta);

  return out;
}

//non-covered bisetion
//SegmentRefineResult TebOptimalPlanner::bisectSegmentLocal(const PoseSE2& p_start, const PoseSE2& p_end, double dt,
//    base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
//    double inscribed_radius, double circumscribed_radius, bool is_root, int depth)
//{
//
//  SegmentRefineResult result;
//  result.poses = { p_start, p_end };
//  result.dts   = { dt };
//
//  // 1) arc length
//  double L = computeArcLength(p_start, p_end);
//  if (L < 0.001)
//    return result;
//
//  // 2) obstacle distances
//  const auto& info = *costmap_info_;
//  const auto& df   = *distance_field_;
//
//  double d1 = distanceFieldAt(p_start.x(), p_start.y());
//  double d2 = distanceFieldAt(p_end.x(), p_end.y());
//
//  double L_eu = euclideanDistance(p_start, p_end);
//
//  // 3) covered region check using arc length (exact collision check)
//  if (L - (d1 + d2) <= 0.0)
//  {
//    return result;
//  }
//  ROS_INFO("depth :%d", depth);
//
//  // 4) compute fractions using euclidean distance
//  double f_start = d1 / L_eu;
//  double f_end   = 1.0 - d2 / L_eu;
//  double f_mid   = 0.5 * (f_start + f_end);
//
//  // 5) interpolation
//  auto interp = [&](const PoseSE2& A, const PoseSE2& B, double frac){
//    PoseSE2 out;
//    out.x() = A.x() + frac * (B.x() - A.x());
//    out.y() = A.y() + frac * (B.y() - A.y());
//    out.theta() = normalizeTheta(A.theta() + frac * normalizeTheta(B.theta() - A.theta()));
//    return out;
//  };
//
//  PoseSE2 A = interp(p_start, p_end, f_start);
//  PoseSE2 M = interp(p_start, p_end, f_mid);
//  PoseSE2 B = interp(p_start, p_end, f_end);
//
//  ROS_INFO("distance of A : %f distance of M : %f distance of B : %f", distanceFieldAt(A.x(), A.y()), distanceFieldAt(M.x(), M.y()), distanceFieldAt(B.x(), B.y()));
//  ROS_INFO("start position x :%f, y: %f  mid position x :%f, y: %f end position x :%f, y: %f ", A.x(), A.y(), M.x(), M.y(), B.x(), B.y());
//
//  // 6) mid collision → medial-ball
//  if (distanceFieldAt(M.x(), M.y()) < 0.2)
//  {
//    auto mp = getModifiedPosition(Eigen::Vector2d(M.x(), M.y()));
//    M.x() = mp.x();
//    M.y() = mp.y();
//    ROS_INFO("new mid position x :%f, y: %f ",M.x(), M.y());
//    ROS_INFO("new distance : %f", distanceFieldAt(M.x(), M.y()));
//  }
//
//  // 7) time split
//  double span = f_end - f_start;
//
//  if (span <= 1e-3)
//    return result;
//
//  double dt1 = dt * ((f_mid - f_start) / span);
//  double dt2 = dt - dt1;
//
//  // 8) Recursive Call (A→M), (M→B)
//  SegmentRefineResult left = bisectSegmentLocal(A, M, dt1, costmap_model, footprint_spec, inscribed_radius, circumscribed_radius, false, depth+1);
//  SegmentRefineResult right = bisectSegmentLocal(M, B, dt2, costmap_model,footprint_spec, inscribed_radius, circumscribed_radius, false, depth+1);
//
//  // 9) Merging
//  if (left.poses.size() == 3)
//  {
//    result.poses.insert(result.poses.begin() + 1, left.poses[1]);
//    // @ToDo adjust timing as needed
//  }
//
//  // insert M at middle (after left.poses[1] if exists, else after start)
//  {
//    int insert_idx = (result.poses.size() == 2) ? 1 : 2;
//    result.poses.insert(result.poses.begin() + insert_idx, M);
//    // @ToDo adjust timing as needed
//  }
//
//  // insert right.poses[1] before end
//  if (right.poses.size() == 3)
//  {
//    result.poses.insert(result.poses.end() - 1, right.poses[1]);
//    // @ToDo adjust timing as needed
//  }
//
//  // @ToDo adjust timing as needed
//
//
//  for (size_t i = 0; i < result.poses.size(); ++i)
//  {
//    const auto& pose = result.poses[i];
//    ROS_INFO("Pose[%zu] → x: %.3f, y: %.3f, theta: %.3f", i, pose.x(), pose.y(), pose.theta());
//  }
//  return result;
//}

//bisection
SegmentRefineResult TebOptimalPlanner::bisectSegmentLocal(PoseSE2& p_start, PoseSE2& p_end, double dt, base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
    double inscribed_radius, double circumscribed_radius, bool is_root, int depth, std::ostream& log)
{
  SegmentRefineResult result;
  result.poses = { p_start, p_end };
  result.dts   = { dt };
  // 1) arc length
  double L = computeArcLength(p_start, p_end);

  if (L < 0.02)
    return result;

  // 2) obstacle distances
  const auto& info = *costmap_info_;
  const auto& df   = *distance_field_;

  double d1 = distanceFieldAt(p_start.x(), p_start.y());
  double d2 = distanceFieldAt(p_end.x(), p_end.y());

  double L_eu = euclideanDistance(p_start, p_end);
  //log << "depth : " << depth << "\n";
  // 3) covered region check using arc length (exact collision check)
  if (L - (d1 + d2) <= 0.0)
  {
    return result;
  }

  // 4) compute mid pose
  PoseSE2 p_mid = interpolatePose(p_start, p_end, 0.5);
  //p_mid.theta() = computeOri({p_mid.x(), p_mid.y()}, {p_end.x(), p_end.y()});

  double arc1 = computeArcLength(p_start,p_mid);
  double arc2 = computeArcLength(p_mid,p_end);
  double mid_dt1 = dt * 0.5 ;//dt * (arc1 / L);
  double mid_dt2 = dt * 0.5 ;//dt * (arc2 / L);

  if(std::min(mid_dt1, mid_dt2) <= 0.05)
      return result;

  //double c = costmap_model->footprintCost(p_mid.x(), p_mid.y(), p_mid.theta(), footprint_spec, inscribed_radius, circumscribed_radius);
  double dist = distanceFieldAt(p_mid.x(), p_mid.y());
  // 5) mid collision → medial-ball
  if (dist < 0.3)
  {
    auto [mp, r] = findModifiedPose(Eigen::Vector2d(p_mid.x(), p_mid.y()), Eigen::Vector2d(p_start.x(), p_start.y()) , Eigen::Vector2d(p_end.x(), p_end.y()), log);
    //auto [mp, r] = findPerpMedialAxis(Eigen::Vector2d(p_mid.x(), p_mid.y()), Eigen::Vector2d(p_start.x(), p_start.y()) , Eigen::Vector2d(p_end.x(), p_end.y()), log);
    //auto [mp, r] = findMedialBallCenter(Eigen::Vector2d(p_mid.x(), p_mid.y()), df, info);
    p_mid.x() = mp.x();
    p_mid.y() = mp.y();

  }
  // auto [theta1, theta3] = estimateTheta({p_start.x(), p_start.y()}, {p_mid.x(), p_mid.y()},{p_end.x(), p_end.y()});
  // p_start.theta() = theta1;
  // p_end.theta() = theta3;

  //visualization_->visualizePoint({p_start.x(),p_start.y()}, {p_end.x(),p_end.y()}, {p_mid.x(),p_mid.y()});

  // 6) Recursive Call (A→M), (M→B)
  SegmentRefineResult left = bisectSegmentLocal(p_start, p_mid, mid_dt1, costmap_model, footprint_spec, inscribed_radius, circumscribed_radius, false, depth+1, log);
  SegmentRefineResult right = bisectSegmentLocal(p_mid, p_end, mid_dt2, costmap_model,footprint_spec, inscribed_radius, circumscribed_radius, false, depth+1, log);

  // 7) Merging
  result.poses.clear();
  result.dts.clear();

  // start point
  result.poses.push_back(p_start);
  //log << "Left Result" << "\n";
  if (left.poses.size() > 2)
  {
    result.poses.insert(result.poses.end(), left.poses.begin() + 1, left.poses.end() - 1);
  }
//  for (size_t i = 0; i < left.poses.size(); ++i)
//  {
//    const auto& p = left.poses[i];
//    log << "Left idx " << i << ": x = " << p.x() << ", y = " << p.y() << ", theta = " << p.theta() << "\n";
//  }

  // mid point
  result.poses.push_back(p_mid);
  //log << "Mid " << ": x = " << p_mid.x() << ", y = " << p_mid.y() << ", theta = " << p_mid.theta() << "\n";
  result.dts.insert(result.dts.end(), left.dts.begin(), left.dts.end());

  // insert right.poses[1] before end
  if (right.poses.size() > 2)
  {
    result.poses.insert(result.poses.end(), right.poses.begin() + 1, right.poses.end() - 1);
    //ROS_INFO("RIGHT");
  }
  //log << "Right Result" << "\n";
//  for (size_t i = 0; i < right.poses.size(); ++i)
//  {
//    const auto& p = right.poses[i];
//    log << "Right idx " << i << ": x = " << p.x() << ", y = " << p.y() << ", theta = " << p.theta() << "\n";
//  }
  result.dts.insert(result.dts.end(), right.dts.begin(), right.dts.end());

  // end point
  result.poses.push_back(p_end);
  //log << "Total Result" << "\n";
//  for (size_t i = 0; i < result.poses.size(); ++i)
//  {
//    const auto& p = result.poses[i];
//    log << "Total idx " << i << ": x = " << p.x() << ", y = " << p.y() << ", theta = " << p.theta() << "\n";
//  }
  for (size_t i = 0; i < result.poses.size(); ++i)
  {
    const auto& pose = result.poses[i];
  }
  for (size_t i = 0; i < result.dts.size(); ++i)
  {
    const auto& dt = result.dts[i];
  }

  return result;
}

double TebOptimalPlanner::computeOri(const Eigen::Vector2d& from, const Eigen::Vector2d& to)
{
    double dx = to.x() - from.x();
    double dy = to.y() - from.y();
    return std::atan2(dy, dx);
}

void TebOptimalPlanner::dumpDistanceMap(const std::vector<float>& distance_field, const DistanceMapInfo& info)
{

    // 1) 타임스탬프 생성 (초 단위)
    std::time_t now = std::time(nullptr);
    // 예: 1712212345  같은 정수 문자열
    std::string ts = std::to_string(now);

    // 2) 파일명 조합
    std::string filename = "/home/glab/distance_map" + ts + ".txt";

    // 3) append 모드로 열기
    std::ofstream outFile(filename, std::ios::app);

    // 5) 포맷 세팅
    outFile << std::fixed << std::setprecision(4);

    // 6) 실제 데이터 덤프
    for (int j = 0; j < info.map_height; ++j) {
      for (int i = 0; i < info.map_width; ++i) {
        int idx = j * info.map_width + i;
        outFile << distance_field[idx];
        if (i + 1 < info.map_width) outFile << ' ';
      }
      outFile << '\n';
    }
    outFile << '\n';

    // 7) 닫기
    outFile.close();
}

std::pair<double, double> TebOptimalPlanner::estimateTheta(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3)
{
    // 1. 중점 계산
    Eigen::Vector2d mid1 = 0.5 * (p1 + p2);
    Eigen::Vector2d mid2 = 0.5 * (p2 + p3);

    // 2. 방향 벡터
    Eigen::Vector2d dir1 = p2 - p1;
    Eigen::Vector2d dir2 = p3 - p2;

    // 3. 수직 벡터 (법선 방향)
    Eigen::Vector2d perp1(-dir1.y(), dir1.x());
    Eigen::Vector2d perp2(-dir2.y(), dir2.x());

    // 4. 교점 계산 (원 중심)
    Eigen::Matrix2d A;
    A << perp1.x(), -perp2.x(),
         perp1.y(), -perp2.y();
    Eigen::Vector2d b = mid2 - mid1;

    Eigen::Vector2d t = A.colPivHouseholderQr().solve(b);
    Eigen::Vector2d center = mid1 + t(0) * perp1;

    // 5. 접선 벡터 at p1, p3
    Eigen::Vector2d r1 = p1 - center;
    Eigen::Vector2d r3 = p3 - center;

    Eigen::Vector2d t1(-r1.y(), r1.x());  // tangent at p1 (시계 방향 기준)
    Eigen::Vector2d t3(-r3.y(), r3.x());  // tangent at p3

    // 방향 보정: t가 다음 점 방향을 향하도록
    if ((p2 - p1).dot(t1) < 0) t1 = -t1;
    if ((p2 - p3).dot(t3) < 0) t3 = -t3;

    // 6. θ 추정
    double theta1 = std::atan2(t1.y(), t1.x());
    double theta3 = std::atan2(t3.y(), t3.x());

    return {theta1, theta3};
}

//------------------------------
// Residual for arc constraint
//------------------------------
// struct SmoothArcConstraintCost
// {
//   SmoothArcConstraintCost(const Eigen::Vector2d& pi, const Eigen::Vector2d& pi1)
//     : p_i(pi), p_i1(pi1) {}

//   template <typename T>
//   bool operator()(const T* const theta_i, const T* const theta_i1, T* residual) const
//   {
//     // heading vector sum
//     Eigen::Matrix<T, 3, 1> g_i, g_i1;
//     g_i << cos(theta_i[0]), sin(theta_i[0]), T(0.0);
//     g_i1 << cos(theta_i1[0]), sin(theta_i1[0]), T(0.0);
//     Eigen::Matrix<T, 3, 1> g_sum = g_i + g_i1;

//     // direction vector between p_i -> p_i1
//     Eigen::Matrix<T, 3, 1> d;
//     d << T(p_i1.x() - p_i.x()), T(p_i1.y() - p_i.y()), T(0.0);

//     // residual = cross product (should be zero vector)
//     Eigen::Matrix<T, 3, 1> cross_prod = g_sum.cross(d);

//     residual[0] = cross_prod[0];
//     residual[1] = cross_prod[1];
//     residual[2] = cross_prod[2];

//     return true;
//   }

//   const Eigen::Vector2d p_i;
//   const Eigen::Vector2d p_i1;
// };

// //------------------------------
// // Main optimization function
// //------------------------------
// void TebOptimalPlanner::optimizeOrientations(const std::vector<Eigen::Vector2d>& positions, std::vector<double>& thetas)
// {
//   const int N = positions.size();
//   if (N < 2) return;

//   thetas.resize(N);

//   for (int i = 0; i < N - 1; ++i)
//   {
//     Eigen::Vector2d diff = positions[i+1] - positions[i];
//     thetas[i] = std::atan2(diff.y(), diff.x());
//   }
//   thetas[N-1] = thetas[N-2];  // 마지막은 앞값과 동일하게 초기화

//   ceres::Problem problem;

//   for (int i = 0; i < N - 1; ++i)
//   {
//     ceres::CostFunction* cost_function =
//       new ceres::AutoDiffCostFunction<SmoothArcConstraintCost, 3, 1, 1>(
//         new SmoothArcConstraintCost(positions[i], positions[i+1]));

//     problem.AddResidualBlock(cost_function, nullptr, &thetas[i], &thetas[i+1]);
//   }

//   // [선택사항] 시작 또는 끝 orientation 고정
//   // problem.SetParameterBlockConstant(&thetas.front());
//   // problem.SetParameterBlockConstant(&thetas.back());

//   ceres::Solver::Options options;
//   options.linear_solver_type = ceres::DENSE_QR;
//   options.minimizer_progress_to_stdout = true;

//   ceres::Solver::Summary summary;
//   ceres::Solve(options, &problem, &summary);

//   std::cout << summary.BriefReport() << std::endl;
// } 

//------------------------------
// Arc constraint residual (cross product)
//------------------------------
struct SmoothArcConstraintCost
{
  SmoothArcConstraintCost(const Eigen::Vector2d& pi, const Eigen::Vector2d& pi1)
    : p_i(pi), p_i1(pi1) {}

  template <typename T>
  bool operator()(const T* const theta_i, const T* const theta_i1, T* residual) const
  {
    // heading vector sum
    Eigen::Matrix<T, 3, 1> g_i, g_i1;
    g_i << cos(theta_i[0]), sin(theta_i[0]), T(0.0);
    g_i1 << cos(theta_i1[0]), sin(theta_i1[0]), T(0.0);
    Eigen::Matrix<T, 3, 1> g_sum = g_i + g_i1;

    // direction vector between p_i -> p_i1
    Eigen::Matrix<T, 3, 1> d;
    d << T(p_i1.x() - p_i.x()), T(p_i1.y() - p_i.y()), T(0.0);

    // residual = cross product (should be zero vector)
    Eigen::Matrix<T, 3, 1> cross_prod = g_sum.cross(d);

    residual[0] = cross_prod[0];
    residual[1] = cross_prod[1];
    residual[2] = cross_prod[2];

    return true;
  }

  const Eigen::Vector2d p_i;
  const Eigen::Vector2d p_i1;
};

//------------------------------
// Heading alignment residual (dot product)
//------------------------------
struct ForwardHeadingConstraintCost
{
  ForwardHeadingConstraintCost(const Eigen::Vector2d& pi, const Eigen::Vector2d& pi1)
    : p_i(pi), p_i1(pi1) {}

  template <typename T>
  bool operator()(const T* const theta_i, T* residual) const
  {
    // Direction vector
    T dx = T(p_i1.x() - p_i.x());
    T dy = T(p_i1.y() - p_i.y());

    // Normalize direction vector
    T norm = sqrt(dx * dx + dy * dy) + T(1e-6);
    dx /= norm;
    dy /= norm;

    // Heading vector
    T cos_theta = cos(theta_i[0]);
    T sin_theta = sin(theta_i[0]);

    // Residual: 1 - dot product (cos of angle between heading and direction)
    T dot = dx * cos_theta + dy * sin_theta;
    residual[0] = T(1.0) - dot;

    return true;
  }

  const Eigen::Vector2d p_i;
  const Eigen::Vector2d p_i1;
};

//------------------------------
// Main optimization function
//------------------------------
void TebOptimalPlanner::optimizeOrientations(const std::vector<Eigen::Vector2d>& positions, std::vector<double>& thetas)
{
  const int N = positions.size();
  if (N < 2) return;

  thetas.resize(N);

  // Initialize orientation based on position differences
  for (int i = 0; i < N - 1; ++i)
  {
    Eigen::Vector2d diff = positions[i + 1] - positions[i];
    thetas[i] = std::atan2(diff.y(), diff.x());
  }
  thetas[N - 1] = thetas[N - 2];  // Copy last orientation

  ceres::Problem problem;

  for (int i = 0; i < N - 1; ++i)
  {
    // Arc constraint
    ceres::CostFunction* arc_cost =
      new ceres::AutoDiffCostFunction<SmoothArcConstraintCost, 3, 1, 1>(
        new SmoothArcConstraintCost(positions[i], positions[i + 1]));

    problem.AddResidualBlock(arc_cost, nullptr, &thetas[i], &thetas[i + 1]);

    // Heading alignment constraint
    ceres::CostFunction* heading_cost =
      new ceres::AutoDiffCostFunction<ForwardHeadingConstraintCost, 1, 1>(
        new ForwardHeadingConstraintCost(positions[i], positions[i + 1]));

    double heading_weight = 1.0;
    problem.AddResidualBlock(heading_cost,
      new ceres::ScaledLoss(nullptr, heading_weight, ceres::TAKE_OWNERSHIP),
      &thetas[i]);
  }

  // Optionally fix final pose orientation
  // problem.SetParameterBlockConstant(&thetas.front());
  problem.SetParameterBlockConstant(&thetas.back());

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << std::endl;
}


bool TebOptimalPlanner::isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
    double inscribed_radius, double circumscribed_radius, int look_ahead_idx, double feasibility_check_lookahead_distance)
{
  const auto& info = *costmap_info_;
  const auto& df   = *distance_field_;
  safe_points_.clear();
  std::ofstream outFile("/home/glab/bisection_log.txt", std::ios::app);
  for (size_t idx = 0; idx < teb().sizePoses(); ++idx)
  {
    if (outFile.is_open())
    {
      outFile << "----- Initial TEB pose -----\n";
      outFile << "TEB pose idx : " << idx << " -> poses = " << teb().Pose(idx) << "\n";
    }
    else
    {
      std::cerr << "File not opened\n";
    }
  }
  for (size_t idx = 0; idx <teb().sizePoses()-1 ; ++idx)
  {
    if (outFile.is_open())
    {
      outFile << "----- Initial TEB Time diff -----\n";
      outFile << "time diff idx : " << idx << " -> dt = " << teb().TimeDiff(idx) << "\n";
    }
    else
    {
      std::cerr << "File not opened\n";
    }
  }
 std::vector<PoseSE2> pose_list;
 std_msgs::ColorRGBA orange;
 orange.r = 1.0;
 orange.g = 0.5;
 orange.b = 0.0;
 orange.a = 1.0;

 for (int i = 0; i < teb().sizePoses(); ++i)
 {
   pose_list.push_back(teb().Pose(i));
 }

 visualization_->visualizeTebPoses(pose_list, orange);
 //std::cin.get();
 const double min_distance_threshold = 0.05;
  //dumpDistanceMap(df, info);
  // 1) initial pose collision → correction
  for (int i = 1; i < teb().sizePoses()-1;)
  {
    auto &p = teb().Pose(i);
    double c = costmap_model->footprintCost(p.x(), p.y(), p.theta(), footprint_spec, inscribed_radius, circumscribed_radius);
    double dist = distanceFieldAt(teb().Pose(i).x(), teb().Pose(i).y());
    if (c <= -1.0){
      //visualization_->publishInfeasibleRobotPose(teb().Pose(i), *cfg_->robot_model, footprint_spec);
      auto [mp, r] = findModifiedPose(Eigen::Vector2d(p.x(), p.y()), Eigen::Vector2d(teb().Pose(i-1).x(), teb().Pose(i-1).y()) , Eigen::Vector2d(p.x(), p.y()), outFile);
      //auto [mp, r] = findPerpMedialAxis(Eigen::Vector2d(p.x(), p.y()), Eigen::Vector2d(teb().Pose(i-1).x(), teb().Pose(i-1).y()) , Eigen::Vector2d(p.x(), p.y()), outFile);
      //auto [mp, r] = findMedialBallCenter(Eigen::Vector2d(p.x(), p.y()), df, info);

      //Modify Pose
      teb().Pose(i).x() = mp.x();
      teb().Pose(i).y() = mp.y();

      teb().Pose(i).theta() = computeOri({teb().Pose(i-1).x(), teb().Pose(i-1).y()}, {teb().Pose(i).x(), teb().Pose(i).y()});


      // 앞뒤 pose 거리 계산
      double dist_prev = (mp - Eigen::Vector2d(teb().Pose(i - 1).x(), teb().Pose(i - 1).y())).norm();
      double dist_next = (mp - Eigen::Vector2d(teb().Pose(i + 1).x(), teb().Pose(i + 1).y())).norm();

      // 너무 가까우면 삭제
      if (dist_prev < min_distance_threshold || dist_next < min_distance_threshold)
      {
        teb().deletePose(i);
        continue;  // 삭제했으므로 index 유지
      }
      
      //safe_points_.emplace_back(mp);
    }
    ++i;
  }

  std::vector<PoseSE2> pose_list2;
  std_msgs::ColorRGBA green;
  green.r = 0.0;
  green.g = 1.0;
  green.b = 0.5;
  green.a = 1.0;

  for (int i = 0; i < teb().sizePoses(); ++i)
  {
    pose_list2.push_back(teb().Pose(i));
  }
  visualization_->visualizeTebPoses(pose_list2, green);
  //std::cin.get();

  for (size_t idx = 0; idx < teb().sizePoses(); ++idx)
  {
    if (outFile.is_open())
    {
      outFile << "----- Modified TEB pose -----\n";
      outFile << "TEB pose idx : " << idx << " -> poses = " << teb().Pose(idx) << "\n";
    }
    else
    {
      std::cerr << "File not opened\n";
    }
  }
  for (size_t idx = 0; idx <teb().sizePoses()-1 ; ++idx)
  {
    if (outFile.is_open())
    {
      outFile << "----- Modified TEB Time diff -----\n";
      outFile << "time diff idx : " << idx << " -> dt = " << teb().TimeDiff(idx) << "\n";
    }
    else
    {
      std::cerr << "File not opened\n";
    }
  }


  // 2) build per-segment local vectors
  size_t M = teb().sizePoses()>0 ? teb().sizePoses()-1 : 0;
  std::vector<std::vector<PoseSE2>> segmentPoses(M);
  std::vector<std::vector<double>>  segmentTimeDiffs(M);
  std::vector<double> segLengths(M, 0.0);
  for (size_t i=0; i<M; ++i) {
    segmentPoses[i]     = { teb().Pose(i), teb().Pose(i+1) };
    segmentTimeDiffs[i] = { teb().TimeDiff(i) };
    segLengths[i] = computeArcLength(segmentPoses[i][0], segmentPoses[i][1]);
  }

  // 3) sort by descending arc‐length
  std::vector<size_t> arc_order(M);
  std::iota(arc_order.begin(), arc_order.end(), 0);
  std::sort(arc_order.begin(), arc_order.end(), [&](size_t a, size_t b){return segLengths[a] > segLengths[b];});

  // 4) refine each segment locally
  int depth = 0;
  for (size_t k : arc_order) {
    PoseSE2& s = segmentPoses[k][0];
    PoseSE2& e = segmentPoses[k][1];
    double dt = segmentTimeDiffs[k][0];

    SegmentRefineResult r = bisectSegmentLocal(s, e, dt, costmap_model, footprint_spec, inscribed_radius, circumscribed_radius, true, depth, outFile);
    // r.poses: [s, mid..., e]
    if (r.poses.size() >= 2 && r.dts.size() == r.poses.size()-1)
    {
      segmentPoses[k]     = std::move(r.poses);
      segmentTimeDiffs[k] = std::move(r.dts);
    }
  }

  // 5) apply all insertions back into teb()
  int offset = 0;
  int gidx = 0;
  for (size_t i = 0; i < segmentPoses.size(); ++i)
  {
    auto& P = segmentPoses[i];
    auto& T = segmentTimeDiffs[i];
    if (outFile.is_open()) {
      outFile << "----- Segment Poses and Dts -----\n";
      for (size_t k = 0; k < P.size(); ++k) {
        outFile << "  seg " << i << "pose" << k << ": "<< P[k].x()<< P[k].y() << P[k].theta() <<"\n";
      }
      for (size_t k = 0; k < T.size(); ++k) {
        outFile <<"  seg " << i << "dt" << k << ": "<< T[k] <<"\n";
      }
    } else {
      std::cerr << "Failed to open file for writing." << std::endl;
    }

    teb().Pose(offset + gidx) = P[0];
    teb().TimeDiff(offset + gidx) = T[0];

    ROS_INFO("insert back");

    for (size_t j = 1; j < P.size()-1; ++j)
    {
      teb().insertPose(offset + gidx + 1, P[j]);
      teb().insertTimeDiff(offset + gidx + 1, T[j-1]);
      double dist_prev = (Eigen::Vector2d(teb().Pose(offset + gidx + 1).x(), teb().Pose(offset + gidx + 1).y()) - Eigen::Vector2d(teb().Pose(offset + gidx).x(), teb().Pose(offset + gidx).y())).norm();
      if (dist_prev > 0.05)
      {
        ROS_INFO("Compute orientation of inserted pose");
        teb().Pose(offset + gidx + 1).theta() = computeOri({teb().Pose(offset + gidx).x(), teb().Pose(offset + gidx).y()}, {teb().Pose(offset + gidx + 1).x(), teb().Pose(offset + gidx + 1).y()});
      }
      ++offset; 
    }
    ++gidx;

  }

  //teb().Pose(0).theta() = computeOri({teb().Pose(0).x(), teb().Pose(0).y()}, {teb().Pose(1).x(), teb().Pose(1).y()});

  // for(int i = 1; i < teb().sizePoses(); ++i)
  // {
  //   teb().Pose(i).theta() = computeOri({teb().Pose(i-1).x(), teb().Pose(i-1).y()}, {teb().Pose(i).x(), teb().Pose(i).y()});
  // }
  


  // std::vector<Eigen::Vector2d> positions;
  // std::vector<double> thetas;

  // for (int i = 0; i < teb_.sizePoses(); ++i)
  // {
  //     const auto& pose = teb_.Pose(i);
  //     positions.emplace_back(pose.x(), pose.y());
  // }

  // // 2. Orientation 최적화

  // optimizeOrientations(positions, thetas);

  // // 3. theta 결과를 teb_에 다시 적용
  // for (int i = 0; i < teb_.sizePoses(); ++i)
  // {
  //     teb_.Pose(i).theta() = thetas[i];
  // }

  std_msgs::ColorRGBA blue;
  blue.r = 0.0;
  blue.g = 0.0;
  blue.b = 1.0;
  blue.a = 1.0;
  std::vector<PoseSE2> pose_list1;
  for (int i = 0; i < teb().sizePoses(); ++i)
  {
    pose_list1.push_back(teb().Pose(i));
  }
  visualization_->visualizeTebPoses(pose_list1, blue);
  //std::cin.get();

  for (size_t i = 0; i < teb().sizePoses(); ++i)
  {
    if (outFile.is_open())
    {
      outFile << "----- Final TEB pose -----\n";
      outFile << "TEB pose idx : " << i << " -> poses = " << teb().Pose(i) << "\n";
    }
    else
    {
      std::cerr << "File not opened\n";
      break;
    }
  }

  for (size_t i = 0; i < teb().sizePoses()-1; ++i)
  {
    if (outFile.is_open())
    {
      outFile << "----- Final TEB Time diff -----\n";
      outFile << "time diff idx : " << i << " -> dt = " << teb().TimeDiff(i) << "\n";
    }
    else
    {
      std::cerr << "File not opened\n";
    }
  }

  // 6) Update ref_timediffs_ and hyst_timediffs_ based on final TEB
  ref_timediffs_.clear();
  hyst_timediffs_.clear();

  for (size_t i = 0; i < teb().sizeTimeDiffs(); ++i)
  {
    double dt = teb().TimeDiff(i);
    ref_timediffs_.push_back(dt);
    hyst_timediffs_.push_back(0.1 * dt);
  }

  //adaptiveoptimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
  
  return true;
}

} // namespace teb_local_planner