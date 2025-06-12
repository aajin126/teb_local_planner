#ifndef EDGE_MEDIAL_ATTRACTION_H_
#define EDGE_MEDIAL_ATTRACTION_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <Eigen/Core>
#include "g2o/core/base_unary_edge.h"

namespace teb_local_planner
{

/**
 * @class EdgeMedialAttraction
 * @brief Edge defining the cost function for using medial point as positional constraint for each teb pose.
 */
class EdgeMedialAttraction : public BaseTebUnaryEdge<1, const Eigen::Vector2d*, VertexPose>
{
public:
  EdgeMedialAttraction()
  {
    _measurement = NULL;
  }

  /**
   * @brief Compute the error vector.
   * The error is the 2D vector between the current pose and the target medial point.
   */
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig(), setMedialAttraction() on EdgeMedialAttraction()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

    _error[0] = (bandpt->position() - *_measurement).norm();

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeMedialAttraction::computeError() _error[0]=%f \n",_error[0]);
  }

  /**
   * @brief Set the target medial point
   * @param medial_point 2D point the pose should be attracted to
   */
  void setMedialPoint(const Eigen::Vector2d* medial_point)
  {
    _measurement = medial_point;
  }

   /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param medial_point 2D position vector containing the position of the medial_point
   */
  void setParameters(const TebConfig& cfg, const Eigen::Vector2d* medial_point)
  {
    cfg_ = &cfg;
    _measurement = medial_point;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace teb_local_planner

#endif // EDGE_MEDIAL_ATTRACTION_H_
