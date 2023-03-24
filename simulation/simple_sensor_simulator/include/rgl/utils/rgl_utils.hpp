#ifndef RGL__UTILS__RGL_UTILS_HPP__
#define RGL__UTILS__RGL_UTILS_HPP__

#include <geometry_msgs/msg/pose.hpp>
// #include <geometry_msgs/msg/vector3.hpp>
#include <rgl/api/core.h>
#include <Eigen/Core>
// #include <Eigen/Geometry>

namespace rgl
{
/**
 * @brief Creates identity matrix
 * @return Identity matrix wit position [0, 0, 0]
 */
rgl_mat3x4f getRglIdentity();

/**
 * @brief Prints matrix to the standard output
 * @param mat Matrix to print
 */
void printMat(const Eigen::Matrix3d & mat);

/**
 * @brief Prints matrix to the standard output
 * @param mat Matrix to print
 */
void printMat(const rgl_mat3x4f & mat);

/**
 * @brief Converts quaternion rotation to RGL matrix rotation (position is set to [0, 0, 0])
 * @param rotation rotation quaternion
 * @param correct_rgl_bias optional, if set to true applies 90 degree rotation about Y axis
 * @return rotation in matrix rgl_mat3x4f
 */
rgl_mat3x4f getRglMatRotation(const geometry_msgs::msg::Quaternion & rotation, const bool correct_rgl_bias = true);

/**
 * @brief Sets given rotation to RGL matrix with possible correction for RGL base orientation aligned with Z axis
 * @param entity_tf reference to the matrix where changes are applied
 * @param rotation rotation quaternion to apply
 * @param correct_rgl_bias optional, if set to true applies 90 degree rotation about Y axis
 */
void setRglMatRotation(rgl_mat3x4f & entity_tf, geometry_msgs::msg::Quaternion rotation, const bool correct_rgl_bias = true);

/**
 * @brief Sets given position to RGL matrix
 * @param entity_tf reference to the matrix to modify
 * @param position position to set in matrix
 */
void setRglMatPosition(rgl_mat3x4f & entity_tf, const geometry_msgs::msg::Point & position);

/**
 * @brief Converts pose (both position and orientation) to RGL matrix
 * @param entity_tf RGL matrix to change values
 * @param pose pose with values to transfer into RGL matrix
 * @param correct_rgl_bias optional, if set to true applies 90 degree rotation about Y axis
 */
void setRglMatPose(rgl_mat3x4f & entity_tf, const geometry_msgs::msg::Pose & pose, const bool correct_rgl_bias = true);

/**
 * @brief Inverts rotation and converts it to RGL matrix
 * @param entity_tf RGL matrix to modify
 * @param pose pose with rotation to invert
 * @param correct_rgl_bias optional, if set to true applies 90 degree rotation about Y axis
 */
void setRglMatRotationInv(rgl_mat3x4f & entity_tf, const geometry_msgs::msg::Pose & pose, const bool correct_rgl_bias = true);

/**
 * @brief Inverts position and converts it to RGL matrix
 * @param entity_tf RGL matrix to modify
 * @param pose pose with position to invert
 * @param correct_rgl_bias optional, if set to true applies 90 degree rotation about Y axis
 */
void setRglMatPositionInv(rgl_mat3x4f & entity_tf, const geometry_msgs::msg::Pose & pose);

/**
 * @brief Initialize vertices of a box with given dimensions
 * @param vertices reference to an array to write values to
 * @param depth depth of a box
 * @param width width of a box
 * @param height height of a box
 */
void initBoxVertices(rgl_vec3f (&vertices)[8], float depth, float width, float height);

/**
 * @brief Initialize indices of a box (works with initBoxVertices())
 * @param indices reference to an array to write values to
 */
void initBoxIndices(rgl_vec3i (&indices)[12]);
}  // namespace rgl

#endif  // RGL__UTILS__RGL_UTILS_HPP__
