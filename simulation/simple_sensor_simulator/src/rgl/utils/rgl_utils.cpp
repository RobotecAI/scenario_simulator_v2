#include <rgl/utils/rgl_utils.hpp>
#include <quaternion_operation/quaternion_operation.h>

namespace rgl
{
rgl_mat3x4f getRglIdentity()
{
  rgl_mat3x4f mat;

  mat.value[0][0] = 1;
  mat.value[1][0] = 0;
  mat.value[2][0] = 0;

  mat.value[0][1] = 0;
  mat.value[1][1] = 1;
  mat.value[2][1] = 0;

  mat.value[0][2] = 0;
  mat.value[1][2] = 0;
  mat.value[2][2] = 1;

  mat.value[0][3] = 0;
  mat.value[1][3] = 0;
  mat.value[2][3] = 0;

  return mat;
}

void printMat(const Eigen::Matrix3d & mat)
{
  printf("%.2f %.2f %.2f\n", mat(0, 0), mat(0, 1), mat(0, 2));
  printf("%.2f %.2f %.2f\n", mat(1, 0), mat(1, 1), mat(1, 2));
  printf("%.2f %.2f %.2f\n", mat(2, 0), mat(2, 1), mat(2, 2));
}

void printMat(const rgl_mat3x4f & mat)
{
  printf("%.2f %.2f %.2f %.2f\n", mat.value[0][0], mat.value[0][1], mat.value[0][2], mat.value[0][3]);
  printf("%.2f %.2f %.2f %.2f\n", mat.value[1][0], mat.value[1][1], mat.value[1][2], mat.value[1][3]);
  printf("%.2f %.2f %.2f %.2f\n", mat.value[2][0], mat.value[2][1], mat.value[2][2], mat.value[2][3]);
}

void setRglMatRotation(rgl_mat3x4f & entity_tf, geometry_msgs::msg::Quaternion rotation, const bool correct_rgl_bias)
{
  if (correct_rgl_bias) {
    geometry_msgs::msg::Vector3 rpy;
    rpy.x = 0;
    rpy.y = M_PI/2;
    rpy.z = 0;
    const auto bias = quaternion_operation::convertEulerAngleToQuaternion(rpy);
    rotation = quaternion_operation::rotation(rotation, bias);   // rotates about a local frame
    // rotation = bias * rotation;   // rotates about a global frame
  }
  const auto rotation_mat = quaternion_operation::getRotationMatrix(rotation);

  for (uint8_t row = 0; row < 3; ++row) {
    for (uint8_t col = 0; col < 3; ++col) {
      // no need to convert from column-major to row-major as () operator already does this
      entity_tf.value[row][col] = static_cast<float>(rotation_mat(row, col));
    }
  }
}

rgl_mat3x4f getRglMatRotation(const geometry_msgs::msg::Quaternion & rotation, const bool correct_rgl_bias)
{
  rgl_mat3x4f tf = getRglIdentity();
  setRglMatRotation(tf, rotation, correct_rgl_bias);
  return tf;
}

void setRglMatPosition(rgl_mat3x4f & entity_tf, const geometry_msgs::msg::Point & position)
{
  entity_tf.value[0][3] = position.x;
  entity_tf.value[1][3] = position.y;
  entity_tf.value[2][3] = position.z;
}

void setRglMatPose(rgl_mat3x4f & entity_tf, const geometry_msgs::msg::Pose & pose, const bool correct_rgl_bias)
{
  setRglMatRotation(entity_tf, pose.orientation, correct_rgl_bias);
  setRglMatPosition(entity_tf, pose.position);
}

void setRglMatPositionInv(rgl_mat3x4f & entity_tf, const geometry_msgs::msg::Pose & pose)
{
  auto pose_inv = pose;
  {
    pose_inv.position.x = -pose.position.x;
    pose_inv.position.y = -pose.position.y;
    pose_inv.position.z = -pose.position.z;
  }
  setRglMatPosition(entity_tf, pose_inv.position);
}

void setRglMatRotationInv(rgl_mat3x4f & entity_tf, const geometry_msgs::msg::Pose & pose, const bool correct_rgl_bias)
{
  auto pose_inv = pose;
  {
    pose_inv.orientation.x = -pose.orientation.x;
    pose_inv.orientation.y = -pose.orientation.y;
    pose_inv.orientation.z = -pose.orientation.z;
  }
  setRglMatRotation(entity_tf, pose_inv.orientation, correct_rgl_bias);
}

void initBoxVertices(rgl_vec3f (&vertices)[8], float depth, float width, float height)
{
  vertices[0].value[0] = -0.5 * depth;
  vertices[0].value[1] = -0.5 * width;
  vertices[0].value[2] = -0.5 * height;

  vertices[1].value[0] = -0.5 * depth;
  vertices[1].value[1] = -0.5 * width;
  vertices[1].value[2] = +0.5 * height;

  vertices[2].value[0] = -0.5 * depth;
  vertices[2].value[1] = +0.5 * width;
  vertices[2].value[2] = -0.5 * height;

  vertices[3].value[0] = -0.5 * depth;
  vertices[3].value[1] = +0.5 * width;
  vertices[3].value[2] = +0.5 * height;

  vertices[4].value[0] = +0.5 * depth;
  vertices[4].value[1] = -0.5 * width;
  vertices[4].value[2] = -0.5 * height;

  vertices[5].value[0] = +0.5 * depth;
  vertices[5].value[1] = -0.5 * width;
  vertices[5].value[2] = +0.5 * height;

  vertices[6].value[0] = +0.5 * depth;
  vertices[6].value[1] = +0.5 * width;
  vertices[6].value[2] = -0.5 * height;

  vertices[7].value[0] = +0.5 * depth;
  vertices[7].value[1] = +0.5 * width;
  vertices[7].value[2] = +0.5 * height;
}

void initBoxIndices(rgl_vec3i (&indices)[12])
{
  indices[0].value[0] = 0;
  indices[0].value[1] = 1;
  indices[0].value[2] = 2;

  indices[1].value[0] = 1;
  indices[1].value[1] = 3;
  indices[1].value[2] = 2;

  indices[2].value[0] = 4;
  indices[2].value[1] = 6;
  indices[2].value[2] = 5;

  indices[3].value[0] = 5;
  indices[3].value[1] = 6;
  indices[3].value[2] = 7;

  indices[4].value[0] = 0;
  indices[4].value[1] = 4;
  indices[4].value[2] = 1;

  indices[5].value[0] = 1;
  indices[5].value[1] = 4;
  indices[5].value[2] = 5;

  indices[6].value[0] = 2;
  indices[6].value[1] = 3;
  indices[6].value[2] = 6;

  indices[7].value[0] = 3;
  indices[7].value[1] = 7;
  indices[7].value[2] = 6;

  indices[8].value[0] = 0;
  indices[8].value[1] = 2;
  indices[8].value[2] = 4;

  indices[9].value[0] = 2;
  indices[9].value[1] = 6;
  indices[9].value[2] = 4;

  indices[10].value[0] = 1;
  indices[10].value[1] = 5;
  indices[10].value[2] = 3;

  indices[11].value[0] = 3;
  indices[11].value[1] = 5;
  indices[11].value[2] = 7;
}
} // namespace rgl
