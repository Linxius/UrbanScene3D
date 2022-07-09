#ifndef INTERSECTION_TOOLS_H
#define INTERSECTION_TOOLS_H
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_triangle_primitive.h>

#include "cgal_tools.h"
#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "embree3/rtcore.h"

typedef CGAL::Ray_3<K> Ray;

typedef CGAL::AABB_face_graph_triangle_primitive<
  Surface_mesh,
  CGAL::Default,
  CGAL::Tag_false>
                                        Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_Traits;
typedef CGAL::AABB_tree<AABB_Traits>    Tree;
typedef Tree::Primitive_id              Primitive_id;

typedef CGAL::Search_traits_3<K>                         KDTreeTraits;
typedef CGAL::Orthogonal_k_neighbor_search<KDTreeTraits> Neighbor_search;
typedef Neighbor_search::Tree                            KDTree;

// @brief: Get the depth map by building a bvh tree and query the distance and
// instance label of each mesh
// @notice: Distance is perspective distance, not planar distance!
// @param:
// @ret: `{ img_distance,img_distance_planar, img_distance_perspective} (0
// represent the background, object id start by 1)`
std::tuple<cv::Mat, cv::Mat, cv::Mat>
get_depth_map_through_meshes(
  const std::vector<Surface_mesh>& v_meshes,
  const int                        v_width,
  const int                        v_height,
  const Eigen::Matrix3f&           v_intrinsic
);

const Point_cloud
remove_points_inside(
  const Surface_mesh&         v_mesh,
  const std::vector<Point_3>& v_points
);

RTCScene
generate_embree_scene(const Surface_mesh& v_mesh);
RTCScene
generate_embree_scene(
  const std::vector<Point_3>&            v_vertices,
  const std::vector<std::array<int, 3>>& v_face_index
);

// Deprecation
// bool is_visible(const Eigen::Vector3f& v_view_pos, const Eigen::Vector3f&
// v_view_direction, const Eigen::Vector3f& v_point_pos,const Tree& v_tree,const
// float fov_in_degree_h, const float fov_in_degree_v, float max_distance=-1.f);
bool
is_visible(
  const Eigen::Matrix3f&   v_intrinsic_matrix,
  const Eigen::Isometry3f& v_camera_matrix,
  const Eigen::Vector3f&   v_view_pos,
  const Eigen::Vector3f&   v_point_pos,
  const RTCScene&          v_scene,
  double                   max_distance
);
bool
is_visible(
  const Eigen::Matrix3f&   v_intrinsic_matrix,
  const Eigen::Isometry3f& v_camera_matrix,
  const Eigen::Vector3f&   v_view_pos,
  const Eigen::Vector3f&   v_point_pos,
  const Tree&              v_tree,
  float                    max_distance
);
bool
is_visible(
  const Eigen::Matrix3f&   v_intrinsic_matrix,
  const Eigen::Isometry3f& v_camera_matrix,
  const Eigen::Vector3f&   v_view_pos,
  const Eigen::Vector3f&   v_point_pos,
  const RTCScene&          v_scene,
  double                   max_distance,
  const Surface_mesh&      v_mesh
);

Eigen::Vector3f
get_hit_position(
  const Eigen::Matrix3f&   v_intrinsic_matrix,
  const Eigen::Isometry3f& v_camera_matrix,
  const Eigen::Vector3f&   v_view_pos,
  const Eigen::Vector3f&   v_point_pos,
  const RTCScene&          v_scene,
  double                   max_distance
);

bool
fall_in_camera_region(
  const Eigen::Matrix3f&   v_intrinsic_matrix,
  const Eigen::Isometry3f& v_camera_matrix,
  const Eigen::Vector3f&   v_view_pos,
  const Eigen::Vector3f&   v_point_pos,
  const RTCScene&          v_scene,
  double                   max_distance
);

#endif // INTERSECTION_TOOLS_H
