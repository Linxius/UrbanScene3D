#ifndef CGAL_TOOLS_H
#define CGAL_TOOLS_H
#include <tiny_obj_loader.h>
#include <tuple>

#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/IO/write_ply_points.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Triangle_3.h>
#include <CGAL/polygon_mesh_processing.h>
#include <CGAL/property_map.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Sparse>

#include <opencv2/opencv.hpp>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2                                          Point_2;
typedef K::Point_3                                          Point_3;
typedef CGAL::Polygon_2<K>                                  Polygon_2;
typedef CGAL::Triangle_3<K>                                 Triangle_3;
// typedef CGAL::Point_2<K> Point_2;
// typedef CGAL::Point_3<K> Point_3;
typedef CGAL::Direction_3<K>                                Direction_3;
typedef CGAL::Vector_3<K>                                   Vector_3;
typedef CGAL::Surface_mesh<K::Point_3>                      Surface_mesh;
typedef CGAL::Polygon_2<K>                                  Polygon_2;
typedef CGAL::Point_set_3<K::Point_3>                       Point_cloud;
typedef CGAL::Point_set_3<Point_3>                          Point_set;
typedef Surface_mesh::Face_index                            face_descriptor;
typedef Surface_mesh::Vertex_index                          vertex_descriptor;
typedef Surface_mesh::Halfedge_index                        halfedge_descriptor;

typedef CGAL::Polyhedron_3<K> Polyhedron_3;
typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron_3>
                                                   Polyhedron_Primitive;
typedef CGAL::AABB_traits<K, Polyhedron_Primitive> Polyhedron_Traits;
typedef CGAL::AABB_tree<Polyhedron_Traits>         Polyhedron_Tree;

typedef std::vector<Triangle_3>::iterator                   Triangle_Iterator;
typedef CGAL::AABB_triangle_primitive<K, Triangle_Iterator> Triangle_Primitive;
typedef CGAL::AABB_traits<K, Triangle_Primitive> Triangle_AABB_triangle_traits;
typedef CGAL::AABB_tree<Triangle_AABB_triangle_traits> Triangle_Tree;

struct Rotated_box
{
    cv::RotatedRect     cv_box;
    Eigen::AlignedBox3f box;
    float               angle;
    Rotated_box(){};

    Rotated_box(const Eigen::AlignedBox3f& v_box)
      : box(v_box)
      , angle(0.f)
    {
        cv_box = cv::RotatedRect(
          cv::Point2f(v_box.center().x(), v_box.center().y()),
          cv::Size2f(v_box.sizes().x(), v_box.sizes().y()),
          0.f
        );
    }

    Rotated_box(const Eigen::AlignedBox3f& v_box, float v_angle_in_degree)
      : box(v_box)
      , angle(v_angle_in_degree / 180.f * 3.1415926f)
    {
        cv_box = cv::RotatedRect(
          cv::Point2f(v_box.center().x(), v_box.center().y()),
          cv::Size2f(v_box.sizes().x(), v_box.sizes().y()),
          v_angle_in_degree
        );
    }

    bool inside_2d(const Eigen::Vector3f& v_point) const
    {
        Eigen::Vector2f point(v_point.x(), v_point.y());
        float           s = std::sin(-angle);
        float           c = std::cos(-angle);

        // set origin to rect center
        Eigen::Vector2f newPoint
          = point - Eigen::Vector2f(box.center().x(), box.center().y());
        // rotate
        newPoint = Eigen::Vector2f(
          newPoint.x() * c - newPoint.y() * s,
          newPoint.x() * s + newPoint.y() * c
        );
        // put origin back
        newPoint
          = newPoint + Eigen::Vector2f(box.center().x(), box.center().y());
        if (newPoint.x() >= box.min().x() && newPoint.x() <= box.max().x() && newPoint.y() >= box.min().y() && newPoint.y() <= box.max().y())
            return true;
        else
            return false;
    }
};

// @brief:
// @notice: Currently only transfer vertices to the cgal Surface mesh
// @param: `attrib_t, shape_t, material_t`
// @ret: Surface_mesh
Surface_mesh
convert_obj_from_tinyobjloader_to_surface_mesh(
  const std::tuple<
    tinyobj::attrib_t,
    std::vector<tinyobj::shape_t>,
    std::vector<tinyobj::material_t>> v_obj_in
);

Eigen::AlignedBox3f
get_bounding_box(const Point_set& v_point_set);
Rotated_box
get_bounding_box_rotated(const Point_set& v_point_set);

// Eigen::Vector3f cgal_point_2_eigen(const Point_3& p);
// Eigen::Vector3f cgal_vector_2_eigen(const Vector_3& p);
// Point_3 eigen_2_cgal_point(const Eigen::Vector3f& p);
// Vector_3 eigen_2_cgal_vector(const Eigen::Vector3f& p);
inline Eigen::Vector3f
cgal_point_2_eigen(const Point_3& p)
{
    return Eigen::Vector3f(p.x(), p.y(), p.z());
}

inline Eigen::Vector3f
cgal_vector_2_eigen(const Vector_3& p)
{
    return Eigen::Vector3f(p.x(), p.y(), p.z());
}

inline Point_3
eigen_2_cgal_point(const Eigen::Vector3f& p)
{
    return Point_3(p.x(), p.y(), p.z());
}

inline Vector_3
eigen_2_cgal_vector(const Eigen::Vector3f& p)
{
    return Vector_3(p.x(), p.y(), p.z());
}

#endif // CGAL_TOOLS_H
