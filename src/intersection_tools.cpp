#include "intersection_tools.h"

#include <algorithm>

std::tuple<cv::Mat, cv::Mat, cv::Mat>
get_depth_map_through_meshes(
  const std::vector<Surface_mesh>& v_meshes,
  const int                        v_width,
  const int                        v_height,
  const Eigen::Matrix3f&           v_intrinsic
)
{
    std::vector<Surface_mesh> meshes(v_meshes);

    Tree tree;
    for (int i = 0; i < v_meshes.size(); ++i) {
        meshes[i].add_property_map<Surface_mesh::face_index, int>(
          "instance_id", i
        );
        tree.insert(
          CGAL::faces(meshes[i]).first, CGAL::faces(meshes[i]).second, meshes[i]
        );
    }
    tree.build();

    cv::Mat img_instance(v_height, v_width, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat img_distance_perspective(
      v_height, v_width, CV_32FC1, cv::Scalar(0.f)
    );
    cv::Mat img_distance_planar(v_height, v_width, CV_32FC1, cv::Scalar(0.f));

#pragma omp parallel for
    for (int y = 0; y < v_height; y++) {
        for (int x = 0; x < v_width; x++) {
            Eigen::Vector3f p(x, y, 1);
            p = v_intrinsic.inverse() * p;
            Ray  ray(Point_3(0, 0, 0), Vector_3(p.x(), p.y(), p.z()));
            auto result  = tree.first_intersection(ray);
            auto result1 = tree.first_intersected_primitive(ray);
            if (result1) {
                const Surface_mesh::face_index face_index = result1->first;
                const Surface_mesh*            mesh       = result1->second;
                if (mesh) {
                    const Primitive_id& primitive_id
                      = boost::get<Primitive_id>(result->second);
                    for (int id_mesh = 0; id_mesh < meshes.size(); id_mesh++) {
                        Surface_mesh::
                          Property_map<Surface_mesh::face_index, int>
                             gnus_iter;
                        bool found;
                        boost::tie(gnus_iter, found)
                          = meshes[id_mesh]
                              .property_map<Surface_mesh::face_index, int>(
                                "instance_id"
                              );

                        Surface_mesh::
                          Property_map<Surface_mesh::face_index, int>
                            gnus;
                        boost::tie(gnus, found)
                          = mesh->property_map<Surface_mesh::face_index, int>(
                            "instance_id"
                          );

                        if (gnus_iter[*(meshes[id_mesh].faces_begin())] == gnus[face_index])
                            img_instance.at<cv::Vec3b>(y, x)[2] = id_mesh;
                    }
                    img_distance_perspective.at<float>(y, x)
                      = std::sqrt(CGAL::squared_distance(
                        boost::get<Point_3>(result->first), Point_3(0, 0, 0)
                      ));

                    img_distance_planar.at<float>(y, x)
                      = boost::get<Point_3>(result->first)[2];
                }
            }
        }
    }

    // cv::imshow("1", img_instance);
    // cv::waitKey();

    return { img_instance, img_distance_planar, img_distance_perspective };
}

const Point_cloud
remove_points_inside(
  const Surface_mesh&         v_mesh,
  const std::vector<Point_3>& v_points
)
{
    Tree tree;
    tree.insert(CGAL::faces(v_mesh).first, CGAL::faces(v_mesh).second, v_mesh);
    tree.build();

    // Define scanner camera's position
    std::vector<Point_3> camera;
    float                min_x = std::numeric_limits<float>::max();
    float                min_y = std::numeric_limits<float>::max();
    float                min_z = std::numeric_limits<float>::max();
    float                max_z = std::numeric_limits<float>::min();
    float                max_y = std::numeric_limits<float>::min();
    float                max_x = std::numeric_limits<float>::min();

    for (auto vertex : v_mesh.vertices()) {
        min_x
          = v_mesh.point(vertex).x() < min_x ? v_mesh.point(vertex).x() : min_x;
        min_y
          = v_mesh.point(vertex).y() < min_y ? v_mesh.point(vertex).y() : min_y;
        min_z
          = v_mesh.point(vertex).z() < min_z ? v_mesh.point(vertex).z() : min_z;
        max_x
          = v_mesh.point(vertex).x() > max_x ? v_mesh.point(vertex).x() : max_x;
        max_y
          = v_mesh.point(vertex).y() > max_y ? v_mesh.point(vertex).y() : max_y;
        max_z
          = v_mesh.point(vertex).z() > max_z ? v_mesh.point(vertex).z() : max_z;
    }

    float diff_x = max_x - min_x;
    float diff_y = max_y - min_y;
    float diff_z = max_z - min_z;

    float max_diff = std::max({ diff_x, diff_y, diff_z });
    float pad      = max_diff * 5;

    // Back
    camera.emplace_back(min_x - pad, min_y - pad, min_z - pad);
    camera.emplace_back(min_x - pad, (min_y + max_y) / 2, min_z - pad);
    camera.emplace_back(min_x - pad, max_y + pad, min_z - pad);

    camera.emplace_back(min_x - pad, min_y - pad, (min_z + max_z) / 2);
    camera.emplace_back(min_x - pad, (min_y + max_y) / 2, (min_z + max_z) / 2);
    camera.emplace_back(min_x - pad, max_y + pad, (min_z + max_z) / 2);

    camera.emplace_back(min_x - pad, min_y - pad, max_z + pad);
    camera.emplace_back(min_x - pad, (min_y + max_y) / 2, max_z + pad);
    camera.emplace_back(min_x - pad, max_y + pad, max_z + pad);

    // Mid
    camera.emplace_back((min_x + max_x) / 2, min_y - pad, min_z - pad);
    camera.emplace_back((min_x + max_x) / 2, (min_y + max_y) / 2, min_z - pad);
    camera.emplace_back((min_x + max_x) / 2, max_y + pad, min_z - pad);

    camera.emplace_back((min_x + max_x) / 2, min_y - pad, (min_z + max_z) / 2);
    camera.emplace_back((min_x + max_x) / 2, max_y + pad, (min_z + max_z) / 2);

    camera.emplace_back((min_x + max_x) / 2, min_y - pad, max_z + pad);
    camera.emplace_back((min_x + max_x) / 2, (min_y + max_y) / 2, max_z + pad);
    camera.emplace_back((min_x + max_x) / 2, max_y + pad, max_z + pad);

    // Front
    camera.emplace_back(max_x + pad, min_y - pad, min_z - pad);
    camera.emplace_back(max_x + pad, (min_y + max_y) / 2, min_z - pad);
    camera.emplace_back(max_x + pad, max_y + pad, min_z - pad);

    camera.emplace_back(max_x + pad, min_y - pad, (min_z + max_z) / 2);
    camera.emplace_back(max_x + pad, (min_y + max_y) / 2, (min_z + max_z) / 2);
    camera.emplace_back(max_x + pad, max_y + pad, (min_z + max_z) / 2);

    camera.emplace_back(max_x + pad, min_y - pad, max_z + pad);
    camera.emplace_back(max_x + pad, (min_y + max_y) / 2, max_z + pad);
    camera.emplace_back(max_x + pad, max_y + pad, max_z + pad);

    Point_cloud out_points;

    //#pragma omp parallel for
    for (int i = 0; i < v_points.size(); i++) {
        int visible_flag = 0;
        for (int i_camera = 0; i_camera < camera.size(); ++i_camera) {
            Ray ray(v_points[i], camera[i_camera]);
            int result = tree.number_of_intersected_primitives(ray);
            if (result <= 1) { visible_flag += 1; }
        }
        if (visible_flag > 0) {
            //#pragma omp critical
            out_points.insert(v_points[i]);
        }
    }
    return out_points;
}

RTCScene
generate_embree_scene(
  const std::vector<Point_3>&            v_vertices,
  const std::vector<std::array<int, 3>>& v_face_index
)
{
    RTCDevice   g_device = rtcNewDevice(NULL);
    RTCScene    scene    = rtcNewScene(g_device);
    RTCGeometry d_mesh   = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

    struct Vertex
    {
        float x, y, z;
    }; // FIXME: rename to Vertex4f

    struct Triangle
    {
        int v0, v1, v2;
    };

    Vertex* vertices = (Vertex*)rtcSetNewGeometryBuffer(
      d_mesh,
      RTC_BUFFER_TYPE_VERTEX,
      0,
      RTC_FORMAT_FLOAT3,
      sizeof(Vertex),
      v_vertices.size()
    );
    Triangle* triangles = (Triangle*)rtcSetNewGeometryBuffer(
      d_mesh,
      RTC_BUFFER_TYPE_INDEX,
      0,
      RTC_FORMAT_UINT3,
      sizeof(Triangle),
      v_face_index.size()
    );

    int i_index = 0;
    for (const auto& vertex : v_vertices) {
        vertices[i_index].x = vertex.x();
        vertices[i_index].y = vertex.y();
        vertices[i_index].z = vertex.z();
        i_index++;
    }

    i_index = 0;
    for (const auto& face : v_face_index) {
        triangles[i_index].v0 = std::get<0>(face);
        triangles[i_index].v1 = std::get<1>(face);
        triangles[i_index].v2 = std::get<2>(face);
        i_index++;
    }

    rtcCommitGeometry(d_mesh);
    rtcAttachGeometry(scene, d_mesh);
    rtcReleaseGeometry(d_mesh);
    rtcCommitScene(scene);
    return scene;
}

RTCScene
generate_embree_scene(const Surface_mesh& v_mesh)
{
    if (!CGAL::is_triangle_mesh(v_mesh))
        throw "Cannot handle non triangle mesh";
    RTCDevice   g_device = rtcNewDevice(NULL);
    RTCScene    scene    = rtcNewScene(g_device);
    RTCGeometry d_mesh   = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

    struct Vertex
    {
        float x, y, z;
    }; // FIXME: rename to Vertex4f

    struct Triangle
    {
        int v0, v1, v2;
    };

    Vertex* vertices = (Vertex*)rtcSetNewGeometryBuffer(
      d_mesh,
      RTC_BUFFER_TYPE_VERTEX,
      0,
      RTC_FORMAT_FLOAT3,
      sizeof(Vertex),
      v_mesh.num_vertices()
    );
    Triangle* triangles = (Triangle*)rtcSetNewGeometryBuffer(
      d_mesh,
      RTC_BUFFER_TYPE_INDEX,
      0,
      RTC_FORMAT_UINT3,
      sizeof(Triangle),
      v_mesh.num_faces()
    );

    int i_index = 0;
    for (const auto& vertex : v_mesh.points()) {
        vertices[i_index].x = vertex.x();
        vertices[i_index].y = vertex.y();
        vertices[i_index].z = vertex.z();
        i_index++;
    }

    i_index = 0;
    for (const auto& face : v_mesh.faces()) {
        int vertex_index = 0;
        for (const auto item :
             v_mesh.vertices_around_face(v_mesh.halfedge(face))) {
            if (vertex_index == 0) {
                triangles[i_index].v0 = item.idx();
                vertex_index += 1;
            } else if (vertex_index == 1) {
                triangles[i_index].v1 = item.idx();
                vertex_index += 1;
            } else if (vertex_index == 2)
                triangles[i_index].v2 = item.idx();
        }
        i_index++;
    }

    rtcCommitGeometry(d_mesh);
    rtcAttachGeometry(scene, d_mesh);
    rtcReleaseGeometry(d_mesh);
    rtcCommitScene(scene);
    return scene;
}

// Deprecation
// bool is_visible(const Eigen::Vector3f& v_view_pos, const Eigen::Vector3f&
// v_view_direction, 	const Eigen::Vector3f& v_point_pos, const Tree& v_tree,
// const float fov_in_degree_h, const float fov_in_degree_v, float max_distance)
//{
//	const Point_3 point_pos(v_point_pos.x(), v_point_pos.y(), v_point_pos.z());
//	const Point_3 view_pos(v_view_pos.x(), v_view_pos.y(), v_view_pos.z());
//	Vector_3 view_direction(v_view_direction.x(), v_view_direction.y(),
// v_view_direction.z());
//
//	Vector_3 view_to_point = point_pos - view_pos;
//	Vector_3 view_to_point_norm = view_to_point /
// std::sqrt(view_to_point.squared_length()); 	double squared_distance =
// view_to_point.squared_length(); 	if (max_distance != -1.f)
// if(max_distance
// * max_distance < squared_distance) 			return false;
//
//	// Horizontal
//	Vector_3 normal_1 =
// CGAL::cross_product(view_direction,Vector_3(0.f,0.f,1.f)); 	normal_1 /=
// std::sqrt(normal_1.squared_length()); 	double h_delta =
// std::asin(normal_1*view_to_point_norm)/ 3.1415926f * 180.f;
//	//if (h_delta>180.f)
//	//	h_delta = 360.f-h_delta;
//	h_delta=std::abs(h_delta);
//
//	Vector_3 normal_2 = CGAL::cross_product(normal_1,view_direction);
//	normal_2 /= std::sqrt(normal_2.squared_length());
//	double v_delta =  std::abs(std::asin(normal_2*view_to_point_norm)
/// 3.1415926f * 180.f); 	v_delta=std::abs(v_delta);
//
//	if((view_direction*view_to_point_norm) < 0)
//		return false;
//	if (v_delta >= fov_in_degree_v / 2)
//		return false;
//	if(h_delta >= fov_in_degree_h / 2)
//		return false;
//
//	// Intersection Test
//	Ray ray_query(view_pos, view_to_point_norm);
//	auto intersection = v_tree.first_intersection(ray_query);
//	double intersection_point_squared_distance;
//	double squared_ray_distance;;
//	if (intersection && boost::get<Point_3>(&(intersection->first))) {
//		const Point_3* p = boost::get<Point_3>(&(intersection->first));
//		squared_ray_distance = CGAL::squared_distance(view_pos,*p);
//		intersection_point_squared_distance =
// CGAL::squared_distance(point_pos,*p);
//	}
//	else
//		return false;
//
//	return squared_ray_distance <= squared_distance ||
// intersection_point_squared_distance<=1e-8;
//	//return true;
//}

bool
is_visible(
  const Eigen::Matrix3f&   v_intrinsic_matrix,
  const Eigen::Isometry3f& v_camera_matrix,
  const Eigen::Vector3f&   v_view_pos,
  const Eigen::Vector3f&   v_point_pos,
  const Tree&              v_tree,
  float                    max_distance
)
{

    const Point_3 point_pos(v_point_pos.x(), v_point_pos.y(), v_point_pos.z());
    const Point_3 view_pos(v_view_pos.x(), v_view_pos.y(), v_view_pos.z());

    Vector_3 view_to_point = point_pos - view_pos;
    Vector_3 view_to_point_norm
      = view_to_point / std::sqrt(view_to_point.squared_length());
    double view_to_origin_point = view_to_point.squared_length();
    if (max_distance != -1.f)
        if (max_distance * max_distance < view_to_origin_point) return false;

    // Horizontal
    Eigen::Vector3f pixel_pos
      = v_intrinsic_matrix * v_camera_matrix * v_point_pos;
    pixel_pos /= pixel_pos.z();
    if(pixel_pos.x()<0||pixel_pos.x()>v_intrinsic_matrix(0,2)*2||pixel_pos.y()<0||pixel_pos.y()>v_intrinsic_matrix(1,2)*2)
        return false;

    // Intersection Test
    Ray    ray_query(view_pos, view_to_point_norm);
    auto   intersection = v_tree.first_intersection(ray_query);
    double intersect_point_to_origin_point;
    double view_to_intersect_point;
    ;
    if (intersection && boost::get<Point_3>(&(intersection->first))) {
        const Point_3* p        = boost::get<Point_3>(&(intersection->first));
        view_to_intersect_point = CGAL::squared_distance(view_pos, *p);
        intersect_point_to_origin_point = CGAL::squared_distance(point_pos, *p);
    } else
        return false;
    return view_to_intersect_point >= view_to_origin_point
        || intersect_point_to_origin_point <= 1e-8;
}

bool
is_visible(
  const Eigen::Matrix3f&   v_intrinsic_matrix,
  const Eigen::Isometry3f& v_camera_matrix,
  const Eigen::Vector3f&   v_view_pos,
  const Eigen::Vector3f&   v_point_pos,
  const RTCScene&          v_scene,
  double                   max_distance
)
{
    Eigen::Vector3f view_to_point        = v_point_pos - v_view_pos;
    double          view_to_origin_point = view_to_point.norm();
    view_to_point.normalize();
    // std::cout<<view_to_origin_point;
    if (max_distance != -1.)
        if (max_distance < view_to_origin_point) return false;

    // Horizontal
    Eigen::Vector3f pixel_pos
      = v_intrinsic_matrix * v_camera_matrix * v_point_pos;
    if (pixel_pos.z() < 0) return false;
    pixel_pos /= pixel_pos.z();
    if (pixel_pos.x() < 0 || pixel_pos.x() > v_intrinsic_matrix(0, 2) * 2 ||
    pixel_pos.y() < 0 || pixel_pos.y() > v_intrinsic_matrix(1, 2) * 2)
        return false;

    // Intersection Test
    RTCRayHit rayhits;
    rayhits.ray.org_x  = static_cast<float>(v_view_pos.x());
    rayhits.ray.org_y  = static_cast<float>(v_view_pos.y());
    rayhits.ray.org_z  = static_cast<float>(v_view_pos.z());
    rayhits.ray.dir_x  = static_cast<float>(view_to_point.x());
    rayhits.ray.dir_y  = static_cast<float>(view_to_point.y());
    rayhits.ray.dir_z  = static_cast<float>(view_to_point.z());
    rayhits.ray.tnear  = 0;
    rayhits.ray.tfar   = std::numeric_limits<float>::infinity();
    rayhits.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rayhits.hit.primID = RTC_INVALID_GEOMETRY_ID;

    RTCIntersectContext context;
    rtcInitIntersectContext(&context);

    rtcIntersect1(v_scene, &context, &rayhits);
    if (rayhits.ray.tfar < 0) return false;
    if (rayhits.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
        // std::cout<<view_to_origin_point<< std::endl;
        // std::cout<<rayhits.ray.tfar << std::endl;
        // std::cout<<rayhits.hit.primID << std::endl;
        // std::cout<<Eigen::Vector3f(rayhits.ray.tfar * view_to_point +
        // v_view_pos - v_point_pos).norm()<< std::endl;

        auto hit_position = v_view_pos + (view_to_point * rayhits.ray.tfar);
        // std::cout << (cgal_point_2_eigen(hit_position) - v_point_pos).norm()
        // << std::endl;
        return rayhits.ray.tfar >= view_to_origin_point
            || (hit_position - v_point_pos).norm() <= 1e-3;
    } else {
        return false;
    }
}

bool
is_visible(
  const Eigen::Matrix3f&   v_intrinsic_matrix,
  const Eigen::Isometry3f& v_camera_matrix,
  const Eigen::Vector3f&   v_view_pos,
  const Eigen::Vector3f&   v_point_pos,
  const RTCScene&          v_scene,
  double                   max_distance,
  const Surface_mesh&      v_mesh
)
{

    Eigen::Vector3f view_to_point        = v_point_pos - v_view_pos;
    double          view_to_origin_point = view_to_point.norm();
    view_to_point.normalize();
    // std::cout<<view_to_origin_point;
    if (max_distance != -1.)
        if (max_distance < view_to_origin_point) return false;

    // Horizontal
    Eigen::Vector3f pixel_pos
      = v_intrinsic_matrix * v_camera_matrix * v_point_pos;
    if (pixel_pos.z() < 0) return false;
    pixel_pos /= pixel_pos.z();
    if (pixel_pos.x() < 0 || pixel_pos.x() > v_intrinsic_matrix(0, 2) * 2 || pixel_pos.y() < 0 || pixel_pos.y() > v_intrinsic_matrix(1, 2) * 2)
        return false;

    // Intersection Test
    RTCRayHit rayhits;
    rayhits.ray.org_x  = static_cast<float>(v_view_pos.x());
    rayhits.ray.org_y  = static_cast<float>(v_view_pos.y());
    rayhits.ray.org_z  = static_cast<float>(v_view_pos.z());
    rayhits.ray.dir_x  = static_cast<float>(view_to_point.x());
    rayhits.ray.dir_y  = static_cast<float>(view_to_point.y());
    rayhits.ray.dir_z  = static_cast<float>(view_to_point.z());
    rayhits.ray.tnear  = 0;
    rayhits.ray.tfar   = std::numeric_limits<float>::infinity();
    rayhits.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rayhits.hit.primID = RTC_INVALID_GEOMETRY_ID;

    RTCIntersectContext context;
    rtcInitIntersectContext(&context);

    rtcIntersect1(v_scene, &context, &rayhits);
    if (rayhits.ray.tfar < 0) return false;
    if (rayhits.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
        // std::cout<<view_to_origin_point<< std::endl;
        // std::cout<<rayhits.ray.tfar << std::endl;
        // std::cout<<rayhits.hit.primID << std::endl;
        // std::cout<<Eigen::Vector3f(rayhits.ray.tfar * view_to_point +
        // v_view_pos - v_point_pos).norm()<< std::endl;

        auto iter
          = v_mesh
              .vertices_around_face(
                v_mesh.halfedge(*(v_mesh.faces_begin() + rayhits.hit.primID))
              )
              .begin();
        Point_3 p1 = v_mesh.point(*iter);
        iter++;
        Point_3 p2 = v_mesh.point(*iter);
        iter++;
        Point_3  p3           = v_mesh.point(*iter);
        Vector_3 v1           = p2 - p1;
        Vector_3 v2           = p3 - p1;
        Point_3  hit_position = p1 + (v1 * rayhits.hit.u + v2 * rayhits.hit.v);
        // std::cout << (cgal_point_2_eigen(hit_position) -
        // v_point_pos).norm()
        // << std::endl;
        return rayhits.ray.tfar >= view_to_origin_point
            || (cgal_point_2_eigen(hit_position) - v_point_pos).norm() <= 1e-3;
    } else {
        return false;
    }
}

Eigen::Vector3f
get_hit_position(
  const Eigen::Matrix3f&   v_intrinsic_matrix,
  const Eigen::Isometry3f& v_camera_matrix,
  const Eigen::Vector3f&   v_view_pos,
  const Eigen::Vector3f&   v_point_pos,
  const RTCScene&          v_scene,
  double                   max_distance
)
{
    Eigen::Vector3f view_to_point        = v_point_pos - v_view_pos;
    double          view_to_origin_point = view_to_point.norm();
    view_to_point.normalize();

    Eigen::Vector3f hit_position(
      std::numeric_limits<float>::max(),
      std::numeric_limits<float>::max(),
      std::numeric_limits<float>::max()
    );

    // Intersection Test
    RTCRayHit rayhits;
    rayhits.ray.org_x  = static_cast<float>(v_view_pos.x());
    rayhits.ray.org_y  = static_cast<float>(v_view_pos.y());
    rayhits.ray.org_z  = static_cast<float>(v_view_pos.z());
    rayhits.ray.dir_x  = static_cast<float>(view_to_point.x());
    rayhits.ray.dir_y  = static_cast<float>(view_to_point.y());
    rayhits.ray.dir_z  = static_cast<float>(view_to_point.z());
    rayhits.ray.tnear  = 0;
    rayhits.ray.tfar   = std::numeric_limits<float>::infinity();
    rayhits.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rayhits.hit.primID = RTC_INVALID_GEOMETRY_ID;

    RTCIntersectContext context;
    rtcInitIntersectContext(&context);

    rtcIntersect1(v_scene, &context, &rayhits);
    if (rayhits.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
        hit_position = v_view_pos + (view_to_point * rayhits.ray.tfar);
    }
    return hit_position;
}

bool
fall_in_camera_region(
  const Eigen::Matrix3f&   v_intrinsic_matrix,
  const Eigen::Isometry3f& v_camera_matrix,
  const Eigen::Vector3f&   v_view_pos,
  const Eigen::Vector3f&   v_point_pos,
  const RTCScene&          v_scene,
  double                   max_distance
)
{
    Eigen::Vector3f view_to_point        = v_point_pos - v_view_pos;
    double          view_to_origin_point = view_to_point.norm();
    view_to_point.normalize();
    // std::cout<<view_to_origin_point;
    if (max_distance != -1.)
        if (max_distance < view_to_origin_point) return false;

    // Horizontal
    Eigen::Vector3f pixel_pos
      = v_intrinsic_matrix * v_camera_matrix * v_point_pos;
    if (pixel_pos.z() < 0) return false;
    pixel_pos /= pixel_pos.z();
    if (pixel_pos.x() < 0 || pixel_pos.x() > v_intrinsic_matrix(0, 2) * 2 ||
    pixel_pos.y() < 0 || pixel_pos.y() > v_intrinsic_matrix(1, 2) * 2)
        return false;

    return true;
}