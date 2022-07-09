#include "evaluate_model.h"

#define TINYOBJLOADER_IMPLEMENTATION

int
EvaluateModel::read_gt_model(std::string gt_model_path)
{
    std::vector<Point_3>             gt_vertices;
    std::vector<Vector_3>            gt_normals;
    std::vector<CGAL::Triangle_3<K>> gt_faces;
    std::vector<std::array<int, 3>>  gt_face_indexes;
    read_model(
      gt_model_path, gt_vertices, gt_normals, gt_face_indexes, gt_faces
    );
    bvhtree_gt_mesh = new Triangle_Tree(gt_faces.begin(), gt_faces.end());
    bvhtree_gt_mesh->build();
    bvhtree_gt_mesh->accelerate_distance_queries(
      gt_vertices.begin(), gt_vertices.end()
    );

    gt_point_set
      = sample_points_according_density(gt_faces, mesh_sampling_density);
    LOG(INFO) << "Read GT model done. It has " << gt_vertices.size()
              << " points and " << gt_faces.size() << " faces";
    LOG(INFO) << "Sampling points on the mesh according to density "
                 "1000/m2, result in "
              << gt_point_set.size() << " points";
    kdtree_gt_points.insert(
      gt_point_set.points().begin(), gt_point_set.points().end()
    );
    kdtree_gt_points.build();
    return 0;
}

int
EvaluateModel::read_recon_model(std::string recon_model_path)
{
    std::vector<Point_3>             recon_vertices;
    std::vector<Vector_3>            recon_normals;
    std::vector<CGAL::Triangle_3<K>> recon_faces;
    std::vector<std::array<int, 3>>  recon_face_indexes;
    read_model(
      recon_model_path,
      recon_vertices,
      recon_normals,
      recon_face_indexes,
      recon_faces
    );
    recon_point_set = sample_points_according_density(recon_faces, 1);
    LOG(INFO) << "Read Recon model done. It has " << recon_vertices.size()
              << " points and " << recon_faces.size() << " faces";
    LOG(INFO) << "Sampling points on the mesh according to density "
              << mesh_sampling_density << "/m2, result in "
              << recon_point_set.size() << " points";
    LOG(INFO) << "Start to build tree";
    bvhtree_recon_mesh
      = new Triangle_Tree(recon_faces.begin(), recon_faces.end());
    bvhtree_recon_mesh->build();
    bvhtree_recon_mesh->accelerate_distance_queries(
      recon_vertices.begin(), recon_vertices.end()
    );
    kdtree_recon_points.insert(
      recon_point_set.points().begin(), recon_point_set.points().end()
    );
    kdtree_recon_points.build();
    return 0;
}

int
EvaluateModel::read_gt_points(std::string gt_points_path)
{
    gt_point_set.clear();
    CGAL::IO::read_point_set(gt_points_path, gt_point_set);
    LOG(INFO) << "Read gt " << gt_point_set.size() << " points.";
    kdtree_gt_points.clear();
    kdtree_gt_points.insert(
      gt_point_set.points().begin(), gt_point_set.points().end()
    );
    kdtree_gt_points.build();
    return gt_point_set.size();
}

int
EvaluateModel::read_recon_points(std::string recon_points_path)
{
    recon_point_set.clear();
    CGAL::IO::read_point_set(recon_points_path, recon_point_set);
    LOG(INFO) << "Read recon " << recon_point_set.size() << " points.";
    kdtree_recon_points.clear();
    kdtree_recon_points.insert(
      recon_point_set.points().begin(), recon_point_set.points().end()
    );
    kdtree_recon_points.build();
    return recon_point_set.size();
}

int
EvaluateModel::set_gt_points(std::vector<Eigen::Vector3f> points)
{
    gt_point_set.clear();
    for (auto point : points) {
        gt_point_set.insert(eigen_2_cgal_point(point));
    }
    kdtree_gt_points.clear();
    kdtree_gt_points.insert(
      gt_point_set.points().begin(), gt_point_set.points().end()
    );
    kdtree_gt_points.build();
    return gt_point_set.size();
}

int
EvaluateModel::set_recon_points(std::vector<Eigen::Vector3f> points)
{
    recon_point_set.clear();
    for (auto point : points) {
        recon_point_set.insert(eigen_2_cgal_point(point));
    }
    kdtree_recon_points.clear();
    kdtree_recon_points.insert(
      recon_point_set.points().begin(), recon_point_set.points().end()
    );
    kdtree_recon_points.build();
    return recon_point_set.size();
}

int
EvaluateModel::filter_points_by_z(Point_set point_set)
{
    int original_size = point_set.size();
    for (int i = point_set.size() - 1; i >= 0; --i)
        if (point_set.point(i).z() < filter_z) point_set.remove(i);
    point_set.collect_garbage();
    LOG(INFO) << boost::format(
                   "Filter out %d points whose z value is less than %f; Remain "
                   "%d points"
                 ) % (original_size - point_set.size())
                   % filter_z % point_set.size();
    return original_size - point_set.size();
}

int
EvaluateModel::evaluate()
{
    LOG(INFO) << "Start to calculate accuracy";
    std::vector<double> accuracy(recon_point_set.size());
#pragma omp parallel for
    for (int i_point = 0; i_point < recon_point_set.size(); ++i_point) {
        const auto&     point2 = recon_point_set.point(i_point);
        Neighbor_search search(kdtree_gt_points, point2, 1);
        for (Neighbor_search::iterator it = search.begin(); it != search.end();
             ++it)
            accuracy[i_point] = std::sqrt(it->second);
    }

    LOG(INFO) << "Start to calculate completeness";

    std::vector<double> completeness(gt_point_set.size());
#pragma omp parallel for
    for (int i_point = 0; i_point < gt_point_set.size(); ++i_point) {
        const auto&     point1 = gt_point_set.point(i_point);
        Neighbor_search search(kdtree_recon_points, point1, 1);
        for (Neighbor_search::iterator it = search.begin(); it != search.end();
             ++it)
            completeness[i_point] = std::sqrt(it->second);
    }

    auto gt_error_map = gt_point_set.add_property_map<double>("error").first;
    auto recon_error_map
      = recon_point_set.add_property_map<double>("error").first;

    for (const auto& i_point : gt_point_set)
        gt_error_map[i_point] = completeness[i_point];
    for (const auto& i_point : recon_point_set)
        recon_error_map[i_point] = accuracy[i_point];

    LOG(INFO
    ) << "Chamfer distance "
      << std::accumulate(accuracy.begin(), accuracy.end(), 0.f)
             / accuracy.size()
           + std::accumulate(completeness.begin(), completeness.end(), 0.f)
               / completeness.size();

    LOG(INFO) << "Husdoff distance "
              << std::max(
                   *std::max_element(accuracy.begin(), accuracy.end()),
                   *std::max_element(completeness.begin(), completeness.end())
                 );

    if (if_write_error_file) {
        std::ofstream a("recon_sample_points_with_error.ply", std::ios::binary);
        a.precision(10);
        CGAL::set_binary_mode(a);
        CGAL::IO::write_PLY_with_properties(
          a,
          recon_point_set,
          CGAL::IO::make_ply_point_writer(recon_point_set.point_map()),
          std::make_pair(recon_error_map, CGAL::PLY_property<double>("error"))
        );
        a.close();

        std::ofstream b("gt_sample_points_with_error.ply", std::ios::binary);
        b.precision(10);
        CGAL::set_binary_mode(b);
        CGAL::IO::write_PLY_with_properties(
          b,
          gt_point_set,
          CGAL::IO::make_ply_point_writer(gt_point_set.point_map()),
          std::make_pair(gt_error_map, CGAL::PLY_property<double>("error"))
        );
        b.close();
    }

    // =================================================== Accuracy
    // ===================================================
    {
        LOG(INFO) << "Start to sort";
        std::vector<double> sorted_accuracy(accuracy);
        std::sort(sorted_accuracy.begin(), sorted_accuracy.end());

        std::vector<double> accuracy_threshold{
            sorted_accuracy[(int)(sorted_accuracy.size() / 100 * 50)],
            sorted_accuracy[(int)(sorted_accuracy.size() / 100 * 60)],
            sorted_accuracy[(int)(sorted_accuracy.size() / 100 * 70)],
            sorted_accuracy[(int)(sorted_accuracy.size() / 100 * 80)],
            sorted_accuracy[(int)(sorted_accuracy.size() / 100 * 90)],
            sorted_accuracy[(int)(sorted_accuracy.size() / 100 * 95)]
        };

        std::vector<Point_set> point_set_accuracy_outlier(
          accuracy_threshold.size()
        );

        LOG(INFO) << "Start to extract";
        for (int i_point = 0; i_point < recon_point_set.size(); ++i_point) {
            for (int i_threshold = 0; i_threshold < accuracy_threshold.size();
                 ++i_threshold) {
                if (accuracy[i_point] > accuracy_threshold[i_threshold])
                    point_set_accuracy_outlier[i_threshold].insert(
                      recon_point_set.point(i_point)
                    );
            }
        }

        if (if_write_outlier_file)
            for (int i_threshold = 0; i_threshold < accuracy_threshold.size();
                 ++i_threshold) {
                std::ofstream f(
                  "accuracy_outlier_"
                    + std::to_string(accuracy_threshold[i_threshold]) + ".ply",
                  std::ios::out | std::ios::binary
                );
                CGAL::set_binary_mode(f);
                CGAL::write_ply_point_set(
                  f, point_set_accuracy_outlier[i_threshold]
                );
            }

        LOG(INFO) << "50% Accuracy: " << accuracy_threshold[0];
        LOG(INFO) << "60% Accuracy: " << accuracy_threshold[1];
        LOG(INFO) << "70% Accuracy: " << accuracy_threshold[2];
        LOG(INFO) << "80% Accuracy: " << accuracy_threshold[3];
        LOG(INFO) << "90% Accuracy: " << accuracy_threshold[4];
        LOG(INFO) << "95% Accuracy: " << accuracy_threshold[5];

        LOG(INFO
        ) << "50% Average: "
          << std::accumulate(
               sorted_accuracy.begin(),
               sorted_accuracy.begin() + (int)(sorted_accuracy.size() * 0.5),
               0.f
             ) / (int)(sorted_accuracy.size() * 0.5);
        LOG(INFO
        ) << "60% Average: "
          << std::accumulate(
               sorted_accuracy.begin(),
               sorted_accuracy.begin() + (int)(sorted_accuracy.size() * 0.6),
               0.f
             ) / (int)(sorted_accuracy.size() * 0.6);
        LOG(INFO
        ) << "70% Average: "
          << std::accumulate(
               sorted_accuracy.begin(),
               sorted_accuracy.begin() + (int)(sorted_accuracy.size() * 0.7),
               0.f
             ) / (int)(sorted_accuracy.size() * 0.7);
        LOG(INFO
        ) << "80% Average: "
          << std::accumulate(
               sorted_accuracy.begin(),
               sorted_accuracy.begin() + (int)(sorted_accuracy.size() * 0.8),
               0.f
             ) / (int)(sorted_accuracy.size() * 0.8);
        LOG(INFO
        ) << "90% Average: "
          << std::accumulate(
               sorted_accuracy.begin(),
               sorted_accuracy.begin() + (int)(sorted_accuracy.size() * 0.9),
               0.f
             ) / (int)(sorted_accuracy.size() * 0.9);
        LOG(INFO
        ) << "95% Average: "
          << std::accumulate(
               sorted_accuracy.begin(),
               sorted_accuracy.begin() + (int)(sorted_accuracy.size() * 0.95),
               0.f
             ) / (int)(sorted_accuracy.size() * 0.95);

        std::vector<double> accuracy_range_threshold{ 0.005, 0.01, 0.02,
                                                      0.03,  0.05, 0.1,
                                                      0.2,   0.5,  1 };
        std::vector<double> accuracy_range(accuracy_range_threshold.size(), 0);
        for (int i_point = 0; i_point < accuracy.size(); i_point++) {
            for (int i_threshold = 0;
                 i_threshold < accuracy_range_threshold.size();
                 ++i_threshold) {
                if (accuracy_range[i_threshold] == 0 && sorted_accuracy[i_point] > accuracy_range_threshold[i_threshold])
                    accuracy_range[i_threshold]
                      = (double)i_point / accuracy.size() * 100;
            }
        }

        for (int i_threshold = 0; i_threshold < accuracy_range_threshold.size();
             ++i_threshold)
            LOG(INFO) << accuracy_range[i_threshold]
                      << "% of points has error lower than "
                      << accuracy_range_threshold[i_threshold];
    }

    // =================================================== Completeness
    // ===================================================
    {
        LOG(INFO) << "Start to compute completeness";
        std::vector<double> sorted_completeness(completeness);
        std::sort(sorted_completeness.begin(), sorted_completeness.end());

        std::vector<double> completeness_threshold{
            sorted_completeness[(int)(sorted_completeness.size() / 100 * 50)],
            sorted_completeness[(int)(sorted_completeness.size() / 100 * 60)],
            sorted_completeness[(int)(sorted_completeness.size() / 100 * 70)],
            sorted_completeness[(int)(sorted_completeness.size() / 100 * 80)],
            sorted_completeness[(int)(sorted_completeness.size() / 100 * 90)],
            sorted_completeness[(int)(sorted_completeness.size() / 100 * 95)]
        };

        std::vector<Point_set> point_set_completeness_outlier(
          completeness_threshold.size()
        );

        LOG(INFO) << "Start to extract";
        for (int i_point = 0; i_point < completeness.size(); ++i_point) {
            for (int i_threshold = 0;
                 i_threshold < completeness_threshold.size();
                 ++i_threshold) {
                if (completeness[i_point] > completeness_threshold[i_threshold])
                    point_set_completeness_outlier[i_threshold].insert(
                      gt_point_set.point(i_point)
                    );
            }
        }

        if (if_write_outlier_file)
            for (int i_threshold = 0;
                 i_threshold < completeness_threshold.size();
                 ++i_threshold) {
                std::ofstream f(
                  "completeness_outlier_"
                    + std::to_string(completeness_threshold[i_threshold])
                    + ".ply",
                  std::ios::out | std::ios::binary
                );
                CGAL::set_binary_mode(f);
                CGAL::write_ply_point_set(
                  f, point_set_completeness_outlier[i_threshold]
                );
            }

        LOG(INFO) << "50% Completness: " << completeness_threshold[0];
        LOG(INFO) << "60% Completness: " << completeness_threshold[1];
        LOG(INFO) << "70% Completness: " << completeness_threshold[2];
        LOG(INFO) << "80% Completness: " << completeness_threshold[3];
        LOG(INFO) << "90% Completness: " << completeness_threshold[4];
        LOG(INFO) << "95% Completness: " << completeness_threshold[5];

        LOG(INFO) << "50% Average: "
                  << std::accumulate(
                       sorted_completeness.begin(),
                       sorted_completeness.begin()
                         + (int)(sorted_completeness.size() * 0.5),
                       0.f
                     ) / (int)(sorted_completeness.size() * 0.5);
        LOG(INFO) << "60% Average: "
                  << std::accumulate(
                       sorted_completeness.begin(),
                       sorted_completeness.begin()
                         + (int)(sorted_completeness.size() * 0.6),
                       0.f
                     ) / (int)(sorted_completeness.size() * 0.6);
        LOG(INFO) << "70% Average: "
                  << std::accumulate(
                       sorted_completeness.begin(),
                       sorted_completeness.begin()
                         + (int)(sorted_completeness.size() * 0.7),
                       0.f
                     ) / (int)(sorted_completeness.size() * 0.7);
        LOG(INFO) << "80% Average: "
                  << std::accumulate(
                       sorted_completeness.begin(),
                       sorted_completeness.begin()
                         + (int)(sorted_completeness.size() * 0.8),
                       0.f
                     ) / (int)(sorted_completeness.size() * 0.8);
        LOG(INFO) << "90% Average: "
                  << std::accumulate(
                       sorted_completeness.begin(),
                       sorted_completeness.begin()
                         + (int)(sorted_completeness.size() * 0.9),
                       0.f
                     ) / (int)(sorted_completeness.size() * 0.9);
        LOG(INFO) << "95% Average: "
                  << std::accumulate(
                       sorted_completeness.begin(),
                       sorted_completeness.begin()
                         + (int)(sorted_completeness.size() * 0.95),
                       0.f
                     ) / (int)(sorted_completeness.size() * 0.95);

        std::vector<double> completeness_range_threshold{ 0.005, 0.01, 0.02,
                                                          0.03,  0.05, 0.1,
                                                          0.2,   0.5,  1 };
        std::vector<double> completeness_range(
          completeness_range_threshold.size(), 0
        );
        for (int i_point = 0; i_point < completeness.size(); i_point++) {
            for (int i_threshold = 0;
                 i_threshold < completeness_range_threshold.size();
                 ++i_threshold) {
                if (completeness_range[i_threshold] == 0 && sorted_completeness[i_point] > completeness_range_threshold[i_threshold])
                    completeness_range[i_threshold]
                      = (double)i_point / completeness.size() * 100;
            }
        }

        for (int i_threshold = 0;
             i_threshold < completeness_range_threshold.size();
             ++i_threshold)
            LOG(INFO) << completeness_range[i_threshold]
                      << "% of points has error lower than "
                      << completeness_range_threshold[i_threshold];
    }
    return 0;
}