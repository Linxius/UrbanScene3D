#ifndef __TEST_BIND_ARGS_H__
#define __TEST_BIND_ARGS_H__

#include <CGAL/IO/write_ply_points.h>
#include <CGAL/OpenGR/compute_registration_transformation.h>
#include <CGAL/aff_transformation_tags.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/point_set_processing_assertions.h>
#include <CGAL/polygon_mesh_processing.h>

#include <iostream>
#include <tuple>

#include <boost/format.hpp>
#include <glog/logging.h>

#include "cgal_tools.h"
#include "common_util.h"
#include "intersection_tools.h"
#include "model_tools.h" 

class EvaluateModel
{
  public:
    bool  if_write_error_file   = false;
    bool  if_write_outlier_file = false;
    int   mesh_sampling_density = 1000;
    float filter_z              = -9;

    int read_gt_model(std::string gt_model_path);

    int read_recon_model(std::string recon_model_path);

    int read_gt_points(
      std::string gt_points_path
      = "E:\\reconstructability\\proxy\\xuexiao_inter.ply"
    );

    int read_recon_points(
      std::string recon_points_path
      = "E:\\reconstructability\\proxy\\xuexiao_coarse.ply"
    );

    int set_gt_points(std::vector<Eigen::Vector3f> points);

    int set_recon_points(std::vector<Eigen::Vector3f> points);

    int filter_points_by_z(Point_set point_set);

    int test()
    {
        read_gt_points();
        read_recon_points();
        filter_points_by_z(recon_point_set);
        filter_points_by_z(gt_point_set);
        evaluate();
        return 0;
    }

    int evaluate();

  private:
    Point_set gt_point_set;
    Point_set recon_point_set;

    KDTree kdtree_gt_points;
    KDTree kdtree_recon_points;

    Triangle_Tree* bvhtree_recon_mesh = nullptr;
    Triangle_Tree* bvhtree_gt_mesh    = nullptr;
};

#endif
