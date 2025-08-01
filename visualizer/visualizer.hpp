#pragma once

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "open3d/Open3D.h"

inline void visualize(
        const std::vector<Eigen::Vector3d>& pointcloud) {
    open3d::visualization::DrawGeometries(
            {std::make_shared<open3d::geometry::PointCloud>(pointcloud)});
}