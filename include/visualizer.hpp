#pragma once

#include <Eigen/Core>
#include <memory>
#include <vector>
#include <iostream>

#include "open3d/Open3D.h"

inline void visualize(
        const std::vector<Eigen::Vector3d>& pointcloud, const std::string& name) {
    std::cout << "Creating PointCloud with " << pointcloud.size() << " points" << std::endl;
    
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    
    for (size_t i = 0; i < pointcloud.size(); i++) {
        const auto& pt = pointcloud[i];
        pcd->points_.emplace_back(pt.x(), pt.y(), pt.z());
    } 

    open3d::visualization::DrawGeometries({pcd}, name, 1600, 1024);
}
