#include <Eigen/Dense>
#include <open3d/Open3D.h>
#include <iostream>

int main() {
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    std::cout << "Test matrix:\n" << m << std::endl;

    open3d::geometry::PointCloud cloud;
    std::cout << "Open3D cloud initialized with " << cloud.points_.size() << " points\n";

    return 0;
}
