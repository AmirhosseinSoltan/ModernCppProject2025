#pragma once

#include <Eigen/Core>
#include <string>
#include <utility>
#include <vector>

using Vector3dVector = std::vector<Eigen::Vector3d>;
using PoseAndCloud = std::pair<Eigen::Matrix4d, Vector3dVector>;

namespace dataloader
{
    class Dataset
    {
    public:
        Dataset(const std::string &data_dir);
        std::size_t size() const { return pointcloud_files_.size(); }
        PoseAndCloud operator[](const int idx) const;

    private:
        std::vector<std::string> pointcloud_files_;
        std::vector<Eigen::Matrix4d> poses_;
    };
} // namespace dataloader
