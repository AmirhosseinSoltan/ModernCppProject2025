#pragma once
#include <vector>
#include <Eigen/Dense>
#include "dataloader.hpp"

using Vector3dVector = std::vector<Eigen::Vector3d>;

namespace transformer {
    Vector3dVector TransformDataset(const dataloader::Dataset& dataset);
}