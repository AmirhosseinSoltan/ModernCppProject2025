#include <Eigen/Dense>
#include <open3d/Open3D.h>
#include <iostream>
#include "dataloader.hpp"
#include "transformer.hpp"
#include <algorithm>
using namespace std;

namespace transformer {
    Vector3dVector TransformDataset(const dataloader::Dataset& dataset) {
        
        cout << "TRANSFORMING POINTS TO REFRENCE FRAME ... " << endl;
        //output cloud
        Vector3dVector transformed_cloud;
        transformed_cloud.reserve(dataset.size() * 64000); // rough upper bound

        // for (int i = 0; i < dataset.size(); ++i) {
        for (int i = 0; i < 500; ++i) {
            const auto& [pose, cloud] = dataset[i];
            
            //lambda
            auto homogenous_transform = [&](const Eigen::Vector3d& point) {
                Eigen::Vector4d hom(point.x(), point.y(), point.z(), 1.0);
                transformed_cloud.emplace_back((pose * hom).head<3>());
            };

            // Useing std::for_each with lambda to satisfy STL algorithm requirement
            std::for_each(cloud.begin(), cloud.end(), homogenous_transform);
        }

        transformed_cloud.shrink_to_fit();

        cout << " FINISHED TRANSFORMATION." << endl;
        return transformed_cloud;
    }
}