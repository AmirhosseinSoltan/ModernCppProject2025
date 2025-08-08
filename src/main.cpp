#include <Eigen/Dense>
#include <open3d/Open3D.h>
#include <iostream>
// #include <graphics.h>
#include "dataloader.hpp"
#include "transformer.hpp"
#include "visualizer.hpp"

using namespace std;
using Vector3dVector = vector<Eigen::Vector3d>;
using PoseAndCloud = pair<Eigen::Matrix4d,Vector3dVector>;


int main() {

    const string dataset_dir = "/Users/amir/Desktop/class/4th_Semester/ModernC++/project/data/";
    dataloader::Dataset dataset(dataset_dir);
    cout << "Dataset size: " << dataset.size() << endl;
    cout << " -------------------------------- " << endl;

    // auto [pose, cloud_sensor_frame] = dataset[6000];
    // cout << "Pose: " << endl;
    // cout << pose << endl;
    // cout << "Cloud size: " << cloud_sensor_frame.size() << endl; 
    // cout << " -------------------------------- " << endl;

    // // Debug: Print first few original points
    // cout << "First 5 original points:" << endl;
    // for (int i = 0; i < 5; i++) {
    //     cout << "Point " << i << ": " << cloud_sensor_frame[i].transpose() << endl;
    // }
    // cout << " -------------------------------- " << endl;

    // using DatasetType = std::pair<std::vector<Eigen::Matrix4d>, Vector3dVector>;


    Vector3dVector transformed_cloud = transformer::TransformDataset (dataset);

    
    // Printing first few transformed points
    cout << "First 5 transformed points:" << endl;
    for (int i = 0; i < 5; i++) {
        cout << "Transformed Point " << i << ": " << transformed_cloud[i].transpose() << endl;
    }
    cout << " -------------------------------- " << endl;

    // // visualizing the original cloud 
    // cout << "Visualizing original cloud (without transformation)..." << endl;
    // visualize(cloud_sensor_frame, string("Original points"));
    

    cout << "Visualizing transformed cloud..." << endl;
    visualize(transformed_cloud, string("Transformed Point Cloud"));

    return 0;
}

