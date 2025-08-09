#include <Eigen/Dense>
#include <open3d/Open3D.h>
#include <iostream>
#include "dataloader.hpp"
// #include "transformer.hpp"
#include "visualizer.hpp"
#include "bresenham.hpp"
#include <chrono>
#include <algorithm>

using namespace std;
using clock_ = std::chrono::steady_clock;
using Vector3dVector = vector<Eigen::Vector3d>;
using PoseAndCloud = pair<Eigen::Matrix4d,Vector3dVector>;


int main() {

    cout << " LOADING THE DATA ..." << endl;
    const string dataset_dir = "/Users/amir/Desktop/class/4th_Semester/ModernC++/project/data/";
    dataloader::Dataset dataset(dataset_dir); 
    cout << "Dataset size: " << dataset.size() << endl;
    cout << "Finished loading the data." << endl;
    cout << " -------------------------------- " << endl;
    cout << "TRANSFORMING THE SCAN POINTS AND CREATING 3D OCCUPANCY GRID MAP USING BRESENHAM ALGORITHM." << endl;
    cout << "Please BE PATIENT! THIS WILL TAKE SOME TIME .... !" << endl;
    // auto [pose, cloud_sensor_frame] = dataset[6000];
    // cout << "Pose: " << endl;
    // cout << pose << endl;
    // cout << "Cloud size: " << cloud_sensor_frame.size() << endl; 
    // cout << " -------------------------------- " << endl;

    const auto process_start = clock_::now();

    size_t scans_processed = 0;  // std::size_t an alias for unsigned long or unsigned long long
    double accumulated_scan_ms = 0.0;

    OccupancyGrid3D grid(0.1);

    // for (size_t i = 0; i < dataset.size(); ++i){
    for (size_t i = 0; i < 300; ++i){
        const auto& [pose, cloud_sensor] = dataset[i];

        auto now = clock_::now();
        auto elapsed = chrono::duration_cast<chrono::minutes>(now - process_start).count();
        cout << "TIME SPENT : " << elapsed << " minutes" << endl;

        const auto scan_start = clock_::now();

        Eigen::Vector3d origin = pose.block<3,1>(0,3); // capturing the translation vector as the origin point

        // Vector3dVector cloud_world;
        // cloud_world.reserve(cloud_sensor.size());

        auto transform_and_cast = [&](const Eigen::Vector3d& point_s) {
            Eigen::Vector4d hom(point_s.x(), point_s.y(), point_s.z(), 1.0);
            Eigen::Vector3d point_w = (pose * hom).head<3>();
            grid.insertRay(origin, point_w);
        };

        std::for_each(cloud_sensor.begin(), cloud_sensor.end(), transform_and_cast);

        const auto scan_end = clock_::now();

        const double scan_ms = chrono::duration<double, milli>(scan_end - scan_start).count();
        accumulated_scan_ms += scan_ms;
        ++scans_processed;
        cout << "Scan " << i << " process time: " << scan_ms << " ms" << endl;

    }

    const auto process_end = clock_::now();
    const double total_process_minutes = chrono::duration_cast<chrono::duration<double>>(process_end - process_start).count() / 60.0;

    cout << "--------------------------------" << endl;
    cout << "Scans processed: " << scans_processed << endl;
    cout << "Average per-scan process time: " << (accumulated_scan_ms / scans_processed) << " ms" << endl;
    cout << "Total process time : " << total_process_minutes << " minutes!" << endl;
    cout << "--------------------------------" << endl;

    // Get the final occupied points after processing all rays
    auto occupied_points = grid.getOccupiedPoints();
    cout << "Final Occupied Voxels: " << occupied_points.size() << endl;
    
    cout << "Visualizing final Occupied Voxels..." << endl;
    visualize(occupied_points, string("Occupied Voxels"));

    return 0;
}

