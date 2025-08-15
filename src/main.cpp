#include <Eigen/Dense>
#include <iostream>
#include "dataloader.hpp"
#include "visualizer.hpp"
#include "bresenham.hpp"
#include <chrono>
#include <algorithm>
#include <cxxopts.hpp>

using namespace std;
using clock_ = std::chrono::steady_clock;
using Vector3dVector = vector<Eigen::Vector3d>;
using Vector3d = Eigen::Vector3d;
using PoseAndCloud = pair<Eigen::Matrix4d,Vector3dVector>;


int main(int argc, char **argv) {
    
    cxxopts::Options options("OccupancyMapper", "3D occupancy grid mapper");
    options.add_options()
    ("d,data-path", "Dataset directory", cxxopts::value<string>()->default_value("/Users/amir/Desktop/class/4th_Semester/ModernC++/project/data/"))
    ("s,voxel-size", "Voxel Size", cxxopts::value<double>()->default_value("0.3"))
    ("h,help", "Show help");

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }

    const string dataset_dir = result["data-path"].as<string>();
    double VoxelSize = result["voxel-size"].as<double>();

    cout << "LOADING THE DATA ..." << endl;
    dataloader::Dataset dataset(dataset_dir); 
    cout << "Dataset size: " << dataset.size() << endl;
    cout << "Finished loading the data." << endl;
    cout << " -------------------------------- " << endl;
    cout << "TRANSFORMING THE SCAN POINTS AND CREATING 3D OCCUPANCY GRID MAP USING BRESENHAM ALGORITHM." << endl;
    cout << "Please BE PATIENT! THIS WILL TAKE SOME TIME .... !" << endl;

    const auto process_start = clock_::now();

    size_t scans_processed = 0;  // std::size_t an alias for unsigned long or unsigned long long
    double accumulated_scan_s = 0.0;

    OccupancyGrid3D grid(VoxelSize);

    int num_scans = dataset.size();
    // int num_scans = 300;
    for (size_t i = 0; i < num_scans; ++i){
        const auto& [pose, cloud_sensor] = dataset[i];

        const auto scan_start = clock_::now();

        Vector3d origin = pose.block<3,1>(0,3); // capturing the translation vector as the origin point

        auto transform_and_cast = [&](const Vector3d& point_s) {
            Eigen::Vector4d hom(point_s.x(), point_s.y(), point_s.z(), 1.0);
            Vector3d point_w = (pose * hom).head<3>();
            grid.insertRay(origin, point_w);
        };

        // Note: we could use std::execution::par here to run in parallel. But macOS libc++ doesn't support 
        std::for_each(cloud_sensor.begin(), cloud_sensor.end(), transform_and_cast);

        const auto scan_end = clock_::now();

        const double scan_s = chrono::duration<double>(scan_end - scan_start).count();

        std::cout << " Scan " << (i + 1) << "/" << num_scans << "   |  " << scan_s << " s " << std::endl;

        accumulated_scan_s += scan_s;
        ++scans_processed;

    }
    
    const auto process_end = clock_::now();
    const double total_process_minutes = chrono::duration_cast<chrono::duration<double>>(process_end - process_start).count() / 60.0;
    
    cout << "--------------------------------" << endl;
    cout << "Scans processed: " << scans_processed << endl;
    cout << "Average per-scan process time: " << (accumulated_scan_s / scans_processed) << " s" << endl;
    cout << "Total process time : " << total_process_minutes << " minutes!" << endl;
    cout << "--------------------------------" << endl;

    // Get the final occupied points after processing all rays
    float occupancy_probability = 0.9;
    auto occupied_points = grid.getOccupiedPoints(occupancy_probability);
    cout << "Final Occupied Voxels: " << occupied_points.size() << endl;
	cout << "Voxel Size " << VoxelSize << endl;
    cout << "Visualizing final Occupied Voxels..." << endl;
    visualize(occupied_points, string("Occupied Voxels"));

    return 0;
}

