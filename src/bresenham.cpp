#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>
#include "bresenham.hpp"

using namespace std;
using Key = tuple<int,int,int>;
using Vector3dVector = vector<Eigen::Vector3d>;
using Vector3d = Eigen::Vector3d;

// Returns occupied voxel centers for visualization
Vector3dVector OccupancyGrid3D::getOccupiedPoints() const 
{
	Vector3dVector pts;
	pts.reserve(occupancy_map_.size());
	for (const auto& [key,status] : occupancy_map_) {
		if (status) {
			pts.emplace_back(
				(key.x + 0.5) * voxel_size_,
				(key.y + 0.5) * voxel_size_,
				(key.z + 0.5) * voxel_size_);
		}
	}
	cout << "Total voxels in map: " << occupancy_map_.size() << endl;
	cout << "Occupied voxels: " << pts.size() << endl;
	return pts;
}


void OccupancyGrid3D::insertRay(const Vector3d& start, const Vector3d& end) {

    auto s = toVoxelKey(start);
    auto e = toVoxelKey(end);

	int x1 = s.x, y1 = s.y, z1 = s.z;
	int x2 = e.x, y2 = e.y, z2 = e.z;
	// Distances along each axis
	int dx = abs(x2 - x1);
	int dy = abs(y2 - y1);
	int dz = abs(z2 - z1);
	// Step variables or direction indicators for each axis
	int xs = (x2 > x1) ? 1 : -1;
	int ys = (y2 > y1) ? 1 : -1;
	int zs = (z2 > z1) ? 1 : -1;

    // Temporary vector for storing free voxels
    Vector3dVector free_voxels;
    free_voxels.reserve(max({dx, dy, dz}) + 1);

    // Driving axis X
    if (dx >= dy && dx >= dz) {
        int p1 = 2 * dy - dx; // error terms that decide when to step in the other two axes? needed to stay close to the true line.
        int p2 = 2 * dz - dx;
        while (x1 != x2) {
            free_voxels.emplace_back( // marking all intermediate voxels free 
                (x1 + 0.5) * voxel_size_,
                (y1 + 0.5) * voxel_size_,
                (z1 + 0.5) * voxel_size_);
            x1 += xs;
            if (p1 >= 0) { y1 += ys; p1 -= 2 * dx; }
            if (p2 >= 0) { z1 += zs; p2 -= 2 * dx; }
            p1 += 2 * dy;
            p2 += 2 * dz;
        }
    }
    // Driving axis Y
    else if (dy >= dx && dy >= dz) {
        int p1 = 2 * dx - dy;
        int p2 = 2 * dz - dy;
        while (y1 != y2) {
            free_voxels.emplace_back(
                (x1 + 0.5) * voxel_size_,
                (y1 + 0.5) * voxel_size_,
                (z1 + 0.5) * voxel_size_);
            y1 += ys;
            if (p1 >= 0) { x1 += xs; p1 -= 2 * dy; }
            if (p2 >= 0) { z1 += zs; p2 -= 2 * dy; }
            p1 += 2 * dx;
            p2 += 2 * dz;
        }
    }
    // Driving axis Z
    else {
        int p1 = 2 * dy - dz;
        int p2 = 2 * dx - dz;
        while (z1 != z2) {
            free_voxels.emplace_back(
                (x1 + 0.5) * voxel_size_,
                (y1 + 0.5) * voxel_size_,
                (z1 + 0.5) * voxel_size_);
            z1 += zs;
            if (p1 >= 0) { y1 += ys; p1 -= 2 * dz; }
            if (p2 >= 0) { x1 += xs; p2 -= 2 * dz; }
            p1 += 2 * dy;
            p2 += 2 * dx;
        }
    }

	// Bulk insert free voxels
	for (const auto& v : free_voxels) {
		occupancy_map_.emplace(toVoxelKey(v), false); //emplace means if a voxel previously marked occupied, it won’t be overwritten back to free.
	}

	// Mark last voxel as occupied 
	occupancy_map_[VoxelKey{x2, y2, z2}] = true;

}
