#include <Eigen/Dense>
#include <open3d/Open3D.h>
#include <iostream>
#include "dataloader.hpp"
#include "transformer.hpp"
#include <algorithm>

#include <Eigen/Core>
#include <unordered_map>
#include <vector>
#include <tuple>
#include <cmath>
#include <cstdlib>
#include "bresenham.hpp"
using namespace std;
using Key = tuple<int,int,int>;
using Vector3dVector = vector<Eigen::Vector3d>;
using Vector3d = Eigen::Vector3d;

// Return voxel centers for visualization
Vector3dVector OccupancyGrid3D::getOccupiedPoints() const 
{
	Vector3dVector pts;
	pts.reserve(occupancy_map_.size());
	for (const auto& [point,status] : occupancy_map_) {
		if (status) {
			auto [ix, iy, iz] = point;
			pts.emplace_back(
				(ix + 0.5) * voxel_size_,
				(iy + 0.5) * voxel_size_,
				(iz + 0.5) * voxel_size_);
		}
	}
	std::cout << "Total voxels in map: " << occupancy_map_.size() << std::endl;
	std::cout << "Occupied voxels: " << pts.size() << std::endl;
	return pts;
}


Key OccupancyGrid3D::toVoxelKey(const Vector3d& pt) const
{
            return {
                static_cast<int>(std::floor(pt.x() / voxel_size_)), //converting to int
                static_cast<int>(std::floor(pt.y() / voxel_size_)),
                static_cast<int>(std::floor(pt.z() / voxel_size_))
            };
}

// Mark voxel as occupied
void OccupancyGrid3D::insertPoint(const Vector3d& point) {
	auto key = toVoxelKey(point);
	occupancy_map_[key] = true;
}

void OccupancyGrid3D::insertRay(const Vector3d& start, const Vector3d& end) {
    auto s = toVoxelKey(start);
    auto e = toVoxelKey(end);

    int x1 = std::get<0>(s);
    int y1 = std::get<1>(s);
    int z1 = std::get<2>(s);
    int x2 = std::get<0>(e);
    int y2 = std::get<1>(e);
    int z2 = std::get<2>(e);

    // Debug: Check if start and end are the same voxel
    if (x1 == x2 && y1 == y2 && z1 == z2) {
        // Same voxel, just mark as occupied
        insertOccupiedVoxel(x1, y1, z1);
        return;
    }

    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int dz = std::abs(z2 - z1);

    int xs = (x2 > x1) ? 1 : -1;
    int ys = (y2 > y1) ? 1 : -1;
    int zs = (z2 > z1) ? 1 : -1;

    // Driving axis X
    if (dx >= dy && dx >= dz) {
        int p1 = 2 * dy - dx;
        int p2 = 2 * dz - dx;
        while (x1 != x2) {
            insertFreeVoxel(x1, y1, z1); // Mark free
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
            insertFreeVoxel(x1, y1, z1);
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
            insertFreeVoxel(x1, y1, z1);
            z1 += zs;
            if (p1 >= 0) { y1 += ys; p1 -= 2 * dz; }
            if (p2 >= 0) { x1 += xs; p2 -= 2 * dz; }
            p1 += 2 * dy;
            p2 += 2 * dx;
        }
    }

    // Last voxel is occupied
    insertOccupiedVoxel(x2, y2, z2);
}

