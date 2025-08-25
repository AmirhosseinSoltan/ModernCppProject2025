#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>
#include "bresenham.hpp"

using std::cout;
using std::endl;
using std::abs;
using Vector3d = Eigen::Vector3d;
using Vector3dVector = std::vector<Vector3d>;


// Parameters for occupancy probability update
constexpr double P_OCCUPIED     = 0.85; // Probability for an occupied voxel
constexpr double P_FREE         = 0.2; // Probability for a free voxel
constexpr double P_MIN          = 0.2; // Lower bound (to avoid going to 0) for numerical stability
constexpr double P_MAX          = 0.95; // Upper bound (to avoid going to 1)
double log_max                  = OccupancyGrid3D::probToLogOdds(P_MAX);
double log_min                  = OccupancyGrid3D::probToLogOdds(P_MIN);
double log_free                 = OccupancyGrid3D::probToLogOdds(P_FREE) - OccupancyGrid3D::probToLogOdds(0.5);
double log_occ                  = OccupancyGrid3D::probToLogOdds(P_OCCUPIED) - OccupancyGrid3D::probToLogOdds(0.5);


// Updates the voxel's occupancy log-odds in the map
void OccupancyGrid3D::updateVoxelProbability(const VoxelKey& key, double& log_odds_update) {
    double& log_odds = occupancy_map_[key]; 
    log_odds += log_odds_update;

    // Clamping the probability range
    double prob = logOddsToProb(log_odds);
    if (prob >= P_MAX) {log_odds = log_max;}
    else if (prob < P_MIN) {log_odds = log_min;}
}

void OccupancyGrid3D::insertRay(const Vector3d& start, const Vector3d& end) {
    auto s = toVoxelKey(start);
    auto e = toVoxelKey(end);

    int x = s.x, y = s.y, z = s.z;
    int x2 = e.x, y2 = e.y, z2 = e.z;

	int dx = abs(x2 - x);
	int dy = abs(y2 - y);
	int dz = abs(z2 - z);

    int x_inc = (x2 > x) ? 1 : -1;
    int y_inc = (y2 > y) ? 1 : -1;
    int z_inc = (z2 > z) ? 1 : -1;

    int err_1, err_2;

    // Traverse and mark free voxels (probabilistically)
    // Driving axis X
    if (dx >= dy && dx >= dz) {
        err_1 = 2 * dy - dx; 
        err_2 = 2 * dz - dx;

        while (x != x2){
            updateVoxelProbability(VoxelKey{x, y, z}, log_free);
            if (err_1 > 0) { y += y_inc; err_1 -= 2 * dx; }
            if (err_2 > 0) { z += z_inc; err_2 -= 2 * dx; }
            err_1 += 2 * dy;
            err_2 += 2 * dz;
            x += x_inc;
        }
    // Driving axis Y
    } else if ((dy >= dx) && (dy >= dz)) {
        err_1 = 2 * dx - dy;
        err_2 = 2 * dz - dy;

        while ( y != y2){
            updateVoxelProbability(VoxelKey{x, y, z}, log_free);
            if (err_1 > 0) { x += x_inc; err_1 -= 2 * dy; }
            if (err_2 > 0) { z += z_inc; err_2 -= 2 * dy; }
            err_1 += 2 * dx;
            err_2 += 2 * dz;
            y += y_inc;
        }
    // Driving axis Z
    } else {
        err_1 = 2 * dy - dz;
        err_2 = 2 * dx - dz;
        while  (z != z2){
            updateVoxelProbability(VoxelKey{x, y, z}, log_free);
            if (err_1 > 0) { y += y_inc; err_1 -= 2 * dz; }
            if (err_2 > 0) { x += x_inc; err_2 -= 2 * dz; }
            err_1 += 2 * dy;
            err_2 += 2 * dx;
            z += z_inc;
        }
    }

    // Mark last voxel as occupied (probabilistically)
    updateVoxelProbability(VoxelKey{x2, y2, z2}, log_occ);
}

Vector3dVector OccupancyGrid3D::getOccupiedPoints(const double& occ_threshold) 
{
    Vector3dVector pts;
    pts.reserve(occupancy_map_.size());

    for (const auto& [key, log_odds] : occupancy_map_) {
        double prob = logOddsToProb(log_odds);
        if (prob >= occ_threshold) {
            pts.emplace_back(
                (key.x + 0.5) * voxel_size_,
                (key.y + 0.5) * voxel_size_,
                (key.z + 0.5) * voxel_size_);
        }
    }

    cout << "Total voxels in map: " << occupancy_map_.size() << endl;
    cout << "Occupied voxels (>" << occ_threshold << "): " << pts.size() << endl;

    return pts;
}