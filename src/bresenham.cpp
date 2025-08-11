#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>
#include "bresenham.hpp"

using namespace std;
// using Key = tuple<int,int,int>;
using Vector3dVector = vector<Eigen::Vector3d>;
using Vector3d = Eigen::Vector3d;


// Parameters for occupancy probability update
constexpr double P_OCCUPIED = 0.7; // Probability for an occupied voxel
constexpr double P_FREE     = 0.2; // Probability for a free voxel
constexpr double P_MIN      = 0.12; // Lower bound (to avoid going to 0) for numerical stability and avoid going to -+inf
constexpr double P_MAX      = 0.97; // Upper bound (to avoid going to 1)

// probability --->  log-odds
static inline double probToLogOdds(double p) {
    return std::log(p / (1.0 - p));
}

// log-odds --> probability
static inline double logOddsToProb(double l) {
    return 1.0 - (1.0 / (1.0 + std::exp(l)));
}

// Updates the voxel's occupancy log-odds in the map
void OccupancyGrid3D::updateVoxelProbability(const VoxelKey& key, double log_odds_update) {
    double& log_odds = occupancy_map_[key]; 
    log_odds += log_odds_update;

    // Clamping the probability range
    double prob = logOddsToProb(log_odds);
    if (prob >= P_MAX) {log_odds = probToLogOdds(P_MAX);}
    else if (prob < P_MIN) {log_odds = probToLogOdds(P_MIN);}
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

    double log_free = probToLogOdds(P_FREE) - probToLogOdds(0.5);
    double log_occ  = probToLogOdds(P_OCCUPIED) - probToLogOdds(0.5);

    // Traverse and mark free voxels (probabilistically)
    // Driving axis X
    if (dx >= dy && dx >= dz) {
        err_1 = 2 * dy - dx; // error terms decide when to step in the other two axes? needed to stay close to the true line.
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

Vector3dVector OccupancyGrid3D::getOccupiedPoints(double occ_threshold) const 
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

    std::cout << "Total voxels in map: " << occupancy_map_.size() << std::endl;
    std::cout << "Occupied voxels (>" << occ_threshold << "): " << pts.size() << std::endl;

    return pts;
}