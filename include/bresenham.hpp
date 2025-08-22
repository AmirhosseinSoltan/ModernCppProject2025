#pragma once
#include <Eigen/Dense>
#include <Eigen/Core>
#include <unordered_map>
#include <vector>
#include <tuple>
#include <cmath>

using namespace std;
using Vector3dVector = vector<Eigen::Vector3d>;
using Vector3d = Eigen::Vector3d;


struct VoxelKey {
    int x;
    int y;
    int z;

    bool operator==(const VoxelKey& other) const noexcept { 
        return x == other.x && y == other.y && z == other.z;
    }
};

//prime multiplier hashing. 
struct VoxelHash {
    size_t operator()(const VoxelKey& k) const noexcept { //overloading operator(),
        // Large prime multipliers for 
        return (static_cast<size_t>(k.x) * 73856093) ^
               (static_cast<size_t>(k.y) * 19349669) ^
               (static_cast<size_t>(k.z) * 83492791);
    }
}; 

class OccupancyGrid3D {

    private:
        double voxel_size_;
        inline VoxelKey toVoxelKey(const Vector3d& pt) const {
            return {
                static_cast<int>(floor(pt.x() / voxel_size_)),
                static_cast<int>(floor(pt.y() / voxel_size_)),
                static_cast<int>(floor(pt.z() / voxel_size_))
            };
        }
        inline double probToLogOdds(double p) const {return log(p / (1.0 - p));} // probability --->  log-odds
        inline double logOddsToProb(double l) const {return 1.0 - (1.0 / (1.0 + exp(l)));} // log-odds --> probability
        
        unordered_map<VoxelKey, double, VoxelHash> occupancy_map_; // Occupancy stored as : { key : log-odds values }

        void updateVoxelProbability(const VoxelKey& key, double log_odds_update);

    public:
        OccupancyGrid3D(double voxel_size) : voxel_size_(voxel_size) {}
        void insertRay(const Vector3d& start, const Vector3d& end);
        Vector3dVector getOccupiedPoints(double occ_threshold) const;
   
    };
