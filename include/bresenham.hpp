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


// struct TupleHash {
//     size_t operator()(const std::tuple<int,int,int>& k) const noexcept {
//       auto [x,y,z] = k;
//       size_t h1 = std::hash<int>()(x);
//       size_t h2 = std::hash<int>()(y);
//       size_t h3 = std::hash<int>()(z);
//       return h1 ^ (h2 << 1) ^ (h3 << 2);
//     }
//   };

struct VoxelKey {
    int x;
    int y;
    int z;

    bool operator==(const VoxelKey& other) const noexcept { //container needs to check whether two keys that land in the same bucket are actually the same. 
        return x == other.x && y == other.y && z == other.z;
    }
};
//prime multiplier hashing.
struct VoxelHash {
    size_t operator()(const VoxelKey& k) const noexcept {
        // Large prime multipliers for 
        return (static_cast<size_t>(k.x) * 73856093) ^
               (static_cast<size_t>(k.y) * 19349663) ^
               (static_cast<size_t>(k.z) * 83492791);
    }
}; 
//Packed-bit hashing is basically a way to store (x, y, z) directly inside one integer by shifting their bits into separate sections instead of multiplying by primes
// struct VoxelHash {
//     size_t operator()(const VoxelKey& k) const noexcept {
//         // Shift coordinates into positive space
//         uint64_t ux = static_cast<uint64_t>(k.x + 1048576); // 2^20
//         uint64_t uy = static_cast<uint64_t>(k.y + 1048576);
//         uint64_t uz = static_cast<uint64_t>(k.z + 1048576);

//         return (ux << 42) | (uy << 21) | uz; // 64-bit packed
//     }
// };

class OccupancyGrid3D {
    public:

        OccupancyGrid3D(double voxel_size) : voxel_size_(voxel_size) {}
        void insertRay(const Vector3d& start, const Vector3d& end);
        Vector3dVector getOccupiedPoints(double occ_threshold) const;

    
    private:
        double voxel_size_;

        // using Key = std::tuple<int,int,int>;
        // std::unordered_map<Key, bool, TupleHash> occupancy_map_; // {point_cordinate : occupancy_status}
// /        Key toVoxelKey(const Vector3d& pt) const;       
        
        inline VoxelKey toVoxelKey(const Vector3d& pt) const {
            return {
                static_cast<int>(std::floor(pt.x() / voxel_size_)),
                static_cast<int>(std::floor(pt.y() / voxel_size_)),
                static_cast<int>(std::floor(pt.z() / voxel_size_))
            };
        }

        // std::unordered_map<VoxelKey, bool, VoxelHash> occupancy_map_;
        
        // Occupancy stored as log-odds values
        std::unordered_map<VoxelKey, double, VoxelHash> occupancy_map_;

        void updateVoxelProbability(const VoxelKey& key, double log_odds_update);

    };
