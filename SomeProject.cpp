#include <iostream>
#include <unordered_map>
#include <vector>
#include <tuple>
#include <cmath>
#include <memory>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace voxelStruct {


    struct VoxelHash {
        std::size_t operator()(const std::tuple<int, int, int>& key) const {
            return std::get<0>(key) + 31 * std::get<1>(key) + 17 * std::get<2>(key);
        }
    };

    class VoxelHashing {
    public:
        
       VoxelHashing(float voxel_size, float mini_voxel_size, int mini_grid_size)
            : voxel_size_(voxel_size),
            mini_voxel_size_(mini_voxel_size),
            mini_grid_size_(mini_grid_size) {}
       
       void addPoint(const pcl::PointXYZ& point) {
            auto voxel_index = getVoxelIndex(point);
            if (voxel_map_.find(voxel_index) == voxel_map_.end()) {
                voxel_map_[voxel_index] = std::unordered_map<std::tuple<int, int, int>, std::vector<pcl::PointXYZ>, VoxelHash>();
            }
            auto mini_voxel_index = getMiniVoxelIndex(point, voxel_index);
            voxel_map_[voxel_index][mini_voxel_index].push_back(point);
        }
       

       bool IsPointInVoxel(const pcl::PointXYZ& point) {
            auto voxel_index = getVoxelIndex(point);
            auto voxel_it = voxel_map_.find(voxel_index);
            if (voxel_it == voxel_map_.end()) {
                return false;
            }

            auto mini_voxel_index = getMiniVoxelIndex(point, voxel_index);
            const auto& mini_voxel_map = voxel_it->second;
            auto mini_voxel_it = mini_voxel_map.find(mini_voxel_index);

            if (mini_voxel_it == mini_voxel_map.end()) {
                return false;
            }

            const auto& points = mini_voxel_it->second;
            for (const auto& p : points) {
                if (p.x == point.x && p.y == point.y && p.z == point.z) {
                    return true;
                }
            }

            return false;
        }
        

         std::vector<pcl::PointXYZ> selectAllPointsFromVoxel(std::tuple<int, int, int> voxel_index) {
            auto voxel_it = voxel_map_.find(voxel_index);
            std::vector<pcl::PointXYZ> vector;

            if (voxel_it == voxel_map_.end()) {
                return vector;
            }

            const auto& mini_voxel_map = voxel_it->second;

            for (const auto& mini_voxel_pair : mini_voxel_map) {
                const auto& points = mini_voxel_pair.second;
                vector.insert(vector.end(), points.begin(), points.end());
            }

            return vector;
        } 
        

    private:
        float voxel_size_, mini_voxel_size_;
        int mini_grid_size_;

        std::unordered_map<
            std::tuple<int, int, int>,
            std::unordered_map<
            std::tuple<int, int, int>,
            std::vector<pcl::PointXYZ>,
            VoxelHash
            >,
            VoxelHash
        > voxel_map_;

        std::tuple<int, int, int> getVoxelIndex(const pcl::PointXYZ& point) const {
            return std::make_tuple(
                static_cast<int>(std::floor(point.x / voxel_size_)),
                static_cast<int>(std::floor(point.y / voxel_size_)),
                static_cast<int>(std::floor(point.z / voxel_size_))
            );
        } 

       std::tuple<int, int, int> getMiniVoxelIndex(const pcl::PointXYZ& point, const std::tuple<int, int, int>& voxel_index) const {
            float base_x = std::get<0>(voxel_index) * voxel_size_;
            float base_y = std::get<1>(voxel_index) * voxel_size_;
            float base_z = std::get<2>(voxel_index) * voxel_size_;

            int mini_x = static_cast<int>(std::floor((point.x - base_x) / mini_voxel_size_));
            int mini_y = static_cast<int>(std::floor((point.y - base_y) / mini_voxel_size_));
            int mini_z = static_cast<int>(std::floor((point.z - base_z) / mini_voxel_size_));

            return std::make_tuple(mini_x, mini_y, mini_z);
        }
    }; 

}

int main() {
   
}
