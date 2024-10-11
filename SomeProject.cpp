

#include <iostream>
#include <unordered_map>
#include <vector>
#include <tuple>
#include <cmath>
#include <memory> 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/geometry.h>


namespace voxelStruct {
   
    struct VoxelHash {
        std::size_t operator()(const std::tuple<int, int, int>& key) const {
            return std::get<0>(key) + 31 * std::get<1>(key) + 17 * std::get<2>(key);
        }
    };

    class VoxelHashing {
    public:
        VoxelHashing(float voxel_size) : voxel_size_(voxel_size) {}
        
        void addPoint(const pcl::PointXYZ& point) {
            auto voxel_index = getVoxelIndex(point);
            voxel_map_[voxel_index].push_back(point);
        }



    private:
        float voxel_size_; 

       
        std::unordered_map<std::tuple<int, int, int>, std::vector<pcl::PointXYZ>, VoxelHash> voxel_map_;

        std::tuple<int, int, int> getVoxelIndex(const pcl::PointXYZ& point) const {
            return std::make_tuple(
                static_cast<int>(std::floor(point.x / voxel_size_)),
                static_cast<int>(std::floor(point.y / voxel_size_)),
                static_cast<int>(std::floor(point.z / voxel_size_))
            );
        }
    };

}


int main()
{
}


