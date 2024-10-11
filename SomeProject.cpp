

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

     
        std::vector<pcl::PointXYZ> radiusSearch(const pcl::PointXYZ& query_point, float radius) {
            std::vector<pcl::PointXYZ> neighbors;
            int voxel_range = std::ceil(radius / voxel_size_);

            auto query_voxel = getVoxelIndex(query_point);

           
            for (int i = -voxel_range; i <= voxel_range; ++i) {
                for (int j = -voxel_range; j <= voxel_range; ++j) {
                    for (int k = -voxel_range; k <= voxel_range; ++k) {
                        std::tuple<int, int, int> neighbor_voxel(std::get<0>(query_voxel) + i, std::get<1>(query_voxel) + j, std::get<2>(query_voxel) + k);


                      
                        if (voxel_map_.find(neighbor_voxel) != voxel_map_.end()) {
                            for (const auto& point : voxel_map_[neighbor_voxel]) {
                                if (pcl::geometry::squaredDistance(query_point, point) <= radius * radius) {
                                    neighbors.push_back(point);
                                }
                            }
                        }
                    }
                }
            }

            return neighbors;
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


