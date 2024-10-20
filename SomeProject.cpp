﻿#include <iostream>
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

    template <typename PointT>
    class VoxelHashing {
    public:
        VoxelHashing(float voxel_size, float mini_voxel_size, int mini_grid_size)
            : voxel_size_(voxel_size),
            mini_voxel_size_(mini_voxel_size),
            mini_grid_size_(mini_grid_size) {}

        void addPoint(const PointT& point) {
            auto voxel_index = getVoxelIndex(point);
            if (voxel_map_.find(voxel_index) == voxel_map_.end()) {
                voxel_map_[voxel_index] = std::unordered_map<std::tuple<int, int, int>, std::vector<PointT>, VoxelHash>();
            }
            auto mini_voxel_index = getMiniVoxelIndex(point, voxel_index);
            voxel_map_[voxel_index][mini_voxel_index].push_back(point);
        }

        bool IsPointInVoxel(const PointT& point) {
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

        std::vector<PointT> selectAllPointsFromVoxel(const std::tuple<int, int, int>& voxel_index) {
            auto voxel_it = voxel_map_.find(voxel_index);
            std::vector<PointT> vector;

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
            std::unordered_map<std::tuple<int, int, int>, std::vector<PointT>, VoxelHash>,
            VoxelHash
        > voxel_map_;



        std::tuple<int, int, int> getVoxelIndex(const PointT& point) const {
            return std::make_tuple(
                static_cast<int>(std::floor(point.x / voxel_size_)),
                static_cast<int>(std::floor(point.y / voxel_size_)),
                static_cast<int>(std::floor(point.z / voxel_size_))
            );
        }


        std::tuple<int, int, int> getMiniVoxelIndex(const PointT& point, const std::tuple<int, int, int>& voxel_index) const {
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

    std::cout << "Testing with pcl::PointXYZ:" << std::endl;
    voxelStruct::VoxelHashing<pcl::PointXYZ> voxel_hashing_xyz(0.1f, 0.01f, 10);


    pcl::PointXYZ point1(0.05f, 0.05f, 0.05f);
    pcl::PointXYZ point2(0.07f, 0.05f, 0.05f);
    pcl::PointXYZ point3(0.08f, 0.06f, 0.05f);
    voxel_hashing_xyz.addPoint(point1);
    voxel_hashing_xyz.addPoint(point2);
    voxel_hashing_xyz.addPoint(point3);


    std::cout << "Is point1 in voxel? "
        << (voxel_hashing_xyz.IsPointInVoxel(point1) ? "Yes" : "No") << std::endl;
    std::cout << "Is point2 in voxel? "
        << (voxel_hashing_xyz.IsPointInVoxel(point2) ? "Yes" : "No") << std::endl;
    std::cout << "Is point3 in voxel? "
        << (voxel_hashing_xyz.IsPointInVoxel(point3) ? "Yes" : "No") << std::endl;


    std::tuple<int, int, int> voxel_index_xyz = std::make_tuple(0, 0, 0);
    auto points_xyz = voxel_hashing_xyz.selectAllPointsFromVoxel(voxel_index_xyz);

    std::cout << "Extracted points (PointXYZ):" << std::endl;
    for (const auto& p : points_xyz) {
        std::cout << "(" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
    }


    std::cout << "\nTesting with pcl::PointXYZRGB:" << std::endl;
    voxelStruct::VoxelHashing<pcl::PointXYZRGB> voxel_hashing_xyzrgb(0.1f, 0.01f, 10);

    pcl::PointXYZRGB point_rgb1;
    point_rgb1.x = 0.05f; point_rgb1.y = 0.05f; point_rgb1.z = 0.05f;
    point_rgb1.r = 255; point_rgb1.g = 0; point_rgb1.b = 0;

    pcl::PointXYZRGB point_rgb2;
    point_rgb2.x = 0.06f; point_rgb2.y = 0.05f; point_rgb2.z = 0.05f;
    point_rgb2.r = 0; point_rgb2.g = 255; point_rgb2.b = 0;

    voxel_hashing_xyzrgb.addPoint(point_rgb1);
    voxel_hashing_xyzrgb.addPoint(point_rgb2);


    std::cout << "Is point_rgb1 in voxel? "
        << (voxel_hashing_xyzrgb.IsPointInVoxel(point_rgb1) ? "Yes" : "No") << std::endl;
    std::cout << "Is point_rgb2 in voxel? "
        << (voxel_hashing_xyzrgb.IsPointInVoxel(point_rgb2) ? "Yes" : "No") << std::endl;


    std::tuple<int, int, int> voxel_index_xyzrgb = std::make_tuple(0, 0, 0);
    auto points_xyzrgb = voxel_hashing_xyzrgb.selectAllPointsFromVoxel(voxel_index_xyzrgb);

    std::cout << "Extracted points (PointXYZRGB):" << std::endl;
    for (const auto& p : points_xyzrgb) {
        std::cout << "(" << p.x << ", " << p.y << ", " << p.z << ") - Color: ("
            << static_cast<int>(p.r) << ", "
            << static_cast<int>(p.g) << ", "
            << static_cast<int>(p.b) << ")" << std::endl;
    }


    std::cout << "\nTesting with pcl::PointNormal:" << std::endl;
    voxelStruct::VoxelHashing<pcl::PointNormal> voxel_hashing_normal(0.1f, 0.01f, 10);

    pcl::PointNormal point_normal1;
    point_normal1.x = 0.05f; point_normal1.y = 0.05f; point_normal1.z = 0.05f;
    point_normal1.normal_x = 1.0f; point_normal1.normal_y = 0.0f; point_normal1.normal_z = 0.0f;

    pcl::PointNormal point_normal2;
    point_normal2.x = 0.07f; point_normal2.y = 0.05f; point_normal2.z = 0.06f;
    point_normal2.normal_x = 0.0f; point_normal2.normal_y = 1.0f; point_normal2.normal_z = 0.0f;

    voxel_hashing_normal.addPoint(point_normal1);
    voxel_hashing_normal.addPoint(point_normal2);


    std::cout << "Is point_normal1 in voxel? "
        << (voxel_hashing_normal.IsPointInVoxel(point_normal1) ? "Yes" : "No") << std::endl;
    std::cout << "Is point_normal2 in voxel? "
        << (voxel_hashing_normal.IsPointInVoxel(point_normal2) ? "Yes" : "No") << std::endl;


    std::tuple<int, int, int> voxel_index_normal = std::make_tuple(0, 0, 0);
    auto points_normal = voxel_hashing_normal.selectAllPointsFromVoxel(voxel_index_normal);

    std::cout << "Extracted points (PointNormal):" << std::endl;
    for (const auto& p : points_normal) {
        std::cout << "(" << p.x << ", " << p.y << ", " << p.z << ") - Normal: ("
            << p.normal_x << ", "
            << p.normal_y << ", "
            << p.normal_z << ")" << std::endl;
    }

    return 0;
}

/*
Testing with pcl::PointXYZ:
Is point1 in voxel? Yes
Is point2 in voxel? Yes
Is point3 in voxel? Yes
Extracted points (PointXYZ):
(0.05, 0.05, 0.05)
(0.08, 0.06, 0.05)
(0.07, 0.05, 0.05)

Testing with pcl::PointXYZRGB:
Is point_rgb1 in voxel? Yes
Is point_rgb2 in voxel? Yes
Extracted points (PointXYZRGB):
(0.05, 0.05, 0.05) - Color: (255, 0, 0)
(0.06, 0.05, 0.05) - Color: (0, 255, 0)

Testing with pcl::PointNormal:
Is point_normal1 in voxel? Yes
Is point_normal2 in voxel? Yes
Extracted points (PointNormal):
(0.05, 0.05, 0.05) - Normal: (1, 0, 0)
(0.07, 0.05, 0.06) - Normal: (0, 1, 0)
*/
