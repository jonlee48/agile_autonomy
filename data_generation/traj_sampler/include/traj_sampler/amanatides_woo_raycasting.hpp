#pragma once

#include <Eigen/Core>
#include <open3d/Open3D.h>
#include <random>

using namespace std;
namespace o3d = open3d;

bool rayBoxIntersection(const Eigen::Vector3d& ray_origin, const Eigen::Vector3d& ray_direction, const o3d::geometry::VoxelGrid& grid, double& tMin, double& tMax,
                        double t0, double t1) {
    double tYMin, tYMax, tZMin, tZMax;
    const double x_inv_dir = 1 / ray_direction.x();
    if (x_inv_dir >= 0) {
        tMin = (grid.GetMinBound().x() - ray_origin.x()) * x_inv_dir;
        tMax = (grid.GetMaxBound().x() - ray_origin.x()) * x_inv_dir;
    } else {
        tMin = (grid.GetMaxBound().x() - ray_origin.x()) * x_inv_dir;
        tMax = (grid.GetMinBound().x() - ray_origin.x()) * x_inv_dir;
    }

    const double y_inv_dir = 1 / ray_direction.y();
    if (y_inv_dir >= 0) {
        tYMin = (grid.GetMinBound().y() - ray_origin.y()) * y_inv_dir;
        tYMax = (grid.GetMaxBound().y() - ray_origin.y()) * y_inv_dir;
    } else {
        tYMin = (grid.GetMaxBound().y() - ray_origin.y()) * y_inv_dir;
        tYMax = (grid.GetMinBound().y() - ray_origin.y()) * y_inv_dir;
    }

    if (tMin > tYMax || tYMin > tMax) return false;
    if (tYMin > tMin) tMin = tYMin;
    if (tYMax < tMax) tMax = tYMax;

    const double z_inv_dir = 1 / ray_direction.z();
    if (z_inv_dir >= 0) {
        tZMin = (grid.GetMinBound().z() - ray_origin.z()) * z_inv_dir;
        tZMax = (grid.GetMaxBound().z() - ray_origin.z()) * z_inv_dir;
    } else {
        tZMin = (grid.GetMaxBound().z() - ray_origin.z()) * z_inv_dir;
        tZMax = (grid.GetMinBound().z() - ray_origin.z()) * z_inv_dir;
    }

    if (tMin > tZMax || tZMin > tMax) return false;
    if (tZMin > tMin) tMin = tZMin;
    if (tZMax < tMax) tMax = tZMax;
    return (tMin < t1 && tMax > t0);
}

std::vector<Eigen::Vector3i> amanatidesWooAlgorithm(const Eigen::Vector3d& ray_origin, const Eigen::Vector3d& ray_direction, const o3d::geometry::VoxelGrid& grid, double t0, double t1) noexcept {
//    std::cout << "Entered amanatidesWooAlgorithm" << std::endl;
    double tMin;
    double tMax;
    const bool ray_intersects_grid = rayBoxIntersection(ray_origin, ray_direction, grid, tMin, tMax, t0, t1);
    if (!ray_intersects_grid) {
//        std::cout << "amanatidesWooAlgorithm failed! !ray_intersects_grid" << std::endl;
        return {};
    }

    tMin = std::max(tMin, t0);
    tMax = std::max(tMax, t1);
    const Eigen::Vector3d ray_start = ray_origin + ray_direction * tMin;
    const Eigen::Vector3d ray_end = ray_origin + ray_direction * tMax;

    Eigen::Vector3i current_index = grid.GetVoxel(ray_origin);

    Eigen::Vector3d idx_cent;
    idx_cent.x() = grid.origin_.x() + current_index.x() * grid.voxel_size_;
    idx_cent.y() = grid.origin_.y() + current_index.y() * grid.voxel_size_;
    idx_cent.z() = grid.origin_.z() + current_index.z() * grid.voxel_size_;

    size_t current_X_index = current_index.x();
    const size_t end_X_index = std::max(1.0, std::ceil(ray_end.x() - grid.GetMinBound().x() / grid.voxel_size_));
    int stepX;
    double tDeltaX;
    double tMaxX;
    if (ray_direction.x() > 0.0) {
        stepX = 1;
        tDeltaX = grid.voxel_size_ / ray_direction.x();
        tMaxX = tMin + (grid.GetMinBound().x() + current_X_index * grid.voxel_size_
                        - ray_start.x()) / ray_direction.x();
    } else if (ray_direction.x() < 0.0) {
        stepX = -1;
        tDeltaX = grid.voxel_size_ / -ray_direction.x();
        const size_t previous_X_index = current_X_index - 1;
        tMaxX = tMin + (grid.GetMinBound().x() + previous_X_index * grid.voxel_size_
                        - ray_start.x()) / ray_direction.x();
    } else {
        stepX = 0;
        tDeltaX = tMax;
        tMaxX = tMax;
    }

    size_t current_Y_index = current_index.y();
    const size_t end_Y_index = std::max(1.0, std::ceil(ray_end.y() - grid.GetMinBound().y() / grid.voxel_size_));
    int stepY;
    double tDeltaY;
    double tMaxY;
    if (ray_direction.y() > 0.0) {
        stepY = 1;
        tDeltaY = grid.voxel_size_ / ray_direction.y();
        tMaxY = tMin + (grid.GetMinBound().y() + current_Y_index * grid.voxel_size_
                        - ray_start.y()) / ray_direction.y();
    } else if (ray_direction.y() < 0.0) {
        stepY= -1;
        tDeltaY = grid.voxel_size_ / -ray_direction.y();
        const size_t previous_Y_index = current_Y_index - 1;
        tMaxY = tMin + (grid.GetMinBound().y() + previous_Y_index * grid.voxel_size_
                        - ray_start.y()) / ray_direction.y();
    } else {
        stepY = 0;
        tDeltaY = tMax;
        tMaxY = tMax;
    }

    size_t current_Z_index = current_index.z();
    const size_t end_Z_index = std::max(1.0, std::ceil(ray_end.z() - grid.GetMinBound().z() / grid.voxel_size_));
    int stepZ;
    double tDeltaZ;
    double tMaxZ;
    if (ray_direction.z() > 0.0) {
        stepZ = 1;
        tDeltaZ = grid.voxel_size_ / ray_direction.z();
        tMaxZ = tMin + (grid.GetMinBound().z() + current_Z_index * grid.voxel_size_
                        - ray_start.z()) / ray_direction.z();
    } else if (ray_direction.z() < 0.0) {
        stepZ = -1;
        tDeltaZ = grid.voxel_size_ / -ray_direction.z();
        const size_t previous_Z_index = current_Z_index - 1;
        tMaxZ = tMin + (grid.GetMinBound().z() + previous_Z_index * grid.voxel_size_
                        - ray_start.z()) / ray_direction.z();
    } else {
        stepZ = 0;
        tDeltaZ = tMax;
        tMaxZ = tMax;
    }

    std::vector<Eigen::Vector3i> unoccupiedVoxelsIndices;

    int num_voxels_expanded = 0;
    int max_dist = 0;
//    while (grid.voxels_.count({current_X_index, current_Y_index, current_Z_index}) == 0 && num_voxels_expanded < 100 && (tMaxX*tMaxX + tMaxY * tMaxY + tMaxZ * tMaxZ) < 10*10) {
    while (grid.voxels_.count({current_X_index, current_Y_index, current_Z_index}) == 0 && num_voxels_expanded < 30) {
        num_voxels_expanded++;
        if (tMaxX < tMaxY && tMaxX < tMaxZ) {
            // X-axis traversal.
            current_X_index += stepX;
            tMaxX += tDeltaX;
        } else if (tMaxY < tMaxZ) {
            // Y-axis traversal.
            current_Y_index += stepY;
            tMaxY += tDeltaY;
        } else {
            // Z-axis traversal.
            current_Z_index += stepZ;
            tMaxZ += tDeltaZ;
        }
        unoccupiedVoxelsIndices.push_back({current_X_index, current_Y_index, current_Z_index});
    }
//    std::cout << "Exiting because: " << std::endl;
//    std::cout << "grid.voxels_.count({current_X_index, current_Y_index, current_Z_index}) != 0 " << (grid.voxels_.count({current_X_index, current_Y_index, current_Z_index}) != 0) << std::endl;
//    std::cout << "num_voxels_expanded >= 100 " << (num_voxels_expanded >= 100) << std::endl;
//    std::cout << "(tMaxX*tMaxX + tMaxY * tMaxY + tMaxZ * tMaxZ) >= 10*10) " << ((tMaxX*tMaxX + tMaxY * tMaxY + tMaxZ * tMaxZ) >= 10*10) << std::endl;
//    std::cout << "Exited amanatidesWooAlgorithm" << std::endl;
    return unoccupiedVoxelsIndices;
}
