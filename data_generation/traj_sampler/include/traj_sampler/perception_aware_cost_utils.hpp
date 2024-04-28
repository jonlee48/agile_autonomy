#pragma once

#include <chrono>
#include <Eigen/Core>
#include <iostream>
#include <open3d/Open3D.h>
#include <random>

#include "../../src/example-utils.hpp"
#include "../../src/tinyply.h"

#include "traj_sampler/amanatides_woo_raycasting.hpp"
#include "traj_sampler/progressbar.hpp"


using namespace std;
namespace o3d = open3d;


std::vector<Eigen::Vector3d> parse_pointcloud(const std::string pointcloud_filename, double z_cutoff=numeric_limits<double>::infinity()) {
  Eigen::MatrixXd points_;

  std::ifstream csvFile;
  csvFile.open(pointcloud_filename.c_str());

  if (!csvFile.is_open()) {
    std::cout << "Path Wrong!!!!" << std::endl;
    std::cout << pointcloud_filename << std::endl;
    exit(EXIT_FAILURE);
  }

  open3d::geometry::PointCloud pointcloud;

  std::cout << "Now Reading: " << pointcloud_filename << std::endl;

  std::unique_ptr<std::istream> file_stream;
  std::vector<uint8_t> byte_buffer;
  std::vector<Eigen::Vector3d> points_vec, normals_vec, colors_vec;

  try {
    // For most files < 1gb, pre-loading the entire file upfront and wrapping it
    // into a stream is a net win for parsing speed, about 40% faster.
    bool preload_into_memory = true;
    if (preload_into_memory) {
      byte_buffer = read_file_binary(pointcloud_filename);
      file_stream.reset(
          new memory_stream((char *)byte_buffer.data(), byte_buffer.size()));
    } else {
      file_stream.reset(
          new std::ifstream(pointcloud_filename, std::ios::binary));
    }

    if (!file_stream || file_stream->fail())
      throw std::runtime_error("file_stream failed to open " +
                               pointcloud_filename);

    file_stream->seekg(0, std::ios::end);
    const double size_mb = file_stream->tellg() * double(1e-6);
    file_stream->seekg(0, std::ios::beg);

    PlyFile file;
    file.parse_header(*file_stream);

    // Because most people have their own mesh types, tinyply treats parsed data
    // as structured/typed byte buffers. See examples below on how to marry your
    // own application-specific data structures with this one.
    std::shared_ptr<PlyData> vertices, normals, colors, texcoords, faces,
        tripstrip;

    // The header information can be used to programmatically extract properties
    // on elements known to exist in the header prior to reading the data. For
    // brevity of this sample, properties like vertex position are hard-coded:
    try {
      vertices =
          file.request_properties_from_element("vertex", {"x", "y", "z"});
    } catch (const std::exception &e) {
      //      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    try {
      normals =
          file.request_properties_from_element("vertex", {"nx", "ny", "nz"});
    } catch (const std::exception &e) {
//            std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    try {
      colors = file.request_properties_from_element(
          "vertex", {"red", "green", "blue", "alpha"});
    } catch (const std::exception &e) {
//            std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    try {
      colors =
          file.request_properties_from_element("vertex", {"r", "g", "b", "a"});
    } catch (const std::exception &e) {
      //      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    try {
      texcoords = file.request_properties_from_element("vertex", {"u", "v"});
    } catch (const std::exception &e) {
      //      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    // Providing a list size hint (the last argument) is a 2x performance
    // improvement. If you have arbitrary ply files, it is best to leave this 0.
    try {
      faces =
          file.request_properties_from_element("face", {"vertex_indices"}, 3);
    } catch (const std::exception &e) {
//            std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    // Tristrips must always be read with a 0 list size hint (unless you know
    // exactly how many elements are specifically in the file, which is
    // unlikely);
    try {
      tripstrip = file.request_properties_from_element("tristrips",
                                                       {"vertex_indices"}, 0);
    } catch (const std::exception &e) {
      //      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    manual_timer read_timer;

    read_timer.start();
    file.read(*file_stream);
    read_timer.stop();

    const double parsing_time = read_timer.get() / 1000.f;
    std::cout << "\tparsing " << size_mb << "mb in " << parsing_time
              << " seconds [" << (size_mb / parsing_time) << " MBps]"
              << std::endl;

    if (vertices)
      std::cout << "\tRead " << vertices->count << " total vertices "
                << std::endl;

    const size_t numVerticesBytes = vertices->buffer.size_bytes();
    std::vector<float3> verts(vertices->count);
    std::memcpy(verts.data(), vertices->buffer.get(), numVerticesBytes);

    int idx = 0;
    for (auto point_tinyply : verts) {
      if (point_tinyply.z < 3.5){
          points_vec.push_back(
              Eigen::Vector3d(static_cast<double>(point_tinyply.x),
                              static_cast<double>(point_tinyply.y),
                              static_cast<double>(point_tinyply.z)));
      }
      idx += 1;
    }
  } catch (const std::exception &e) {
    std::cerr << "Caught tinyply exception: " << e.what() << std::endl;
  }

  std::cout << "Completed pointcloud parsing!" << std::endl;
  return points_vec;
}

Eigen::Quaterniond euler_to_quat(Eigen::Vector3d rpy){
    return Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());
}

Eigen::Vector3d quat_to_euler(Eigen::Quaterniond quat){
    auto ypr = quat.toRotationMatrix().eulerAngles(2, 1, 0);
    Eigen::Vector3d rpy;
    rpy.x() = ypr.z();
    rpy.y() = ypr.y();
    rpy.z() = ypr.x();
    return rpy;
}

// Function to parse CSV file and extract trajectory (x, y, z) data
std::vector<std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>> parse_trajectory_from_csv(const std::string& csv_filename) {

    std::vector<std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>> trajectories;

    // Open CSV file for reading
    std::ifstream file(csv_filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << csv_filename << std::endl;
        return {{}, {}};  // Return empty trajectory if file cannot be opened
    }
    std::string line;
    // Read CSV file line by line
    std::getline(file, line);
    while (std::getline(file, line)) {
        std::vector<Eigen::Vector3d> trajectory_pos;
        std::vector<Eigen::Vector3d> trajectory_rpy;

        std::istringstream iss(line);
        std::string token;

        // Extract trajectory data from each line (assuming CSV format)
        std::vector<double> values;
        while (std::getline(iss, token, ',')) {
            values.push_back(std::stod(token));
        }
        for (int i = 0; i < 11; i++){

            Eigen::Vector3d position(values[17 * i], values[17 * i + 1], values[17 * i + 2]);
            Eigen::Quaterniond quat(values[17 * i + 9], values[17 * i + 10], values[17 * i + 11], values[17 * i + 12]);
            trajectory_pos.push_back(position);
            trajectory_rpy.push_back(quat_to_euler(quat));
        }
        trajectories.push_back({trajectory_pos, trajectory_rpy});
//        if (values.size() >= 4) {  // Ensure line contains at least pos_x, pos_y, pos_z
//            Eigen::Vector3d position(values[1], values[2], values[3]);
//            trajectory.push_back(position);
//        }
    }
//    std::cout << "===============================================================" << std::endl;
//    std::cout << "Trajectory loaded from: " << csv_filename << std::endl;
//    std::cout << "Number of points in trajectory: " << trajectory_pos.size() << std::endl;
    return trajectories;
}

double MapValue(double value, double min_in, double max_in, double min_out, double max_out) {
    return min_out + (max_out - min_out) * ((value - min_in) / (max_in - min_in));
}

// HSV (hue, saturation, value) to RGB (red, green, blue) conversion
Eigen::Vector3d HsvToRgb(double h, double s, double v) {
    // Normalize hue to be in the range [0, 1]
    h = std::fmod(h, 1.0);
    if (h < 0.0) {
        h += 1.0;
    }

    // Calculate chroma (color intensity)
    double c = v * s;

    // Calculate hue sector (0 to 5)
    double hue_sector = 6.0 * h;
    int hue_sector_floor = static_cast<int>(std::floor(hue_sector));
    double f = hue_sector - hue_sector_floor;
    double p = v * (1.0 - s);
    double q = v * (1.0 - s * f);
    double t = v * (1.0 - s * (1.0 - f));

    // Calculate RGB components based on hue sector
    double r, g, b;
    switch (hue_sector_floor) {
        case 0:
            r = v;
            g = t;
            b = p;
            break;
        case 1:
            r = q;
            g = v;
            b = p;
            break;
        case 2:
            r = p;
            g = v;
            b = t;
            break;
        case 3:
            r = p;
            g = q;
            b = v;
            break;
        case 4:
            r = t;
            g = p;
            b = v;
            break;
        case 5:
            r = v;
            g = p;
            b = q;
            break;
        default:
            r = g = b = 0.0;
            break;
    }

    return Eigen::Vector3d(r, g, b);
}

Eigen::Matrix3d rpy2rot(Eigen::Vector3d camera_rpy) {
    double roll = camera_rpy(0);
    double pitch = camera_rpy(1);
    double yaw = camera_rpy(2);

    // Calculate sin and cos values
    double sr = sin(roll);
    double cr = cos(roll);
    double sp = sin(pitch);
    double cp = cos(pitch);
    double sy = sin(yaw);
    double cy = cos(yaw);

    // Construct the rotation matrix
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix(0, 0) = cy * cp;
    rotation_matrix(0, 1) = cy * sp * sr - sy * cr;
    rotation_matrix(0, 2) = cy * sp * cr + sy * sr;
    rotation_matrix(1, 0) = sy * cp;
    rotation_matrix(1, 1) = sy * sp * sr + cy * cr;
    rotation_matrix(1, 2) = sy * sp * cr - cy * sr;
    rotation_matrix(2, 0) = -sp;
    rotation_matrix(2, 1) = cp * sr;
    rotation_matrix(2, 2) = cp * cr;

    return rotation_matrix;
}

void generate_rays(const Eigen::Vector3d& camera_pos, const Eigen::Vector3d& camera_rpy,
                   double vfov_deg, double hfov_deg, int image_width, int image_height, std::vector<Eigen::Vector3d>& ray_directions) {
    double d = 2, dyz = 0.6;
    double vfov_rad = vfov_deg * M_PI / 180.0, hfov_rad = hfov_deg * M_PI / 180.0;
    double ylim = d * tan(hfov_rad / 2.0), zlim = d * tan(vfov_rad / 2.0);
    double z = -zlim;
    while (z < zlim) {
        double y = -ylim;
        while (y < ylim) {
            ray_directions.push_back(rpy2rot(camera_rpy) * Eigen::Vector3d(d, y, z));
            y += dyz;
        }
        z += dyz;
    }
}

//void generate_rays(const Eigen::Vector3d& camera_pos, const Eigen::Vector3d& camera_rpy,
//                   double vfov_deg, double hfov_deg, int image_width, int image_height, std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& ray_origin_directions) {
//    double d = 2, dyz = 0.5;
//    double vfov_rad = vfov_deg * M_PI / 180.0, hfov_rad = hfov_deg * M_PI / 180.0;
//    double ylim = d * tan(hfov_rad / 2.0), zlim = d * tan(vfov_rad / 2.0);
//    double z = -zlim;
//    while (z < zlim) {
//        double y = -ylim;
//        while (y < ylim) {
//            ray_origin_directions.push_back({camera_pos, rpy2rot(camera_rpy) * Eigen::Vector3d(d, y, z)});
//            y += dyz;
//        }
//        z += dyz;
//    }
//}

// Hash function for Eigen matrix and vector.
// The code is from `hash_combine` function of the Boost library. See
// http://www.boost.org/doc/libs/1_55_0/doc/html/hash/reference.html#boost.hash_combine .
template<typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    // Note that it is oblivious to the storage order of Eigen matrix (column- or
    // row-major). It will give you the same hash value for two different matrices if they
    // are the transpose of each other in different storage order.
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

bool RotateView(o3d::visualization::Visualizer *vis) {
    vis->GetViewControl().Rotate(1.0, 0.0); // Rotate by 10 degrees around the Y-axis
    return false; // Return false to continue the animation
}

void color_pointcloud_z_gradient(o3d::geometry::PointCloud& point_cloud){
    auto& points = point_cloud.points_;
    std::vector<double> z_values(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        z_values[i] = points[i][2]; // Assuming z-coordinate is the third component (0-indexed)
    }
    double min_z = *std::min_element(z_values.begin(), z_values.end());
    double max_z = *std::max_element(z_values.begin(), z_values.end());
    std::vector<Eigen::Vector3d> colors(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        double normalized_z = (z_values[i] - min_z) / (max_z - min_z);
        double value = MapValue(normalized_z, 0.0, 1.0, 0.8, 0.2); // Saturation decreases with z
        double saturation = 0.05; // Keep value constant at 1.0 for full intensity
        colors[i] = HsvToRgb(0.7, saturation, value); // Constant hue of 0.0 for red
    }
    point_cloud.colors_ = colors;
}

std::vector<std::vector<Eigen::Vector3i>> ray_cast(const Eigen::Vector3d& ray_origin, std::vector<Eigen::Vector3d>& ray_directions, const o3d::geometry::VoxelGrid& voxel_grid) {
    std::vector<std::vector<Eigen::Vector3i>> free_voxels_indices_along_rays(ray_directions.size());
    std::transform(std::execution::par, ray_directions.begin(), ray_directions.end(), free_voxels_indices_along_rays.begin(),
                   [&](const auto& ray_direction) { return amanatidesWooAlgorithm(ray_origin, ray_direction, voxel_grid, 0, 1); });
    return free_voxels_indices_along_rays;
}

std::vector<std::pair<Eigen::Vector3i, Eigen::Vector3d>> extract_unique_voxels(std::vector<std::vector<Eigen::Vector3i>>& free_voxels_indices_along_rays, const o3d::geometry::VoxelGrid& voxel_grid){
    std::unordered_set<Eigen::Vector3i, matrix_hash<Eigen::Vector3i>> free_voxel_indices_set;
    for (auto voxel_indices_along_ray : free_voxels_indices_along_rays){
        for (auto voxel_idx : voxel_indices_along_ray){
            free_voxel_indices_set.insert(voxel_idx);
        }
    }
    std::vector<std::pair<Eigen::Vector3i, Eigen::Vector3d>> free_voxels_idx_loc_pairs;
    for (auto voxel_idx : free_voxel_indices_set) {
        free_voxels_idx_loc_pairs.push_back({voxel_idx, {
            voxel_grid.origin_.x() + voxel_idx.x() * voxel_grid.voxel_size_,
            voxel_grid.origin_.y() + voxel_idx.y() * voxel_grid.voxel_size_,
            voxel_grid.origin_.z() + voxel_idx.z() * voxel_grid.voxel_size_,
        }});
    }
    return free_voxels_idx_loc_pairs;
}

double distance_to_closest_pt_on_traj(const vector<Eigen::Vector3d>& traj, const Eigen::Vector3d& query_pt) {
    if (traj.empty()) return numeric_limits<double>::infinity(); // No points in the trajectory
    double min_distance = numeric_limits<double>::infinity();
    for (const auto& pt : traj) {
        const double d = (pt - query_pt).norm();
        min_distance = min(min_distance, d);
    }
    return min_distance;
}


std::pair<double, std::vector<double>> compute_information_gain(std::vector<std::pair<Eigen::Vector3i, Eigen::Vector3d>>& free_voxels_idx_loc_pairs, const std::vector<Eigen::Vector3d>& trajectory_vec, double w=1){
    std::vector<double> distances_to_closest_pt_on_traj(free_voxels_idx_loc_pairs.size());
    std::transform(std::execution::par, free_voxels_idx_loc_pairs.begin(), free_voxels_idx_loc_pairs.end(), distances_to_closest_pt_on_traj.begin(),
                   [&](const std::pair<Eigen::Vector3i, Eigen::Vector3d>& free_voxels_idx_loc_pair) { return distance_to_closest_pt_on_traj(trajectory_vec, free_voxels_idx_loc_pair.second); });
    std::vector<double> per_voxel_gains;
    double total_gain = 0.0;
    for (const auto& distance : distances_to_closest_pt_on_traj) {
        per_voxel_gains.push_back(std::exp(-w * distance));
        total_gain += per_voxel_gains.back();
    }
    return {total_gain, per_voxel_gains};
}

void visualize_info_gains(o3d::geometry::VoxelGrid& voxel_grid, std::vector<std::pair<Eigen::Vector3i, Eigen::Vector3d>>& free_voxels_idx_loc_pairs, std::vector<double>& per_voxel_gains){
    double min_info_gain = *std::min_element(per_voxel_gains.begin(), per_voxel_gains.end());
    double max_info_gain = *std::max_element(per_voxel_gains.begin(), per_voxel_gains.end());
    for(int i = 0; i < free_voxels_idx_loc_pairs.size(); i++) {
        double normalized_gain = (per_voxel_gains[i] - min_info_gain) / (max_info_gain - min_info_gain);
        double hue = MapValue(normalized_gain, 0.0, 1.0, 0.66, 0); // Map to hue (blue to red)
        voxel_grid.AddVoxel(o3d::geometry::Voxel(free_voxels_idx_loc_pairs[i].first, HsvToRgb(hue, 0.8, 1.0)));
    }
}

class CameraConfig {
    public:
        Eigen::Vector3d m_camera_pos, m_camera_rpy;
        double m_vfov_deg;
        double m_hfov_deg;
        int m_image_width;
        int m_image_height;
        CameraConfig(Eigen::Vector3d camera_pos, Eigen::Vector3d camera_rpy) {
            m_camera_pos = camera_pos;
            m_camera_rpy = camera_rpy;
            m_vfov_deg = 60;
            m_hfov_deg = 90;
            m_image_width = 64;
            m_image_height = 48;
        }
};



//double compute_perception_aware_cost_multi_cameras(const o3d::geometry::VoxelGrid& voxel_grid, const std::vector<Eigen::Vector3d>& trajectory_vec, std::vector<CameraConfig>& camera_configs, bool visualize_gains=false){
//    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> ray_origin_directions;
//
//    for (auto camera_config : camera_configs){
//        generate_rays(camera_config.m_camera_pos, camera_config.m_camera_rpy, camera_config.m_vfov_deg,
//                      camera_config.m_hfov_deg, camera_config.m_image_width, camera_config.m_image_height, ray_origin_directions);
//    }
//
//    std::vector<std::vector<Eigen::Vector3i>> free_voxels_indices_along_rays(ray_origin_directions.size());
//    std::transform(std::execution::par, ray_origin_directions.begin(), ray_origin_directions.end(), free_voxels_indices_along_rays.begin(),
//                   [&](const auto& ray_origin_directions) { return amanatidesWooAlgorithm(ray_origin_directions.first, ray_origin_directions.second, voxel_grid, 0, 1); });
//
////    auto free_voxels_indices_along_rays = ray_cast(camera_config.m_camera_pos, ray_directions, voxel_grid);
////    auto free_voxels_idx_loc_pairs = extract_unique_voxels(free_voxels_indices_along_rays, voxel_grid);
////    auto info_gain_pair = compute_information_gain(free_voxels_idx_loc_pairs, trajectory_vec, 1);
////    if (visualize_gains){
////        visualize_info_gains(voxel_grid, free_voxels_idx_loc_pairs, info_gain_pair.second);
////    }
////    std::cout << "Exited compute_perception_aware_cost" << std::endl;
////    return info_gain_pair.first;
//    return 0;
//}


double compute_perception_aware_cost(o3d::geometry::VoxelGrid& voxel_grid, const std::vector<Eigen::Vector3d>& trajectory_vec, CameraConfig& camera_config, bool visualize_gains=false){
    std::vector<Eigen::Vector3d> ray_directions;
    generate_rays(camera_config.m_camera_pos, camera_config.m_camera_rpy, camera_config.m_vfov_deg,
                  camera_config.m_hfov_deg, camera_config.m_image_width, camera_config.m_image_height, ray_directions);
    auto free_voxels_indices_along_rays = ray_cast(camera_config.m_camera_pos, ray_directions, voxel_grid);
    auto free_voxels_idx_loc_pairs = extract_unique_voxels(free_voxels_indices_along_rays, voxel_grid);
    auto info_gain_pair = compute_information_gain(free_voxels_idx_loc_pairs, trajectory_vec, 1);
    if (visualize_gains){
        visualize_info_gains(voxel_grid, free_voxels_idx_loc_pairs, info_gain_pair.second);
    }
    return info_gain_pair.first;
}

Eigen::Vector3d get_best_yaw(o3d::geometry::VoxelGrid& voxel_grid, std::vector<Eigen::Vector3d>& trajectory_vec, CameraConfig& init_camera_config, double sample_min_deg=-30, double sample_max_deg=30, double sample_dist=10){
//    auto start_time_traj = std::chrono::high_resolution_clock::now();
    std::vector<CameraConfig> camera_configs;
    for (double ang = sample_min_deg; ang <= sample_max_deg; ang += sample_dist){
        camera_configs.push_back(CameraConfig(init_camera_config.m_camera_pos, init_camera_config.m_camera_rpy + Eigen::Vector3d(0, 0, M_PI / 180 * ang)));
    }
//    camera_configs.push_back(CameraConfig(init_camera_config.m_camera_pos, init_camera_config.m_camera_rpy));

    std::vector<double> info_gains_for_yaws(camera_configs.size());
    std::transform(std::execution::par_unseq, camera_configs.begin(), camera_configs.end(), info_gains_for_yaws.begin(),
                   [&](auto& camera_config) { return compute_perception_aware_cost(voxel_grid, trajectory_vec, camera_config); });

//    compute_perception_aware_cost_multi_cameras(voxel_grid, trajectory_vec, camera_configs);
//    for (int i = 0; i < camera_configs.size(); i++){
//        info_gains_for_yaws[i] = compute_perception_aware_cost(voxel_grid, trajectory_vec, camera_configs[i]);
//    }

    size_t max_index = std::distance(info_gains_for_yaws.begin(), std::max_element(info_gains_for_yaws.begin(), info_gains_for_yaws.end()));
//    for (int i = 0; i < camera_configs.size(); i++){
//        std::cout << "Yaw: " << camera_configs[i].m_camera_rpy[2] << " Gain: " << info_gains_for_yaws[i];
//        if (i == max_index){
//            std::cout << " ---------> BEST";
//        }
//        std::cout << std::endl;
//    }
//    std::cout << "BEST: " << camera_configs[max_index].m_camera_rpy << std::endl;

//    auto end_time_traj = std::chrono::high_resolution_clock::now();
//    auto duration_traj = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_traj - start_time_traj);
//    std::cout << "Processing get_best_yaw took " << duration_traj.count() / 1000.0 << "s" << std::endl;
    return camera_configs[max_index].m_camera_rpy;
}

Eigen::Matrix4d compute_transform(const vector<Eigen::Vector3d>& world_traj, const vector<Eigen::Vector3d>& body_traj) {
    // Convert lists of trajectories to Eigen matrices
    Eigen::MatrixXd world_traj_mat(3, world_traj.size());
    Eigen::MatrixXd body_traj_mat(3, body_traj.size());
    for (int i = 0; i < world_traj.size(); ++i) {
        world_traj_mat.col(i) = world_traj[i];
        body_traj_mat.col(i) = body_traj[i];
    }

    // Center the trajectories around the origin
    Eigen::Vector3d world_mean = world_traj_mat.rowwise().mean();
    Eigen::Vector3d body_mean = body_traj_mat.rowwise().mean();
    Eigen::MatrixXd world_centered = world_traj_mat.colwise() - world_mean;
    Eigen::MatrixXd body_centered = body_traj_mat.colwise() - body_mean;

    // Estimate the rotation matrix using Singular Value Decomposition (SVD)
    Eigen::Matrix3d H = world_centered * body_centered.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d R = V * U.transpose();

    // Ensure proper rotation matrix
    if (R.determinant() < 0) {
        V.col(2) *= -1;
        R = V * U.transpose();
    }

    // Translation vector is simply the negative of the first point in the body frame trajectory
    Eigen::Vector3d t = body_mean - R * world_mean;

    // Construct the transformation matrix
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block(0, 0, 3, 3) = R;
    T.block(0, 3, 3, 1) = t;

    return T;
}



