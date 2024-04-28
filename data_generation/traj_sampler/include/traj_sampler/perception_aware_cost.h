#include <random>

#include <open3d/Open3D.h>


#include <iostream>
#include <Eigen/Core>
#include <execution>
#include "example-utils.hpp"
#include "tinyply.h"
#include <cmath>

using namespace std;
namespace o3d = open3d;


std::vector<Eigen::Vector3d> parse_pointcloud(const std::string pointcloud_filename) {
//o3d::geometry::PointCloud parse_pointcloud(const std::string pointcloud_filename) {
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
            std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    try {
      colors = file.request_properties_from_element(
          "vertex", {"red", "green", "blue", "alpha"});
    } catch (const std::exception &e) {
            std::cerr << "tinyply exception: " << e.what() << std::endl;
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
            std::cerr << "tinyply exception: " << e.what() << std::endl;
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

//    std::cout << "numVerticesBytes: " << vertices->buffer.size_bytes() << " numNormalsBytes" << normals->buffer.size_bytes()  << " numColorsBytes" << colors->buffer.size_bytes() << std::endl;

//    std::cout << "numVerticesBytes: " << vertices->buffer.size_bytes()  << std::endl;
//    std::cout << "numColorsBytes: " << colors->buffer.size_bytes()  << std::endl;
//    std::cout << "numNormalsBytes: " << normals->buffer.size_bytes()  << std::endl;

    const size_t numVerticesBytes = vertices->buffer.size_bytes();
    std::vector<float3> verts(vertices->count);
    std::memcpy(verts.data(), vertices->buffer.get(), numVerticesBytes);

//    const size_t numNormalsBytes = normals->buffer.size_bytes();
//    std::vector<float3> normals_float3(normals->count);
//    std::memcpy(normals_float3.data(), normals->buffer.get(), numNormalsBytes);
//
//    const size_t numColorsBytes = colors->buffer.size_bytes();
//    std::vector<float3> colors_float3(colors->count);
//    std::memcpy(colors_float3.data(), colors->buffer.get(), numColorsBytes);

//    std::cout << "numVerticesBytes: " << numVerticesBytes << " numNormalsBytes" << numNormalsBytes  << " numColorsBytes" << numColorsBytes << std::endl;
    int idx = 0;
    for (auto point_tinyply : verts) {
      if (idx == 0) {
        points_ = Eigen::Vector3d(static_cast<double>(point_tinyply.x),
                                  static_cast<double>(point_tinyply.y),
                                  static_cast<double>(point_tinyply.z));
      } else {
        points_.conservativeResize(points_.rows(), points_.cols() + 1);
        points_.col(points_.cols() - 1) =
            Eigen::Vector3d(static_cast<double>(point_tinyply.x),
                            static_cast<double>(point_tinyply.y),
                            static_cast<double>(point_tinyply.z));
      }
      if (point_tinyply.z < 3.5){
          points_vec.push_back(
              Eigen::Vector3d(static_cast<double>(point_tinyply.x),
                              static_cast<double>(point_tinyply.y),
                              static_cast<double>(point_tinyply.z)));
      }
      idx += 1;
    }


//    int idx = 0;
//    for (int i = 0; i < verts.size(); i++) {
//      auto point_tinyply_verts = verts[i];
//      auto point_tinyply_colors = colors_float3[i];
//      auto point_tinyply_normals = normals_float3[i];
//
//      if (point_tinyply_verts.z < 3.5){
//          points_vec.push_back(
//              Eigen::Vector3d(static_cast<double>(point_tinyply_verts.x),
//                              static_cast<double>(point_tinyply_verts.y),
//                              static_cast<double>(point_tinyply_verts.z)));
//          colors_vec.push_back(
//              Eigen::Vector3d(static_cast<double>(point_tinyply_colors.x),
//                              static_cast<double>(point_tinyply_colors.y),
//                              static_cast<double>(point_tinyply_colors.z)));
//          normals_vec.push_back(
//              Eigen::Vector3d(static_cast<double>(point_tinyply_normals.x),
//                              static_cast<double>(point_tinyply_normals.y),
//                              static_cast<double>(point_tinyply_normals.z)));
//      }
//      idx += 1;
//    }
  } catch (const std::exception &e) {
    std::cerr << "Caught tinyply exception: " << e.what() << std::endl;
  }

  std::cout << "Completed pointcloud parsing!" << std::endl;

//  o3d::geometry::PointCloud point_cloud(points_vec);
//  point_cloud.colors_ = colors_vec;
//  point_cloud.normals_ = normals_vec;
//  return point_cloud;
  return points_vec;
}

// Function to parse CSV file and extract trajectory (x, y, z) data
std::vector<Eigen::Vector3d> parse_trajectory_from_csv(const std::string& csv_filename) {
    std::vector<Eigen::Vector3d> trajectory;

    // Open CSV file for reading
    std::ifstream file(csv_filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << csv_filename << std::endl;
        return trajectory;  // Return empty trajectory if file cannot be opened
    }
    std::string line;
    // Read CSV file line by line
    std::getline(file, line);
    while (std::getline(file, line)) {

        std::istringstream iss(line);
        std::string token;

        // Extract trajectory data from each line (assuming CSV format)
        std::vector<double> values;
        while (std::getline(iss, token, ',')) {
            values.push_back(std::stod(token));
        }
//        for (int i = 0; i < 10; i++){
//            Eigen::Vector3d position(values[17 * i], values[17 * i + 1], values[17 * i + 2]);
//            trajectory.push_back(position);
//        }

        if (values.size() >= 4) {  // Ensure line contains at least pos_x, pos_y, pos_z
            Eigen::Vector3d position(values[1], values[2], values[3]);
            trajectory.push_back(position);
        }
    }

    std::cout << "Trajectory loaded from: " << csv_filename << std::endl;
    std::cout << "Number of points in trajectory: " << trajectory.size() << std::endl;

    return trajectory;
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

std::vector<Eigen::Vector3d> raycast(const Eigen::Vector3d& origin, const Eigen::Vector3d& dir, double maxRange, o3d::geometry::VoxelGrid& voxelGrid) {
    std::vector<Eigen::Vector3d> unoccupiedVoxels;

    // Get the inverse direction for optimization
    Eigen::Vector3d invDir(1.0 / dir.x(), 1.0 / dir.y(), 1.0 / dir.z());

    // Calculate the step direction based on the sign of the direction
    Eigen::Vector3i step(
        (dir.x() < 0) ? -1 : 1,
        (dir.y() < 0) ? -1 : 1,
        (dir.z() < 0) ? -1 : 1
    );

    // Calculate initial voxel position
    Eigen::Vector3d tMax(
        (step.x() > 0) ? (voxelGrid.origin_.x() + (voxelGrid.voxel_size_ * (voxelGrid.GetMaxBound().x() - 1) - origin.x()) * invDir.x()) :
                        (voxelGrid.origin_.x() - origin.x()) * invDir.x(),
        (step.y() > 0) ? (voxelGrid.origin_.y() + (voxelGrid.voxel_size_ * (voxelGrid.GetMaxBound().y() - 1) - origin.y()) * invDir.y()) :
                        (voxelGrid.origin_.y() - origin.y()) * invDir.y(),
        (step.z() > 0) ? (voxelGrid.origin_.z() + (voxelGrid.voxel_size_ * (voxelGrid.GetMaxBound().z() - 1) - origin.z()) * invDir.z()) :
                        (voxelGrid.origin_.z() - origin.z()) * invDir.z()
    );

    // Calculate tDelta values
    Eigen::Vector3d tDelta(
        voxelGrid.voxel_size_ * invDir.x(),
        voxelGrid.voxel_size_ * invDir.y(),
        voxelGrid.voxel_size_ * invDir.z()
    );

    // Start raycasting
    double t = 0.0;
    while (t < maxRange) {
        // Calculate voxel index
        Eigen::Vector3i voxelIndex(
            static_cast<int>((origin.x() + t * dir.x() - voxelGrid.origin_.x()) / voxelGrid.voxel_size_),
            static_cast<int>((origin.y() + t * dir.y() - voxelGrid.origin_.y()) / voxelGrid.voxel_size_),
            static_cast<int>((origin.z() + t * dir.z() - voxelGrid.origin_.z()) / voxelGrid.voxel_size_)
        );

        Eigen::Vector3d voxelQuery(
            origin.x() + t * dir.x(),
            origin.y() + t * dir.y(),
            origin.z() + t * dir.z()
        );

        // Check if voxel is within bounds
//        if (voxelIndex.x() >= 0 && voxelIndex.y() >= 0 && voxelIndex.z() >= 0 &&
//            voxelIndex.x() < voxelGrid.GetMaxBound().x() && voxelIndex.y() < voxelGrid.GetMaxBound().y() && voxelIndex.z() < voxelGrid.GetMaxBound().z()) {
//            // Check if voxel is unoccupied
//            if (!voxelGrid.GetVoxel(voxelIndex)) {
//                unoccupiedVoxels.push_back(voxelIndex);
//            }
//        } else {
//            // Ray has reached outside the voxel grid
//            break;
//        }

        if (voxelIndex.x() >= 0 && voxelIndex.y() >= 0 && voxelIndex.z() >= 0 &&
            voxelIndex.x() < voxelGrid.GetMaxBound().x() && voxelIndex.y() < voxelGrid.GetMaxBound().y() && voxelIndex.z() < voxelGrid.GetMaxBound().z()) {
            // Check if voxel is unoccupied
            if (!voxelGrid.CheckIfIncluded({voxelQuery})[0]) {
                unoccupiedVoxels.push_back(voxelQuery);
            }
        } else {
            // Ray has reached outside the voxel grid
            break;
        }

        // Move along the ray to the next voxel
        int axis;
        if (tMax.x() < tMax.y()) {
            if (tMax.x() < tMax.z()) {
                axis = 0;
            } else {
                axis = 2;
            }
        } else {
            if (tMax.y() < tMax.z()) {
                axis = 1;
            } else {
                axis = 2;
            }
        }
        t = tMax(axis);
        tMax(axis) += tDelta(axis);
    }

    return unoccupiedVoxels;
}


bool rayBoxIntersection(const Eigen::Vector3d& ray_origin, const Eigen::Vector3d& ray_direction, o3d::geometry::VoxelGrid& grid, double& tMin, double& tMax,
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

void generate_rays(const Eigen::Vector3d& camera_pos, const Eigen::Matrix3d& camera_rot,
                   double vfov_deg, double hfov_deg, int image_width, int image_height, std::vector<Eigen::Vector3d>& ray_directions) {
    // Convert field of view from degrees to radians
    double vfov_rad = vfov_deg * M_PI / 180.0;
    double hfov_rad = hfov_deg * M_PI / 180.0;

    // Compute aspect ratio
    double aspect_ratio = static_cast<double>(image_width) / static_cast<double>(image_height);

    // Compute camera coordinate system axes
//    Eigen::Vector3d camera_forward = camera_rot.col(2).normalized();
//    Eigen::Vector3d camera_right = camera_rot.col(0).normalized();
//    Eigen::Vector3d camera_up = camera_rot.col(1).normalized();
    Eigen::Vector3d camera_forward = camera_rot.col(0).normalized();
    Eigen::Vector3d camera_right = camera_rot.col(1).normalized();
    Eigen::Vector3d camera_up = camera_rot.col(2).normalized();

    // Initialize ray origins and directions vectors
//    ray_origins.resize(image_width * image_height);
    ray_directions.resize(image_width * image_height);

    // Compute ray origins and directions for each pixel
    for (int y = 0; y < image_height; ++y) {
        for (int x = 0; x < image_width; ++x) {
            // Compute normalized device coordinates (-1 to 1)
            double ndc_x = (2.0 * x / (image_width - 1.0)) - 1.0;
            double ndc_y = 1.0 - (2.0 * y / (image_height - 1.0));

            // Compute ray direction in camera space
            Eigen::Vector3d ray_direction_camera;
            ray_direction_camera = camera_forward +
                                   ndc_x * tan(hfov_rad / 2.0) * aspect_ratio * camera_right +
                                   ndc_y * tan(vfov_rad / 2.0) * camera_up;
            ray_direction_camera.normalize();

            // Transform ray direction to world space
            Eigen::Vector3d ray_direction_world = camera_rot * ray_direction_camera;

            // Set ray origin as camera position
//            Eigen::Vector3d ray_origin = camera_pos;

            // Save ray origin and direction
            int index = y * image_width + x;
//            ray_origins[index] = ray_origin;
            ray_directions[index] = ray_direction_world.normalized();
        }
    }
}

std::vector<Eigen::Vector3i> amanatidesWooAlgorithm(const Eigen::Vector3d& ray_origin, const Eigen::Vector3d& ray_direction, o3d::geometry::VoxelGrid& grid, double t0, double t1) noexcept {
    double tMin;
    double tMax;
    const bool ray_intersects_grid = rayBoxIntersection(ray_origin, ray_direction, grid, tMin, tMax, t0, t1);
    if (!ray_intersects_grid) return {};

    tMin = std::max(tMin, t0);
    tMax = std::max(tMax, t1);
    const Eigen::Vector3d ray_start = ray_origin + ray_direction * tMin;
    const Eigen::Vector3d ray_end = ray_origin + ray_direction * tMax;

    Eigen::Vector3i current_index = grid.GetVoxel(ray_origin);
//    std::cout << "*******************************************************" << std::endl;
//    std::cout << "ray origin: " << ray_origin.x() << ", " << ray_origin.y() << ", " << ray_origin.z() << std::endl;
//    std::cout << "center: " << current_index.x() << ", " << current_index.y() << ", " << current_index.z() << std::endl;
//    auto idx_cent = grid.GetVoxelCenterCoordinate(current_index);

    Eigen::Vector3d idx_cent;
    idx_cent.x() = grid.origin_.x() + current_index.x() * grid.voxel_size_;
    idx_cent.y() = grid.origin_.y() + current_index.y() * grid.voxel_size_;
    idx_cent.z() = grid.origin_.z() + current_index.z() * grid.voxel_size_;

//    std::cout << "idx center: " << idx_cent.x() << ", " << idx_cent.y() << ", " << idx_cent.z() << std::endl;
//    std::cout << "voxel count: " << grid.voxels_.count(current_index) << std::endl;
//    std::cout << "*******************************************************" << std::endl;
//    size_t current_X_index = std::max(1.0, std::ceil(ray_start.x() - grid.GetMinBound().x() / grid.voxel_size_));
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

//    size_t current_Y_index = std::max(1.0, std::ceil(ray_start.y() - grid.GetMinBound().y() / grid.voxel_size_));
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

//    size_t current_Z_index = std::max(1.0, std::ceil(ray_start.z() - grid.GetMinBound().z() / grid.voxel_size_));
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
//    while (current_X_index != end_X_index || current_Y_index != end_Y_index || current_Z_index != end_Z_index) {
    int num_voxels_expanded = 0;
    int max_dist = 0;
    while (grid.voxels_.count({current_X_index, current_Y_index, current_Z_index}) == 0 && num_voxels_expanded < 100 && (tMaxX*tMaxX + tMaxY * tMaxY + tMaxZ * tMaxZ) < 10*10) {
        num_voxels_expanded++;
//        std::cout << current_X_index << ", " << current_Y_index << ", " << current_Z_index << std::endl;
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
//    std::cout << "********************* DONE" << std::endl;
    return unoccupiedVoxelsIndices;
}
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

double distance_to_closest_pt_on_traj(const vector<Eigen::Vector3d>& traj, const Eigen::Vector3d& query_pt) {
    if (traj.empty()) return numeric_limits<double>::infinity(); // No points in the trajectory

    double min_distance = numeric_limits<double>::infinity();
    for (const auto& pt : traj) {
        const double d = (pt - query_pt).norm();
        min_distance = min(min_distance, d);
    }
    return std::exp(-0.1 * min_distance);
//    return min_distance;
}

int main() {
    // Specify the path to the .ply file
    const std::string ply_file = "/root/agile_autonomy_ws/catkin_aa/src/agile_autonomy/data_generation/data/rollout_24-04-17_16-23-03/pointcloud-unity.ply";
//    const std::string csv_file = "/root/agile_autonomy_ws/catkin_aa/src/agile_autonomy/data_generation/data/rollout_24-04-17_16-23-03/trajectories/trajectory_mpc_opt_wf_00000000.csv";
    const std::string csv_file = "/root/agile_autonomy_ws/catkin_aa/src/agile_autonomy/data_generation/data/rollout_24-04-17_16-23-03/reference_trajectory.csv";
//    std::vector<Eigen::Vector3d> trajectory_vec = parse_trajectory_from_csv(csv_file);

    std::vector<Eigen::Vector3d> trajectory_vec = {
        {-13, 20.5, 1.89},
        {-12.5, 20.5, 1.89},
        {-12, 20.5, 1.89},
        {-11.5, 20.5, 1.89},
        {-11, 20.5, 1.89},
        {-10.5, 20.5, 1.89},
        {-10, 20.5, 1.89},
        {-9.5, 20.5, 1.89},
        {-9, 20.5, 1.89},
        {-8.5, 20.5, 1.89},
        {-8, 20.55, 1.89},
        {-7.5, 20.75, 1.89},
//        {-7.0, 21.1, 1.89},
//        {-6.5, 21.5, 1.89},
//        {-6.25, 22, 1.89},
//        {-6, 22.5, 1.89},
//        {-5.8, 23, 1.89},
//        {-5.75, 23.5, 1.89},
//        {-5.6, 24, 1.89},
//        {-5.5, 24.5, 1.89},

        {-7.0, 21.1, 1.89},
        {-7.05, 21.5, 1.89},
        {-7.025, 22, 1.89},
        {-7.0, 22.5, 1.89},
        {-7.08, 23, 1.89},
        {-7.075, 23.5, 1.89},
        {-7.06, 24, 1.89},
        {-7.05, 24.5, 1.89},
    };
    std::vector<Eigen::Vector3d> points_vec = parse_pointcloud(ply_file);
//
    o3d::geometry::PointCloud point_cloud(points_vec);

//    // Calculate number of points
//    size_t num_points = points_vec.size();
//
//    // Create a flat array to hold the point coordinates (flattened XYZ)
//    vector<double> points_data(num_points * 3);
//    for (size_t i = 0; i < num_points; ++i) {
//        points_data ;  // X coordinate
//        points_data ;  // Y coordinate
//        points_data ;  // Z coordinate
//    }
//
//    // Convert points_data to a core::Tensor
//    o3d::core::Tensor points_tensor(points_data, {static_cast<int64_t>(num_points), 3}, o3d::core::Dtype::Float64);
//
//    o3d::t::geometry::PointCloud point_cloud(points_tensor);
//    o3d::geometry::PointCloud point_cloud = parse_pointcloud(ply_file);
//     point_cloud(points_vec);

    auto& points = point_cloud.points_;
    std::vector<double> z_values(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        z_values[i] = points[i][2]; // Assuming z-coordinate is the third component (0-indexed)
    }
    double min_z = *std::min_element(z_values.begin(), z_values.end());
    double max_z = *std::max_element(z_values.begin(), z_values.end());
    std::vector<Eigen::Vector3d> colors(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
//        double normalized_z = (z_values[i] - min_z) / (max_z - min_z);
////        double hue = MapValue(normalized_z, 0.0, 1.0, 0.66, 0.0); // Map to hue (blue to red)
//double hue = MapValue(normalized_z, 0.0, 1.0, 0.0, 0.0);
//        colors[i] = HsvToRgb(hue, 0.8, 1.0);
//        colors[i] = HsvToRgb(hue, 0.3, 0.3);
        double normalized_z = (z_values[i] - min_z) / (max_z - min_z);
        double value = MapValue(normalized_z, 0.0, 1.0, 0.8, 0.2); // Saturation decreases with z
        double saturation = 0.05; // Keep value constant at 1.0 for full intensity
        colors[i] = HsvToRgb(0.7, saturation, value); // Constant hue of 0.0 for red
//        double normalized_z = (z_values[i] - min_z) / (max_z - min_z);
//        double value = 1.0 - normalized_z * 0.9; // Value decreases with z, but keeping a tinge of black
//        double saturation = 0.2 + normalized_z * 0.8; // Saturation increases with z, but capped at 1.0
//        colors[i] = HsvToRgb(0.0, saturation, value); // Constant hue of 0.0 for black
//double normalized_z = (z_values[i] - min_z) / (max_z - min_z);
//        double value = 0.3 + normalized_z * 0.7; // Value increases with z, starting from 0.3
//        double saturation = 0.2 + normalized_z * 0.8; // Saturation increases with z, but capped at 1.0
//        colors[i] = HsvToRgb(0.0, saturation, value); // Constant hue of 0.0 for black

    }
    point_cloud.colors_ = colors;

    o3d::geometry::PointCloud traj(trajectory_vec);

    traj.PaintUniformColor({1, 0, 0});

//    point_cloud.PaintUniformColor({0, 1, 0});
//    o3d::visualization::DrawGeometries({std::make_shared<o3d::geometry::PointCloud>(traj), std::make_shared<o3d::geometry::PointCloud>(point_cloud)}, "Point Cloud Viewer");
//    std::cout << "pcl has colors: " << point_cloud.HasColors() << std::endl;
    double voxel_size = 0.2; // Adjust this value based on your desired voxel grid resolution
    auto voxel_grid = o3d::geometry::VoxelGrid::CreateFromPointCloud(point_cloud, voxel_size);

//    Eigen::Vector3d origin(0.0, 0.0, 0.0);
//    Eigen::Vector3d origin = trajectory_vec[200];
//    Eigen::Vector3d direction(1.0, 1.0, 1.0);
//    double maxRange = 100.0;
//
//    auto unoccupiedVoxelsIndices = amanatidesWooAlgorithm(origin, direction, *voxel_grid, 0, 1);

//    std::vector<Eigen::Vector3d> unoccupiedVoxels;
//    for (auto idx : unoccupiedVoxelsIndices){
////        Eigen::Vector3d idx_cent;
////    idx_cent.x() = grid.origin_.x() + current_index.x() * grid.voxel_size_;
////    idx_cent.y() = grid.origin_.y() + current_index.y() * grid.voxel_size_;
////    idx_cent.z() = grid.origin_.z() + current_index.z() * grid.voxel_size_;
//        voxel_grid->AddVoxel(o3d::geometry::Voxel(idx, {1, 0, 1}));
//        unoccupiedVoxels.push_back({
//            voxel_grid->origin_.x() + idx.x() * voxel_grid->voxel_size_,
//            voxel_grid->origin_.y() + idx.y() * voxel_grid->voxel_size_,
//            voxel_grid->origin_.z() + idx.z() * voxel_grid->voxel_size_
//        });
//    }
////    std::vector<Eigen::Vector3d> unoccupiedVoxels = raycast(origin, direction, maxRange, *voxel_grid);
//    std::cout << "Num of unoccupied voxels: " << unoccupiedVoxels.size() << std::endl;
//    std::cout << "Origin Occupied: " << voxel_grid->CheckIfIncluded({origin})[0] << std::endl;
////    std::cout << "Origin center: " <<  grid.CheckIfIncluded({grid.GetVoxelCenterCoordinate({current_X_index, current_Y_index, current_Z_index})})[0] << endl;
//    o3d::geometry::PointCloud ray(unoccupiedVoxels);
//    ray.PaintUniformColor({1, 0, 1});

//    Eigen::Vector3d camera_pos = trajectory_vec[75]; // Camera position
    Eigen::Vector3d camera_pos = trajectory_vec[0]; // Camera position
//    std::cout << "trajectory_vec[75]: " << trajectory_vec[75] << " trajectory_vec[200]: " << trajectory_vec[200] << std::endl;
//    Eigen::Matrix3d camera_rot; // Camera rotation matrix
//    camera_rot << 0, 0, 1,
//                  0, 1, 0,
//                  1, 0, 0; // Identity matrix (no rotation)
//    Eigen::Matrix3d camera_rot;
//    camera_rot = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());

    double roll = 0;     // 45 degrees
    double pitch = 0;    // 30 degrees
    double yaw = M_PI / 180 * 30;      // 60 degrees

//     Create individual rotation matrices for each RPY angle
    Eigen::Matrix3d rot_roll = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d rot_pitch = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d rot_yaw = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // Combine the individual rotation matrices into a single rotation matrix
    Eigen::Matrix3d camera_rot = rot_yaw * rot_pitch * rot_roll;

    double vfov_deg = 60.0; // Vertical field of view in degrees
    double hfov_deg = 90.0; // Horizontal field of view in degrees
    int image_width = 64; // Image width
    int image_height = 48;

    std::vector<Eigen::Vector3d> ray_directions;

    generate_rays(camera_pos, camera_rot, vfov_deg, hfov_deg, image_width, image_height, ray_directions);
//    std::cout << "Number of ray directions: " << ray_directions.size() << std::endl;

    std::vector<std::vector<Eigen::Vector3i>> results(ray_directions.size());
    std::transform(std::execution::par, ray_directions.begin(), ray_directions.end(), results.begin(),
                   [&](const auto& ray_direction) { return amanatidesWooAlgorithm(camera_pos, ray_direction, *voxel_grid, 0, 1); });

    std::unordered_set<Eigen::Vector3i, matrix_hash<Eigen::Vector3i>> unoccupiedVoxelsIndicesSet;

    for (auto el_vec : results){
        for (auto el : el_vec){
            unoccupiedVoxelsIndicesSet.insert(el);
        }
    }
    std::vector<Eigen::Vector3d> unoccupiedVoxels;
    for (auto idx : unoccupiedVoxelsIndicesSet) {
        unoccupiedVoxels.push_back({
            voxel_grid->origin_.x() + idx.x() * voxel_grid->voxel_size_,
            voxel_grid->origin_.y() + idx.y() * voxel_grid->voxel_size_,
            voxel_grid->origin_.z() + idx.z() * voxel_grid->voxel_size_,
        });

    }

    std::vector<double> dists(unoccupiedVoxelsIndicesSet.size());
    std::transform(std::execution::par, unoccupiedVoxels.begin(), unoccupiedVoxels.end(), dists.begin(),
                   [&](const Eigen::Vector3d& voxel_pos) { return distance_to_closest_pt_on_traj(trajectory_vec, voxel_pos); });

    double total_gain = 0.0;
    for (const auto& elem : dists) {
        total_gain += elem;
    }
    std::cout << "=========== TOTAL GAIN = " << total_gain << " =========================" << std::endl;

int i = 0;
//    for (auto idx : unoccupiedVoxelsIndicesSet) {
//        int dy2 = (voxel_grid->origin_.y() + idx.y() * voxel_grid->voxel_size_ - 20) * (voxel_grid->origin_.y() + idx.y() * voxel_grid->voxel_size_ - 20);
//        int dz2 = (voxel_grid->origin_.z() + idx.z() * voxel_grid->voxel_size_ - 1.89) * (voxel_grid->origin_.z() + idx.z() * voxel_grid->voxel_size_ - 1.89);
//        dists[i++] = dy2 + dz2; // Assuming z-coordinate is the third component (0-indexed)
//    }
    double min_dists = *std::min_element(dists.begin(), dists.end());
    double max_dists = *std::max_element(dists.begin(), dists.end());
//    std::vector<Eigen::Vector3d> colors(points.size());
//    for (size_t i = 0; i < points.size(); ++i) {
//        double normalized_z = (z_values[i] - min_z) / (max_z - min_z);
//        double hue = MapValue(normalized_z, 0.0, 1.0, 0.66, 0.0); // Map to hue (blue to red)
////        double hue = MapValue(normalized_z, 0.0, 1.0, 0.0, 0.0);
//        colors[i] = HsvToRgb(hue, 0.8, 1.0);
//    }
i = 0;
    for (auto idx : unoccupiedVoxelsIndicesSet){
//        Eigen::Vector3d idx_cent;
//    idx_cent.x() = grid.origin_.x() + current_index.x() * grid.voxel_size_;
//    idx_cent.y() = grid.origin_.y() + current_index.y() * grid.voxel_size_;
//    idx_cent.z() = grid.origin_.z() + current_index.z() * grid.voxel_size_;
        double normalized_dists = (dists[i++] - min_dists) / (max_dists - min_dists);
        double hue = MapValue(normalized_dists, 0.0, 1.0, 0.66, 0); // Map to hue (blue to red)
//        double hue = MapValue(normalized_z, 0.0, 1.0, 0.0, 0.0);
//        colors[i] =

        voxel_grid->AddVoxel(o3d::geometry::Voxel(idx, HsvToRgb(hue, 0.8, 1.0)));
//        unoccupiedVoxels.push_back({
//            voxel_grid->origin_.x() + idx.x() * voxel_grid->voxel_size_,
//            voxel_grid->origin_.y() + idx.y() * voxel_grid->voxel_size_,
//            voxel_grid->origin_.z() + idx.z() * voxel_grid->voxel_size_
//        });
    }
//    std::vector<Eigen::Vector3d> unoccupiedVoxels = raycast(origin, direction, maxRange, *voxel_grid);
//    std::cout << "Num of unoccupied voxels: " << unoccupiedVoxels.size() << std::endl;
//    std::cout << "Origin Occupied: " << voxel_grid->CheckIfIncluded({origin})[0] << std::endl;
//    std::cout << "Origin center: " <<  grid.CheckIfIncluded({grid.GetVoxelCenterCoordinate({current_X_index, current_Y_index, current_Z_index})})[0] << endl;
//    o3d::geometry::PointCloud ray(unoccupiedVoxels);
//    ray.PaintUniformColor({1, 0, 1});

//    std::unordered_set<Eigen::Vector3i, matrix_hash<Eigen::Vector3i>> spatial_hashing;
//    std::unordered_set<Eigen::Vector3i, matrix_hash<Eigen::Vector3i>> spatial_hashing;
//    spatial_hashing.insert(Eigen::Vector3i(1, 0, 0));
//    spatial_hashing.insert(Eigen::Vector3i(2, 0, 0));
//    spatial_hashing.insert(Eigen::Vector3i(1, 0, 0));
//    for (auto el : spatial_hashing){
//        std::cout << el.x() << ", " << el.y() << ", " << el.z() << std::endl;
//    }

//    auto triangle_mesh = o3d::geometry::TriangleMesh::CreateFromPointCloudAlphaShape(point_cloud, 0.75);

//    double voxel_size = 3.0 / 512.0;  // Voxel size in meters
//    int block_resolution = 16;        // Voxel block resolution
//    int block_count = 50000;          // Maximum number of voxel blocks
//
//    // Define attribute names and data types
//    vector<std::string> attr_names{"tsdf", "weight"};
//    vector<o3d::core::Dtype> attr_dtypes{o3d::core::Dtype::Float32, o3d::core::Dtype::Float32};
//    vector<o3d::core::SizeVector> attr_channels{{1}, {1}};
//
//    // Create VoxelBlockGrid
//    o3d::t::geometry::VoxelBlockGrid voxel_block_grid(attr_names, attr_dtypes, attr_channels,
//                                                 voxel_size, block_resolution, block_count);
//
//    auto block_coords = voxel_block_grid.GetUniqueBlockCoordinates(point_cloud);
//
////    o3d::t::geometry::TriangleMesh t_mesh =  voxel_block_grid.ExtractTriangleMesh();
//    o3d::t::geometry::PointCloud t_pc =  voxel_block_grid.ExtractPointCloud();
//    std::cout << "has colors: " << point_cloud.HasColors() << " has normals: " << point_cloud.HasNormals() << std::endl;
//    o3d::visualization::DrawGeometries({std::make_shared<o3d::geometry::PointCloud>(traj), std::make_shared<o3d::geometry::PointCloud>(point_cloud)}, "Point Cloud Viewer");
//    o3d::visualization::DrawGeometries({std::make_shared<o3d::geometry::PointCloud>(ray), voxel_grid}, "Point Cloud Viewer");
//o3d::visualization::DrawGeometriesWithAnimationCallback({std::make_shared<o3d::geometry::PointCloud>(traj), voxel_grid}, RotateView);

o3d::visualization::DrawGeometries({std::make_shared<o3d::geometry::PointCloud>(traj), voxel_grid}, "Point Cloud Viewer");
//o3d::visualization::DrawGeometries({voxel_grid}, "Point Cloud Viewer");

//    vector<std::shared_ptr<o3d::t::geometry::Geometry>> geometries;
//    geometries.push_back(std::make_shared<o3d::t::geometry::TriangleMesh>(t_mesh));
//    o3d::visualization::Draw({std::make_shared<o3d::t::geometry::PointCloud>(t_pc)}, "Point Cloud Viewer");

//    o3d::visualization::Draw({std::make_shared<o3d::t::geometry::PointCloud>(t_pc)}, "Point Cloud Viewer");
//    o3d::visualization::Draw({std::make_shared<o3d::t::geometry::TriangleMesh>(t_mesh)}, "Point Cloud Viewer");
    return 0;
}

