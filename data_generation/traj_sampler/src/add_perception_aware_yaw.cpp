#include <execution>
#include <filesystem>
#include <Eigen/Dense>

#include "traj_sampler/perception_aware_cost_utils.hpp"

using namespace std;
namespace o3d = open3d;
namespace fs = std::filesystem;



void compute_perception_aware_cost_rollout(std::string rollout_dir, std::string out_csv_prefix) {
    std::string wf_csv_prefix = "trajectories_wf_";
    std::string bf_csv_prefix = "trajectories_bf_";

    std::string traj_dir = rollout_dir + "/trajectories/";
    std::string ply_file = rollout_dir + "/pointcloud-unity.ply";

    std::vector<Eigen::Vector3d> points_vec = parse_pointcloud(ply_file);
//    o3d::geometry::PointCloud point_cloud(points_vec);
//    pointcloud.Transform(transform.cast<float>())
//    color_pointcloud_z_gradient(point_cloud);
//    auto voxel_grid = o3d::geometry::VoxelGrid::CreateFromPointCloud(point_cloud, 0.2);
    auto start_time_rollout = std::chrono::high_resolution_clock::now();

     int file_count = 0;
     for (const auto& entry : fs::directory_iterator(traj_dir)) {
        if (entry.is_regular_file() && entry.path().filename().string().find(bf_csv_prefix) == 0 && entry.path().extension() == ".csv") {
            file_count++;
        }
    }
    std::cout << "Total files: " << file_count << std::endl;
    progressbar bar(file_count);
    for (const auto& entry : fs::directory_iterator(traj_dir)) {
        // Check if the file is a regular file and has the given prefix and ends with ".csv"
        if (entry.is_regular_file() && entry.path().filename().string().find(bf_csv_prefix) == 0 && entry.path().extension() == ".csv") {
            bar.update();
            std::string bf_csv_name = entry.path().filename().string();
            std::string filename_without_prefix = bf_csv_name.substr(bf_csv_prefix.size());
            std::string wf_csv_name = wf_csv_prefix + filename_without_prefix.substr(0, filename_without_prefix.size() - 4) + entry.path().extension().string();
            std::string bf_csv_file = traj_dir + bf_csv_name;
            std::string wf_csv_file = traj_dir + wf_csv_name;
            std::string output_csv_name = traj_dir + out_csv_prefix + entry.path().filename().string();
            std::ofstream output_file(output_csv_name);
            if (!output_file.is_open()) {
                std::cerr << "Error opening file: " << output_csv_name << std::endl;
                return;
            }
            auto trajectories_bf = parse_trajectory_from_csv(bf_csv_file);
            auto trajectories_wf = parse_trajectory_from_csv(wf_csv_file);
            auto start_time_traj = std::chrono::high_resolution_clock::now();
//            for (auto trajectory : trajectories_bf){
            for (int i = 0; i < trajectories_bf.size(); i++){
                auto trajectory_pos_bf = trajectories_bf[i].first;
                auto trajectory_rpy_bf = trajectories_bf[i].second;
                auto trajectory_pos_wf = trajectories_wf[i].first;
                auto trajectory_rpy_wf = trajectories_wf[i].second;

                Eigen::Matrix4d T_wf_to_bf = compute_transform(trajectory_pos_wf, trajectory_pos_bf);

                o3d::geometry::PointCloud point_cloud_bf(points_vec);
                point_cloud_bf.Transform(T_wf_to_bf);
                color_pointcloud_z_gradient(point_cloud_bf);
                auto voxel_grid_bf = o3d::geometry::VoxelGrid::CreateFromPointCloud(point_cloud_bf, 0.3);

//                o3d::geometry::PointCloud point_cloud_wf(points_vec);
//                color_pointcloud_z_gradient(point_cloud_wf);
//                auto voxel_grid_wf = o3d::geometry::VoxelGrid::CreateFromPointCloud(point_cloud_wf, 0.2);

//                std::vector<CameraConfig> init_camera_configs;
//                for (int j = 0; j < trajectory_pos_bf.size(); j++){
//                    init_camera_configs.push_back(CameraConfig(trajectory_pos_bf[j], trajectory_rpy_bf[j]));
//                }
//                std::vector<double> trajectory_desired_yaws(init_camera_configs.size());
//                auto compute_yaw = [&](auto& init_camera_config) { return get_best_yaw(*voxel_grid_bf, trajectory_pos_bf, init_camera_config)[2]; };
//                std::transform(std::execution::par, init_camera_configs.begin(), init_camera_configs.end(), trajectory_desired_yaws.begin(), compute_yaw);

                std::vector<Eigen::Quaterniond> trajectory_desired_quats;
//                std::vector<double> trajectory_desired_yaws;

                for (int j = 0; j < trajectory_pos_bf.size(); j++){
                    CameraConfig init_camera_config(trajectory_pos_bf[j], trajectory_rpy_bf[j]);
                    auto best_camera_rpy = get_best_yaw(*voxel_grid_bf, trajectory_pos_bf, init_camera_config);
                    trajectory_desired_quats.push_back(euler_to_quat(best_camera_rpy));
//                    trajectory_desired_yaws.push_back(best_camera_rpy[2]);


//                    auto voxel_grid_bf_viz = o3d::geometry::VoxelGrid::CreateFromPointCloud(point_cloud_bf, 0.2);
//                    o3d::geometry::PointCloud traj_viz(trajectory_pos_bf);
//                    traj_viz.PaintUniformColor({1, 0, 0});
//                    CameraConfig best_camera_config(trajectory_pos_bf[j], best_camera_rpy);
//                    double total_gain = compute_perception_aware_cost(*voxel_grid_bf_viz, trajectory_pos_bf, best_camera_config, true);
//                    open3d::visualization::DrawGeometries({std::make_shared<o3d::geometry::PointCloud>(traj_viz), voxel_grid_bf_viz});

                }
//                for (size_t i = 0; i < trajectory_pos_bf.size(); ++i) {
//                    output_file << trajectory_pos_bf[i][0] << ",";
//                }
//                for (size_t i = 0; i < trajectory_pos_bf.size(); ++i) {
//                    output_file << trajectory_pos_bf[i][1] << ",";
//                }
//                for (size_t i = 0; i < trajectory_pos_bf.size(); ++i) {
//                    output_file << trajectory_pos_bf[i][2] << ",";
//                }
//                for (size_t i = 0; i < trajectory_desired_yaws.size() - 1; ++i) {
//                    output_file << trajectory_desired_yaws[i] << ",";
//                }
//                output_file << trajectory_desired_yaws[trajectory_desired_yaws.size() - 1] << std::endl;

                for (size_t i = 0; i < trajectory_desired_quats.size() - 1; ++i) {
                    output_file << trajectory_desired_quats[i].w() << "," <<
                                   trajectory_desired_quats[i].x() << "," <<
                                   trajectory_desired_quats[i].y() << "," <<
                                   trajectory_desired_quats[i].z() << ",";
                }

                output_file << trajectory_desired_quats[trajectory_desired_quats.size() - 1].w() << "," <<
                               trajectory_desired_quats[trajectory_desired_quats.size() - 1].x() << "," <<
                               trajectory_desired_quats[trajectory_desired_quats.size() - 1].y() << "," <<
                               trajectory_desired_quats[trajectory_desired_quats.size() - 1].z() << std::endl;
            }
            // Close the file
            output_file.close();
            auto end_time_traj = std::chrono::high_resolution_clock::now();
            auto duration_traj = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_traj - start_time_traj);
//            std::cout << "Processing trajectory took " << duration_traj.count() / 1000.0 << "s" << std::endl;
        }
    }
    auto end_time_rollout = std::chrono::high_resolution_clock::now();
    auto duration_rollout = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_rollout - start_time_rollout);
    std::cout << "Processing rollout took " << duration_rollout.count() / 1000.0 << "s" << std::endl;
}

void compute_perception_aware_cost_rollouts(std::string root_dir, std::string out_csv_prefix) {
    for (const auto& entry : fs::directory_iterator(root_dir)) {
        if (entry.is_directory()) {
            std::string dir_name = entry.path().filename().string();
            if (dir_name.find("rollout_") == 0) {
                compute_perception_aware_cost_rollout(entry.path(), out_csv_prefix);
            }
        }
    }
}


int main(int argc, char* argv[]) {
    std::string root_dir = "/root/agile_autonomy_ws/catkin_aa/src/agile_autonomy/data_generation";
    std::string out_csv_prefix = "perc_aware_";
    compute_perception_aware_cost_rollouts(root_dir, out_csv_prefix);

    return 0;
}

