#include <chrono>  // NOLINT
#include <iostream>
#include <pcl/common/transforms.h>
#include <thread>  // NOLINT
#include <time.h>

#include "calibration.hpp"

unsigned char color_map[10][3] = {{255, 255, 255},  // "white"
                                  {255, 0, 0},      // "red"
                                  {0, 255, 0},      // "green"
                                  {0, 0, 255},      // "blue"
                                  {255, 255, 0},    // "yellow"
                                  {255, 0, 255},    // "pink"
                                  {50, 255, 255},   // "light-blue"
                                  {135, 60, 0},     //
                                  {150, 240, 80},   //
                                  {80, 30, 180}};   //

void LoadPointCloud(const std::string &filename, std::map<int32_t, pcl::PointCloud<pcl::PointXYZI>> &lidar_points)
{
  std::ifstream file(filename);
  if (!file.is_open())
  {
    std::cout << "[ERROR] open file " << filename << " failed." << std::endl;
    exit(1);
  }
  std::string line, tmpStr;
  while (getline(file, line))
  {
    int32_t device_id;
    std::string point_cloud_path;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    std::stringstream ss(line);
    ss >> tmpStr >> device_id;
    getline(file, line);
    ss = std::stringstream(line);
    ss >> tmpStr >> point_cloud_path;
    if (pcl::io::loadPCDFile(point_cloud_path, *cloud) < 0)
    {
      std::cout << "[ERROR] cannot open pcd_file: " << point_cloud_path << "\n";
      exit(1);
    }
    lidar_points.insert(std::make_pair(device_id, *cloud));
  }
}

void LoadCalibFile(const std::string &filename, std::map<int32_t, InitialExtrinsic> &calib_extrinsic)
{
  std::ifstream file(filename);
  if (!file.is_open())
  {
    std::cout << "open file " << filename << " failed." << std::endl;
    exit(1);
  }
  float degree_2_radian = 0.017453293;
  std::string line, tmpStr;
  while (getline(file, line))
  {
    int32_t device_id;
    InitialExtrinsic extrinsic;
    std::stringstream ss(line);
    ss >> tmpStr >> device_id;
    getline(file, line);
    ss = std::stringstream(line);
    ss >> tmpStr >> extrinsic.euler_angles[0] >> extrinsic.euler_angles[1] >> extrinsic.euler_angles[2] >>
        extrinsic.t_matrix[0] >> extrinsic.t_matrix[1] >> extrinsic.t_matrix[2];

    extrinsic.euler_angles[0] = extrinsic.euler_angles[0] * degree_2_radian;
    extrinsic.euler_angles[1] = extrinsic.euler_angles[1] * degree_2_radian;
    extrinsic.euler_angles[2] = extrinsic.euler_angles[2] * degree_2_radian;
    calib_extrinsic.insert(std::make_pair(device_id, extrinsic));
  }
}

int main(int argc, char *argv[])
{
  if (argc != 3)
  {
    std::cout << "Usage: ./run_lidar2lidar <lidar_file>.txt <calib_file>.txt" << std::endl;
    return 0;
  }
  auto lidar_file = argv[1];
  auto calib_file = argv[2];
  std::map<int32_t, pcl::PointCloud<pcl::PointXYZI>> lidar_points;
  LoadPointCloud(lidar_file, lidar_points);
  std::map<int32_t, InitialExtrinsic> extrinsics;
  LoadCalibFile(calib_file, extrinsics);

  // calibration
  Calibrator calibrator;
  calibrator.LoadCalibrationData(lidar_points, extrinsics);
  auto time_begin = std::chrono::steady_clock::now();
  calibrator.Calibrate();
  auto time_end = std::chrono::steady_clock::now();
  std::cout << "Calibration took " << std::chrono::duration<double>(time_end - time_begin).count() << "s" << std::endl;

  // Save calibration result
  std::ofstream file_stream;
  file_stream.open("config/refined_extrinsic.txt");
  std::map<int32_t, Eigen::Matrix4d> refined_extrinsics = calibrator.GetFinalTransformation();
  for (auto iter = refined_extrinsics.begin(); iter != refined_extrinsics.end(); iter++)
  {
    int32_t slave_id = iter->first;
    Eigen::Matrix4d transform = iter->second;
    auto translation(transform.block<3, 1>(0, 3));
    auto euler_angles = Eigen::Matrix3d(transform.block<3, 3>(0, 0)).eulerAngles(0,1,2);

    // Create file
    file_stream << "device_id: " << slave_id << std::endl;
    file_stream << "(Roll,Pitch,Yaw,tx,tty,tz):" << euler_angles << translation << std::endl;
  }
  file_stream.close();
  return 0;
}
