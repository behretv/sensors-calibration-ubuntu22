#include <chrono>  // NOLINT
#include <iostream>
#include <pcl/common/transforms.h>
#include <thread>  // NOLINT
#include <time.h>

#include "calibration.hpp"

void SaveExtrinsic(Eigen::Matrix4f T, const std::string &file_name)
{
  std::ofstream ofs(file_name);
  if (!ofs.is_open())
  {
    std::cerr << "open file " << file_name << " failed. Cannot write calib result." << std::endl;
    exit(1);
  }
  ofs << "(Roll,Pitch,Yaw,tx,ty,tz):";
  ofs << " " << Util::GetRoll(T) << " " << Util::GetPitch(T) << " " << Util::GetYaw(T);
  ofs << " " << Util::GetX(T) << " " << Util::GetY(T) << " " << Util::GetZ(T) << std::endl;
  ofs.close();

  std::cout << "\033[94mSaved: " << file_name << "\033[0m" << std::endl;
}

int main(int argc, char *argv[])
{
  if (argc != 2)
  {
    std::cout << "Usage: ./bin/run_lidar2camera <data_folder>\n"
                 "example:\n\t"
                 "./bin/run_lidar2camera data/st/1\n"
                 "./bin/run_lidar2camera data/kitti/1"
              << std::endl;
    return 0;
  }

  std::string data_folder = argv[1];
  std::string lidar_file, img_file, calib_file;
  std::string mask_dir = data_folder + "/calib/";
  std::string error_file = data_folder + "/initial_error.txt";
  std::string result_file = data_folder + "/refined_extrinsics.txt";

  DIR *dir;
  struct dirent *ptr;
  if ((dir = opendir(data_folder.c_str())) == NULL)
  {
    std::cout << "Open dir " << mask_dir << " error !" << std::endl;
    exit(1);
  }
  while ((ptr = readdir(dir)) != NULL)
  {
    std::string name = ptr->d_name;
    auto n = name.find_last_of('.');
    if (name == "." || name == ".." || n == std::string::npos)
    {
      ptr++;
      continue;
    }
    std::string suffix = name.substr(n);
    if (suffix == ".png" || suffix == ".jpg" || suffix == ".jpeg")
      img_file = data_folder + '/' + ptr->d_name;
    else if (suffix == ".pcd")
      lidar_file = data_folder + '/' + ptr->d_name;
    else if (suffix == ".txt")
      calib_file = data_folder + '/' + ptr->d_name;
    ptr++;
  }

  auto time_begin = std::chrono::steady_clock::now();
  Calibrator calibrator(mask_dir, lidar_file, calib_file, img_file, error_file);
  calibrator.Calibrate(data_folder);
  Eigen::Matrix4f refined_extrinsic = calibrator.GetFinalTransformation();
  SaveExtrinsic(refined_extrinsic, result_file);
  auto time_end = std::chrono::steady_clock::now();
  std::cout << "Total calib time: " << std::chrono::duration<double>(time_end - time_begin).count() << "s" << std::endl;

  return 0;
}
