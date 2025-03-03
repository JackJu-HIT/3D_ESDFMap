#include "ESDFMap.h"
#include "parameters.h"
#include <vector>
#include <iostream>
#include <Eigen/Eigen>
#include <cassert>
#include <algorithm>

using std::cout;
using std::endl;
using std::vector;
using namespace fiesta;

int main() {
    Eigen::Vector3d origin(0.0, 0.0, 0.0);
    double resolution = 0.1;
    Eigen::Vector3d map_size(10.0, 10.0, 10.0);
    Parameters parameters_;
    parameters_.SetParameters();
#ifdef HASH_TABLE
    fiesta::ESDFMap esdf_map(origin, resolution, 1000);
#else
    fiesta::ESDFMap esdf_map(origin, resolution, map_size);
#endif

#ifdef PROBABILISTIC
    esdf_map.SetParameters(parameters_.p_hit_, parameters_.p_miss_,
                           parameters_.p_min_, parameters_.p_max_, parameters_.p_occ_);
#endif

    Eigen::Vector3d unoccupied_pos(2.0, 2.0, 2.0);
    Eigen::Vector3d obstacle_pos(5.0, 5.0, 5.0);

    // 设置障碍物并强制更新
    esdf_map.SetOccupancy(obstacle_pos, 1);

    // 强制全局更新ESDF
    esdf_map.SetOriginalRange();
    esdf_map.UpdateOccupancy(true);  // 确保全局更新
    esdf_map.UpdateESDF();

    // 计算距离
    double distance = esdf_map.GetDistance(unoccupied_pos);

    // 理论欧氏距离计算
    double expected_distance = (obstacle_pos - unoccupied_pos).norm();

    std::cout << "未占据元素位置: (" << unoccupied_pos.transpose() << ")\n";
    std::cout << "障碍物位置: (" << obstacle_pos.transpose() << ")\n";
    std::cout << "计算距离: " << distance << std::endl;
    std::cout << "理论距离: " << expected_distance << std::endl;

    return 0;
}