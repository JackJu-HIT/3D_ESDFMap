/**
*@Create by:juchunyu@qq.com
*@Date:2025-03-03 19:57:00
**/
#include "ESDFMap.h"
#include "parameters.h"
#include <vector>
#include <iostream>
#include <Eigen/Eigen>
#include <cassert>
#include <algorithm>
#include <random>

using std::cout;
using std::endl;
using std::vector;
using namespace fiesta;



int main() {
    // 初始化 ESDFMap
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

    // 设置一些参数（如果需要）
#ifdef PROBABILISTIC
    //esdf_map.SetParameters(0.7, 0.4, 0.1, 0.9, 0.5);
   esdf_map.SetParameters(parameters_.p_hit_, parameters_.p_miss_,
                            parameters_.p_min_, parameters_.p_max_, parameters_.p_occ_);
 #endif
    // 假设一个未占据的元素的位置
     Eigen::Vector3d unoccupied_pos(2.0, 2.0, 2.0);
    //Eigen::Vector3d unoccupied_pos(4.9, 4.9, 4.9);
    // 假设最近障碍物的位置
    Eigen::Vector3d obstacle_pos(5.0, 5.0, 5.0);

     // 将障碍物位置设置为占据
    esdf_map.SetOccupancy(unoccupied_pos, 0);


    // 将障碍物位置设置为占据
    esdf_map.SetOccupancy(obstacle_pos, 1);

    //Eigen::Vector3d min_pos, Eigen::Vector3d max_pos
     esdf_map.SetUpdateRange(origin,map_size);
     esdf_map.SetOriginalRange();
    // 检查并更新地图
    if (esdf_map.CheckUpdate()) {
        esdf_map.SetOriginalRange();
        esdf_map.UpdateOccupancy(parameters_.global_update_);
        esdf_map.UpdateESDF();
    }

    // 将障碍物位置设置为占据
    esdf_map.SetOccupancy(obstacle_pos, 1);
    //Eigen::Vector3d min_pos, Eigen::Vector3d max_pos
     esdf_map.SetUpdateRange(origin,map_size);
     esdf_map.SetOriginalRange();
    // 检查并更新地图
    if (esdf_map.CheckUpdate()) {
        esdf_map.SetOriginalRange();
        esdf_map.UpdateOccupancy(parameters_.global_update_);
        esdf_map.UpdateESDF();
    }

    // // 计算未占据元素到最近障碍物的距离
    double distance = esdf_map.GetDistance(unoccupied_pos);
    int occupancy = esdf_map.GetOccupancy(obstacle_pos);
    int unoccupied_pos_res = esdf_map.GetOccupancy(unoccupied_pos);

    // 输出结果
    std::cout << "未占据元素位置: (" << unoccupied_pos(0) << ", " << unoccupied_pos(1) << ", " << unoccupied_pos(2) << ")" << unoccupied_pos_res << std::endl;
    std::cout << "最近障碍物位置: (" << obstacle_pos(0) << ", " << obstacle_pos(1) << ", " << obstacle_pos(2) << ")" << "occupancy =" << occupancy << std::endl;
    std::cout << "计算得到的距离: " << distance << std::endl;
     std::cout << "查询点距离: " << distance << " (理论值: " 
               << (unoccupied_pos - obstacle_pos).norm() << ")\n";




    // 将障碍物位置设置为占据
    esdf_map.SetOccupancy(obstacle_pos, 0);
    //Eigen::Vector3d min_pos, Eigen::Vector3d max_pos
     esdf_map.SetUpdateRange(origin,map_size);
     esdf_map.SetOriginalRange();
    // 检查并更新地图
    if (esdf_map.CheckUpdate()) {
        esdf_map.SetOriginalRange();
        esdf_map.UpdateOccupancy(parameters_.global_update_);
        esdf_map.UpdateESDF();
    }


    // // 计算未占据元素到最近障碍物的距离
    distance = esdf_map.GetDistance(unoccupied_pos);
    occupancy = esdf_map.GetOccupancy(obstacle_pos);
    unoccupied_pos_res = esdf_map.GetOccupancy(unoccupied_pos);

    // 输出结果
    std::cout << "未占据元素位置: (" << unoccupied_pos(0) << ", " << unoccupied_pos(1) << ", " << unoccupied_pos(2) << ")" << unoccupied_pos_res << std::endl;
    std::cout << "最近障碍物位置: (" << obstacle_pos(0) << ", " << obstacle_pos(1) << ", " << obstacle_pos(2) << ")" << "occupancy =" << occupancy << std::endl;
    std::cout << "计算得到的距离: " << distance << std::endl;
     std::cout << "查询点距离: " << distance << " (理论值: " 
               << (unoccupied_pos - obstacle_pos).norm() << ")\n";

    return 0;
}

