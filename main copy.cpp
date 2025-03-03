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

// // 测试 ESDFMap 类的基本功能
// void testESDFMapBasicFunctionality() {
//     // 初始化参数
//     Parameters parameters_;
//     Eigen::Vector3d origin, map_size;
//     origin << -5.0, -5.0, 0.0;
//     map_size << 10.0, 10.0, 5.0;
//     double resolution = 0.2;

//     // 创建 ESDFMap 对象
//     ESDFMap *esdf_map_ = new ESDFMap(origin, resolution, map_size);

//     // 设置一些位置的占据信息
//     Eigen::Vector3d pos;
//     Eigen::Vector3d cur_pos_;
//     cur_pos_ << 10, 10, 5;

//     std::vector<std::pair<int, int>> vp;
//     for (int x = -4; x <= 4; x += 2) {
//         for (int y = -4; y <= 4; y += 2) {
//             vp.push_back(std::make_pair(x, y));
//         }
//     }

//     // 随机打乱位置对的顺序
//     std::random_shuffle(vp.begin(), vp.end());

//     // 插入占据信息
//     double tt = 0;
//     for (auto iter = vp.begin(); iter != vp.end(); iter++) {
//         // 一个长方体
//         for (double z = 0.0; z < 5.0; z += 0.1) {
//             pos << iter->first, iter->second, z;
//             int occupancyResult = esdf_map_->SetOccupancy(pos, 1);
//             // 检查设置占据信息是否成功
//             assert(occupancyResult >= 0);
//         }

//         // 检查是否需要更新范围
//         if (esdf_map_->CheckUpdate()) {
//         //     std::cout << ""
//         //     esdf_map_->SetOriginalRange();
//         // } else {
//             esdf_map_->SetUpdateRange(cur_pos_ - parameters_.radius_, cur_pos_ + parameters_.radius_);
//             esdf_map_->UpdateOccupancy(true); // parameters_.global_update_
//             esdf_map_->UpdateESDF();
//         }
//     }

//     // 检查某个位置的占据信息是否正确设置
//     Eigen::Vector3d testPos;
//     testPos << 0, 0, 0;
//     int occupancy = esdf_map_->GetOccupancy(testPos);
//     cout << "Occupancy at (0, 0, 0): " << occupancy << endl;

//     // 清理内存
//     delete esdf_map_;
// }

// int main() {
//     cout << "Starting ESDFMap tests..." << endl;
//     testESDFMapBasicFunctionality();
//     cout << "All tests passed!" << endl;
//     return 0;
// }


#include "ESDFMap.h"
#include "parameters.h"
#include <vector>
#include <iostream>
#include <Eigen/Eigen>

using std::cout;
using std::endl;
using std::vector;
using namespace fiesta;


// int main()
// {
//     Parameters parameters_;
//     Eigen::Vector3d origin, map_size;
//     origin << -5.0, -5.0, 0.0;
//     map_size << 10.0, 10.0, 5.0;
//     double resolution = 0.2;
//     ESDFMap *esdf_map_ = new ESDFMap(origin,resolution, map_size);

//     // set occupancy for some positions
//     Eigen::Vector3d pos;
//     Eigen::Vector3d cur_pos_;
//     cur_pos_<< 10,10,5;

//     std::vector<std::pair<int, int>> vp;
//     for (int x = -4; x <= 4; x += 2)
//         for (int y = -4; y <= 4; y += 2)
//             vp.push_back(std::make_pair(x, y));

//      // insert
//     double tt = 0;
//     std::random_shuffle(vp.begin(), vp.end());
//     for (auto iter = vp.begin(); iter != vp.end(); iter++) {
//         // a cuboid
//         for (double z = 0.0; z < 5.0; z += 0.1) {
//             pos << iter->first, iter->second, z;
//             esdf_map_->SetOccupancy(pos, 1);
//         } // z


//      if (esdf_map_->CheckUpdate()) 
//                esdf_map_->SetOriginalRange();
//       else
//           esdf_map_->SetUpdateRange(cur_pos_ - parameters_.radius_, cur_pos_ + parameters_.radius_);
//           esdf_map_->UpdateOccupancy(true);//parameters_.global_update_);
//           esdf_map_->UpdateESDF();
//      }

 



//      return 0;

    
// }




int main()
{

    // 创建一个地图
    Eigen::Vector3d origin(-5.0, -5.0, 0.0);
    Eigen::Vector3d map_size(30.0, 30.0, 30.0);
    double resolution = 0.2;
    Parameters parameters_;
    parameters_.SetParameters();
    ESDFMap*esdf_map = new  ESDFMap(origin, resolution, map_size);
    esdf_map->SetParameters(parameters_.p_hit_, parameters_.p_miss_,
                              parameters_.p_min_, parameters_.p_max_, parameters_.p_occ_);

    // // 定义一些用于测试的位置
    // std::vector<Eigen::Vector3d> test_positions;
    // for (int x = -2; x <= 2; x++) {
    //     for (int y = -2; y <= 2; y++) {
    //         for (double z = 1.0; z < 2.0; z += 0.5) {
    //             Eigen::Vector3d pos(x, y, z);
    //             test_positions.push_back(pos);
    //         }
    //     }
    // }// 定义一些用于测试的位置
    int num = 0;
   // while(num < 2)

    {
    num++;
    std::vector<Eigen::Vector3d> test_positions;
    for (int x = -2; x <= 2; x++) {
        for (int y = -2; y <= 2; y++) {
            for (double z = 1.0; z < 2.0; z += 0.5) {
                Eigen::Vector3d pos(x, y, z);
                // 计算地图的最大世界坐标
                Eigen::Vector3d max_world_coord = origin + map_size * resolution;
                // 检查位置是否在地图范围内
                {
                    test_positions.push_back(pos);
                }
            }
        }
    }
    // 插入操作
    std::cout << "Starting insert first operations..." << std::endl;
    for (const auto& pos : test_positions) {
        int result = esdf_map->SetOccupancy(pos, 1);
          std::cerr << " position: " << pos.transpose() << std::endl;
          break;
        // if (result != 0) {
      
        //     std::cerr << "Insert operation failed at position: " << pos.transpose() << std::endl;
        //       break;
        // } else 
        // {
        //     std::cerr << "Insert operation success at position: " << pos.transpose() << std::endl;
        // }
        // esdf_map.SetOriginalRange();
        // esdf_map.UpdateOccupancy(true);
        // esdf_map.UpdateESDF();
        // return 0;
        // 这里可以添加更多检查逻辑，比如调用 esdf_map.CheckConsistency() 等
    }
    // esdf_map.SetOriginalRange();
    // esdf_map.UpdateOccupancy(false);
    // esdf_map.UpdateESDF();
      if (esdf_map->CheckUpdate()) {
          esdf_map->SetOriginalRange();
          esdf_map->UpdateOccupancy(parameters_.global_update_);
          esdf_map->UpdateESDF();
      }
        // 检查某个位置的占据信息是否正确设置
    Eigen::Vector3d testPos;
    testPos <<  -2,-2,1;
    int occupancy = esdf_map->GetOccupancy(testPos);
    cout << "Occupancy at ( -2  -2 1): " << occupancy << endl;

    // 获取该点到最近障碍物的距离
    double distance = esdf_map->GetDistance(testPos);
    if (distance == -1000) {
        std::cout << "无法获取该点到最近障碍物的距离（可能点不在地图范围内）" << std::endl;
    } else {
        std::cout << "该点到最近障碍物的距离为: " << distance << std::endl;
    }


     // 插入操作
    std::cout << "Starting insert second operations..." << std::endl;
    for (const auto& pos : test_positions) {
        int result = esdf_map->SetOccupancy(pos, 1);
          std::cerr << " position: " << pos.transpose() << std::endl;
          break;
        // if (result != 0) {
      
        //     std::cerr << "Insert operation failed at position: " << pos.transpose() << std::endl;
        //       break;
        // } else 
        // {
        //     std::cerr << "Insert operation success at position: " << pos.transpose() << std::endl;
        // }
        // esdf_map.SetOriginalRange();
        // esdf_map.UpdateOccupancy(true);
        // esdf_map.UpdateESDF();
        // return 0;
        // 这里可以添加更多检查逻辑，比如调用 esdf_map.CheckConsistency() 等
    }
    // esdf_map.SetOriginalRange();
    // esdf_map.UpdateOccupancy(false);
    // esdf_map.UpdateESDF();
      if (esdf_map->CheckUpdate()) {
          esdf_map->SetOriginalRange();
          esdf_map->UpdateOccupancy(parameters_.global_update_);
          esdf_map->UpdateESDF();
      }
        // 检查某个位置的占据信息是否正确设置
    // Eigen::Vector3d testPos;
    // testPos <<  -2,-2,1;
    occupancy = esdf_map->GetOccupancy(testPos);
    cout << "Occupancy at ( -2  -2 1): " << occupancy << endl;

    // 获取该点到最近障碍物的距离
    distance = esdf_map->GetDistance(testPos);
    if (distance == -1000) {
        std::cout << "无法获取该点到最近障碍物的距离（可能点不在地图范围内）" << std::endl;
    } else {
        std::cout << "该点到最近障碍物的距离为: " << distance << std::endl;
    }

    // // 打乱位置顺序
    // std::random_shuffle(test_positions.begin(), test_positions.end());

    // // 删除操作
    // std::cout << "Starting delete first operations..." << std::endl;
    // for (const auto& pos : test_positions) {
    //     int result = esdf_map->SetOccupancy(pos, 0);
    //     if (result != 0) {
    //         std::cerr << "Delete operation failed at position: " << pos.transpose() << std::endl;
    //     } else
    //     {
    //          std::cerr << "Delete operation success at position: " << pos.transpose() << std::endl;
    //     }
    //     break;
    //     // esdf_map.SetOriginalRange();
    //     // esdf_map.UpdateOccupancy(true);
    //     // esdf_map.UpdateESDF();
    //     // 这里可以添加更多检查逻辑，比如调用 esdf_map.CheckConsistency() 等
    // }
    //  if (esdf_map->CheckUpdate()) {
    //       esdf_map->SetOriginalRange();
    //       esdf_map->UpdateOccupancy(parameters_.global_update_);
    //       esdf_map->UpdateESDF();
    //   }
    // std::cout << "Delete operations completed." << std::endl;
    //    // 检查某个位置的占据信息是否正确设置
    // Eigen::Vector3d testPos1;
    // testPos1 <<  -2,-2,1;
    // occupancy = esdf_map->GetOccupancy(testPos1);
    // cout << "delete Occupancy at ( -2  -2 1): " << occupancy << endl;
    //   // 获取该点到最近障碍物的距离
    //  distance = esdf_map->GetDistance(testPos1);
    // if (distance == -1000) {
    //     std::cout << "de;lete 无法获取该点到最近障碍物的距离（可能点不在地图范围内）" << std::endl;
    // } else {
    //     std::cout << "delete 该点到最近障碍物的距离为: " << distance << std::endl;
    // }


    // // 删除操作
    // std::cout << "Starting delete second operations..." << std::endl;
    // for (const auto& pos : test_positions) {
    //     int result = esdf_map->SetOccupancy(pos, 0);
    //     if (result != 0) {
    //         std::cerr << "Delete operation failed at position: " << pos.transpose() << std::endl;
    //     } else
    //     {
    //          std::cerr << "Delete operation success at position: " << pos.transpose() << std::endl;
    //     }
    //     break;
    //     // esdf_map.SetOriginalRange();
    //     // esdf_map.UpdateOccupancy(true);
    //     // esdf_map.UpdateESDF();
    //     // 这里可以添加更多检查逻辑，比如调用 esdf_map.CheckConsistency() 等
    // }
    //  if (esdf_map->CheckUpdate()) {
    //       esdf_map->SetOriginalRange();
    //       esdf_map->UpdateOccupancy(parameters_.global_update_);
    //       esdf_map->UpdateESDF();
    //   }
    // std::cout << "Delete operations completed." << std::endl;
    //    // 检查某个位置的占据信息是否正确设置
    // // //Eigen::Vector3d testPos1;
    // // testPos1 <<  -2,-2,1;
    //  occupancy = esdf_map->GetOccupancy(testPos1);
    // cout << "delete Occupancy at ( -2  -2 1): " << occupancy << endl;

    // // 获取该点到最近障碍物的距离
    //  distance = esdf_map->GetDistance(testPos1);

    // // 获取该点到最近障碍物的距离
    //  distance = esdf_map->GetDistance(testPos1);
    // if (distance == -1000) {
    //     std::cout << "de;lete 无法获取该点到最近障碍物的距离（可能点不在地图范围内）" << std::endl;
    // } else {
    //     std::cout << "delete 该点到最近障碍物的距离为: " << distance << std::endl;
    // }

    Eigen::Vector3d testPos3;
    testPos3 <<  -2,-2,2;

    // 检查某个位置的占据信息是否正确设置
    // //Eigen::Vector3d testPos1;
    // testPos1 <<  -2,-2,1;
    occupancy = esdf_map->GetOccupancy(testPos3);
    cout << "delete Occupancy at ( -2  -2 2): " << occupancy << endl;


    // 获取该点到最近障碍物的距离
     distance = esdf_map->GetDistance(testPos3);
    if (distance == -1000) {
        std::cout << "de;lete 无法获取该点到最近障碍物的距离（可能点不在地图范围内）" << std::endl;
    } else {
        std::cout << "delete 该点到最近障碍物的距离为: " << distance << std::endl;
    }

    






}
    delete esdf_map;


//  // 创建一个地图
//     Eigen::Vector3d origin, map_size;
//     origin << -5.0, -5.0, 0.0;
//     map_size << 10.0, 10.0, 5.0;
//     double resolution = 0.2;
//     fiesta::ESDFMap esdf_map(origin, resolution,map_size);

//     // 设置一些位置的占用情况
//     Eigen::Vector3d pos;

//     std::vector<std::pair<int, int>> vp;
//     for (int x = -4; x <= 4; x += 2)
//         for (int y = -4; y <= 4; y += 2)
//             vp.push_back(std::make_pair(x, y));

//     // 插入操作
//     double tt = 0;
//     std::random_shuffle(vp.begin(), vp.end());
//     for (auto iter = vp.begin(); iter != vp.end(); iter++) {
//         // 一个长方体
//         for (double z = 0.0; z < 5.0; z += 0.1) {
//             pos << iter->first, iter->second, z;
//             esdf_map.SetOccupancy(pos, 1);
//         } // z

//         esdf_map.UpdateESDF();
//     } // iter

//     cout << "Total Time Used is " << tt << "s." << endl;
//     // visulization(esdf_map);

//     // 删除操作
//     tt = 0;
//     std::random_shuffle(vp.begin(), vp.end());
//     for (auto iter = vp.begin(); iter != vp.end(); iter++) {
//         // 一个长方体
//         for (double z = 0.0; z < 5.0; z += 0.1) {
//             pos << iter->first, iter->second, z;
//             esdf_map.SetOccupancy(pos, 0);
//         } // z

//         esdf_map.UpdateESDF();
//     } // iter
//     cout << "Total Time Used is " << tt << "s." << endl;

}
