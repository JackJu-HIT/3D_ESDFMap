#include <iostream>
#include <Eigen/Dense>
#include <queue>
#include <cmath>
#include <vector>

namespace fiesta {

class ESDFMap {
public:
    ESDFMap(Eigen::Vector3d origin, double resolution, Eigen::Vector3d map_size);
    bool Exist(const int &idx);
    void DeleteFromList(int link, int idx);
    void InsertIntoList(int link, int idx);
    bool PosInMap(Eigen::Vector3d pos);
    bool VoxInRange(Eigen::Vector3i vox);
    void Pos2Vox(Eigen::Vector3d pos, Eigen::Vector3i &vox);
    void Vox2Pos(Eigen::Vector3i vox, Eigen::Vector3d &pos);
    int Vox2Idx(Eigen::Vector3i vox);
    Eigen::Vector3i Idx2Vox(int idx);
    double Dist(const Eigen::Vector3i a, const Eigen::Vector3i b);
    bool UpdateOccupancy(bool global_map);
    void UpdateESDF();
    int SetOccupancy(Eigen::Vector3d pos, int occ);
    int GetOccupancy(Eigen::Vector3d pos);
    double GetDistance(Eigen::Vector3d pos);

private:
    Eigen::Vector3d origin_;
    double resolution_;
    Eigen::Vector3d map_size_;
    double resolution_inv_;
    Eigen::Vector3i grid_size_;
    int grid_size_yz_;
    int grid_total_size_;
    double infinity_;
    int undefined_;
    int reserved_idx_4_undefined_;
    std::vector<double> occupancy_buffer_;
    std::vector<double> distance_buffer_;
    std::vector<Eigen::Vector3i> closest_obstacle_;
    std::vector<int> head_;
    std::vector<int> prev_;
    std::vector<int> next_;
    std::queue<Eigen::Vector3i> insert_queue_;
    std::queue<Eigen::Vector3i> delete_queue_;
    std::queue<Eigen::Vector3i> update_queue_;
    std::queue<Eigen::Vector3i> occupancy_queue_;
    std::vector<Eigen::Vector3i> dirs_;
    int num_dirs_;
};

ESDFMap::ESDFMap(Eigen::Vector3d origin, double resolution, Eigen::Vector3d map_size)
    : origin_(origin), resolution_(resolution), map_size_(map_size) {
    resolution_inv_ = 1 / resolution_;

    for (int i = 0; i < 3; ++i)
        grid_size_(i) = std::ceil(map_size(i) / resolution_);

    grid_size_yz_ = grid_size_(1) * grid_size_(2);
    infinity_ = 10000;
    undefined_ = -10000;

    grid_total_size_ = grid_size_(0) * grid_size_yz_;
    reserved_idx_4_undefined_ = grid_total_size_;

    occupancy_buffer_.resize(grid_total_size_);
    distance_buffer_.resize(grid_total_size_);
    closest_obstacle_.resize(grid_total_size_);

    std::fill(distance_buffer_.begin(), distance_buffer_.end(), infinity_);
    std::fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), 0);
    std::fill(closest_obstacle_.begin(), closest_obstacle_.end(), Eigen::Vector3i(undefined_, undefined_, undefined_));

    head_.resize(grid_total_size_ + 1);
    prev_.resize(grid_total_size_);
    next_.resize(grid_total_size_);
    std::fill(head_.begin(), head_.end(), undefined_);
    std::fill(prev_.begin(), prev_.end(), undefined_);
    std::fill(next_.begin(), next_.end(), undefined_);

    // 初始化邻域方向
    dirs_ = {
        Eigen::Vector3i(1, 0, 0), Eigen::Vector3i(-1, 0, 0),
        Eigen::Vector3i(0, 1, 0), Eigen::Vector3i(0, -1, 0),
        Eigen::Vector3i(0, 0, 1), Eigen::Vector3i(0, 0, -1),
        Eigen::Vector3i(1, 1, 0), Eigen::Vector3i(-1, -1, 0),
        Eigen::Vector3i(1, -1, 0), Eigen::Vector3i(-1, 1, 0),
        Eigen::Vector3i(1, 0, 1), Eigen::Vector3i(-1, 0, -1),
        Eigen::Vector3i(1, 0, -1), Eigen::Vector3i(-1, 0, 1),
        Eigen::Vector3i(0, 1, 1), Eigen::Vector3i(0, -1, -1),
        Eigen::Vector3i(0, 1, -1), Eigen::Vector3i(0, -1, 1)
    };
    num_dirs_ = dirs_.size();
}

bool ESDFMap::Exist(const int &idx) {
    return occupancy_buffer_[idx] == 1;
}

void ESDFMap::DeleteFromList(int link, int idx) {
    if (prev_[idx] != undefined_)
        next_[prev_[idx]] = next_[idx];
    else
        head_[link] = next_[idx];
    if (next_[idx] != undefined_)
        prev_[next_[idx]] = prev_[idx];
    prev_[idx] = next_[idx] = undefined_;
}

void ESDFMap::InsertIntoList(int link, int idx) {
    if (head_[link] == undefined_)
        head_[link] = idx;
    else {
        prev_[head_[link]] = idx;
        next_[idx] = head_[link];
        head_[link] = idx;
    }
}

bool ESDFMap::PosInMap(Eigen::Vector3d pos) {
    for (int i = 0; i < 3; ++i) {
        if (pos(i) < origin_(i) || pos(i) > origin_(i) + map_size_(i))
            return false;
    }
    return true;
}

bool ESDFMap::VoxInRange(Eigen::Vector3i vox) {
    for (int i = 0; i < 3; ++i) {
        if (vox(i) < 0 || vox(i) >= grid_size_(i))
            return false;
    }
    return true;
}

void ESDFMap::Pos2Vox(Eigen::Vector3d pos, Eigen::Vector3i &vox) {
    for (int i = 0; i < 3; ++i)
        vox(i) = std::floor((pos(i) - origin_(i)) / resolution_);
}

void ESDFMap::Vox2Pos(Eigen::Vector3i vox, Eigen::Vector3d &pos) {
    for (int i = 0; i < 3; ++i)
        pos(i) = (vox(i) + 0.5) * resolution_ + origin_(i);
}

int ESDFMap::Vox2Idx(Eigen::Vector3i vox) {
    if (vox(0) == undefined_)
        return reserved_idx_4_undefined_;
    return vox(0) * grid_size_yz_ + vox(1) * grid_size_(2) + vox(2);
}

Eigen::Vector3i ESDFMap::Idx2Vox(int idx) {
    return Eigen::Vector3i(idx / grid_size_yz_,
        idx % (grid_size_yz_) / grid_size_(2),
        idx % grid_size_(2));
}

double ESDFMap::Dist(const Eigen::Vector3i a, const Eigen::Vector3i b) {
    Eigen::Vector3d delta = (a - b).cast<double>() * resolution_;
    return delta.norm();
}

bool ESDFMap::UpdateOccupancy(bool global_map) {
    while (!occupancy_queue_.empty()) {
        Eigen::Vector3i xx = occupancy_queue_.front();
        occupancy_queue_.pop();
        int idx = Vox2Idx(xx);
        if (idx < 0 || idx >= grid_total_size_) continue;
        occupancy_buffer_[idx] = 1;
        insert_queue_.push(xx);
    }
    return !insert_queue_.empty();
}

void ESDFMap::UpdateESDF() {
    // 初始化障碍物的距离为 0
    while (!insert_queue_.empty()) {
        Eigen::Vector3i xx = insert_queue_.front();
        insert_queue_.pop();
        int idx = Vox2Idx(xx);
        if (Exist(idx)) {
            distance_buffer_[idx] = 0.0;
            closest_obstacle_[idx] = xx;
            update_queue_.push(xx);
        }
    }

    // 更新距离场
    while (!update_queue_.empty()) {
        Eigen::Vector3i xx = update_queue_.front();
        update_queue_.pop();
        int idx = Vox2Idx(xx);

        for (const auto &dir : dirs_) {
            Eigen::Vector3i new_pos = xx + dir;
            if (VoxInRange(new_pos)) {
                int new_pos_idx = Vox2Idx(new_pos);
                double new_dist = distance_buffer_[idx] + Dist(xx, new_pos);
                if (new_dist < distance_buffer_[new_pos_idx]) {
                    distance_buffer_[new_pos_idx] = new_dist;
                    closest_obstacle_[new_pos_idx] = closest_obstacle_[idx];
                    update_queue_.push(new_pos);
                }
            }
        }
    }
}

int ESDFMap::SetOccupancy(Eigen::Vector3d pos, int occ) {
    if (occ != 1 && occ != 0) {
        std::cout << "occ value error!" << std::endl;
        return undefined_;
    }
    if (!PosInMap(pos)) {
        std::cout << "Not in map" << std::endl;
        return undefined_;
    }
    Eigen::Vector3i vox;
    Pos2Vox(pos, vox);
    occupancy_queue_.push(vox);
    return Vox2Idx(vox);
}

int ESDFMap::GetOccupancy(Eigen::Vector3d pos) {
    if (!PosInMap(pos))
        return undefined_;
    Eigen::Vector3i vox;
    Pos2Vox(pos, vox);
    return Exist(Vox2Idx(vox));
}

double ESDFMap::GetDistance(Eigen::Vector3d pos) {
    if (!PosInMap(pos))
        return infinity_;
    Eigen::Vector3i vox;
    Pos2Vox(pos, vox);
    int idx = Vox2Idx(vox);
    return distance_buffer_[idx];
}

} // namespace fiesta

// 测试程序
int main() {
    // 初始化地图
    Eigen::Vector3d origin(0, 0, 0);
    double resolution = 1.0;
    Eigen::Vector3d map_size(10, 10, 10);
    fiesta::ESDFMap esdfMap(origin, resolution, map_size);

    // 设置障碍物
    Eigen::Vector3d obstacle_pos(5, 5, 5);
    esdfMap.SetOccupancy(obstacle_pos, 1);

    // 更新占用信息和 ESDF 地图
    esdfMap.UpdateOccupancy(true);
    esdfMap.UpdateESDF();

    // 查询点
    Eigen::Vector3d query_pos(2, 2, 2);
    double distance = esdfMap.GetDistance(query_pos);

    // 理论距离
    double theoretical_distance = (query_pos - obstacle_pos).norm();

    std::cout << "未占据元素位置22: (" << query_pos(0) << ", " << query_pos(1) << ", " << query_pos(2) << ")" << std::endl;
    std::cout << "最近障碍物位置: (" << obstacle_pos(0) << ", " << obstacle_pos(1) << ", " << obstacle_pos(2) << ")" << std::endl;
    std::cout << "计算得到的距离: " << distance << std::endl;
    std::cout << "查询点距离: " << distance << " (理论值: " << theoretical_distance << ")" << std::endl;

    return 0;
}

// #include "ESDFMap.h"
// #include "parameters.h"
// #include <vector>
// #include <iostream>
// #include <Eigen/Eigen>
// #include <cassert>
// #include <algorithm>

// using std::cout;
// using std::endl;
// using std::vector;
// using namespace fiesta;

// int main() {
//     // 初始化 ESDFMap
//     Eigen::Vector3d origin(0.0, 0.0, 0.0);
//     double resolution = 0.1;
//     Eigen::Vector3d map_size(10.0, 10.0, 10.0);
//     Parameters parameters_;
//     parameters_.SetParameters();
// #ifdef HASH_TABLE
//     fiesta::ESDFMap esdf_map(origin, resolution, 1000);
// #else
//     fiesta::ESDFMap esdf_map(origin, resolution, map_size);
// #endif

//     // 设置一些参数（如果需要）
// #ifdef PROBABILISTIC
//     esdf_map.SetParameters(0.7, 0.4, 0.1, 0.9, 0.5);
// //     esdf_map.SetParameters(parameters_.p_hit_, parameters_.p_miss_,
// //                            parameters_.p_min_, parameters_.p_max_, parameters_.p_occ_);
//  #endif

//     // 假设一个未占据的元素的位置
//     // Eigen::Vector3d unoccupied_pos(2.0, 2.0, 2.0);
//     Eigen::Vector3d unoccupied_pos(4.9, 4.9, 4.9);
//     // 假设最近障碍物的位置
//     Eigen::Vector3d obstacle_pos(5.0, 5.0, 5.0);

//     // 将障碍物位置设置为占据
//     esdf_map.SetOccupancy(obstacle_pos, 1);
//       // 将障碍物位置设置为占据
//     esdf_map.SetOccupancy(unoccupied_pos, 0);
    
    
//     //Eigen::Vector3d min_pos, Eigen::Vector3d max_pos
//      esdf_map.SetUpdateRange(origin,map_size);

//     // 检查并更新地图
//     if (esdf_map.CheckUpdate()) {
//         esdf_map.SetOriginalRange();
//         esdf_map.UpdateOccupancy(parameters_.global_update_);
//         esdf_map.UpdateESDF();
//     }

//     // 计算未占据元素到最近障碍物的距离
//     double distance = esdf_map.GetDistance(unoccupied_pos);
//     int occupancy = esdf_map.GetOccupancy(obstacle_pos);

//     // 输出结果
//     std::cout << "未占据元素位置: (" << unoccupied_pos(0) << ", " << unoccupied_pos(1) << ", " << unoccupied_pos(2) << ")" << std::endl;
//     std::cout << "最近障碍物位置: (" << obstacle_pos(0) << ", " << obstacle_pos(1) << ", " << obstacle_pos(2) << ")" << "occupancy =" << occupancy << std::endl;
//     std::cout << "计算得到的距离: " << distance << std::endl;
//      std::cout << "查询点距离: " << distance << " (理论值: " 
//                << (unoccupied_pos - obstacle_pos).norm() << ")\n";

//     return 0;
// }


// // #include "ESDFMap.h"
// // #include "parameters.h"
// // #include <iostream>
// // #include <Eigen/Eigen>

// // using namespace fiesta;

// // int main() {

// // while(1)
// // {
// //     // 地图参数配置
// //     const double resolution = 0.1;
// //     const Eigen::Vector3d map_size(10.0, 10.0, 10.0);
// //     const Eigen::Vector3d origin(0.0, 0.0, 0.0);
// //     Parameters parameters_;
// //     parameters_.SetParameters();
// //     // 初始化地图
// // #ifdef HASH_TABLE
// //     ESDFMap esdf_map(origin, resolution, 1000);  // 哈希表模式
// // #else
// //     ESDFMap esdf_map(origin, resolution, map_size);
// //      // 网格模式
// // #endif
// //     esdf_map.SetParameters(0.7, 0.4, 0.1, 0.9, 0.5);

// //     // 设置障碍物坐标（确保在网格范围内）
// //     const Eigen::Vector3d obstacle_world(5.0, 5.0, 5.0);
    
// //     // 关键修改：世界坐标转体素坐标
// //     Eigen::Vector3i obstacle_vox;
// //     esdf_map.Pos2Vox(obstacle_world,obstacle_vox);
// //     esdf_map.SetOccupancy(obstacle_vox, 1);  // 使用体素坐标接口

// //     // 强制更新ESDF
// //     esdf_map.UpdateOccupancy(true);
// //     esdf_map.UpdateESDF();

// //     // 查询点同样需要坐标转换
// //     const Eigen::Vector3d query_world(2.0, 2.0, 2.0);
// //     Eigen::Vector3i query_vox;
// //     esdf_map.Pos2Vox(query_world,query_vox);
    
// //     // 获取符号距离
// //     double distance = esdf_map.GetDistance(query_vox); // 使用体素坐标接口

// //     // 调试输出
// //     std::cout << "障碍物体素坐标: " << obstacle_vox.transpose() 
// //               << " 是否占据: " << esdf_map.GetOccupancy(obstacle_vox) << "\n";
// //     std::cout << "查询点距离: " << distance << " (理论值: " 
// //               << (obstacle_world - query_world).norm() << ")\n";
// // }
// //     return 0;
// // }