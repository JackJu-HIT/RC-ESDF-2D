/**
 * @file      main.cpp
 * @brief     Testing RC-ESDF: Robo-Centric 2D Signed Distance Field.
 * @author    juchunyu <juchunyu@qq.com>
 * @date      2026-02-15 10:00:01 
 * @copyright Copyright (c) 2025-2026 Institute of Robotics Planning and Control (IRPC). 
 *            All rights reserved.
*/
#include "rc_esdf.h"
#include <iostream>
#include <vector>
#include <chrono> 

int main() 
{
    // 1. 定义机器人形状 (Body Frame)
    std::vector<Eigen::Vector2d> footprint;
    footprint.push_back(Eigen::Vector2d(0.5, 0.3));
    footprint.push_back(Eigen::Vector2d(-0.5, 0.3));
    footprint.push_back(Eigen::Vector2d(-0.5, -0.3));
    footprint.push_back(Eigen::Vector2d(0.5, -0.3));
  
    // 2. 初始化并生成 RC-ESDF
    RcEsdfMap rc_map;
    rc_map.initialize(10.0, 10.0, 0.1); 
    rc_map.generateFromPolygon(footprint);

    std::vector<Eigen::Vector2d> obs_points_body;
    for(int i = 0; i < 100;i++)
    {
        obs_points_body.push_back(Eigen::Vector2d(0.0, 0.0));
        obs_points_body.push_back(Eigen::Vector2d(0.4, 0.2));
        obs_points_body.push_back(Eigen::Vector2d(0.6, 0.6));
        obs_points_body.push_back(Eigen::Vector2d(1, 1));
    }

    std::cout << "obs_points_body size: " << obs_points_body.size() << std::endl;

    std::cout << "--- Query Test ---" << std::endl;

    // [新增] 开始计时点
    auto start_time = std::chrono::high_resolution_clock::now();

    for (const auto& p : obs_points_body) {
        double dist;
        Eigen::Vector2d grad;
        
        // 核心查询函数
        bool in_box = rc_map.query(p, dist, grad);

        // 为了演示输出结果（注意：IO操作会显著增加耗时，测纯算法性能时建议注释掉打印）
        std::cout << "Point: (" << p.x() << ", " << p.y() << ") ";
        if (!in_box) {
            std::cout << "-> Out of Box (Safe)" << std::endl;
        } else {
            std::cout << "-> Dist: " << dist 
                      << " | Grad: (" << grad.x() << ", " << grad.y() << ")" << std::endl;
            
            if (dist < 0) {
                std::cout << "   [COLLISION] Push robot direction: (" 
                          << -grad.x() << ", " << -grad.y() << ")" << std::endl;
            }
        }
    }

    //结束计时点
    auto end_time = std::chrono::high_resolution_clock::now();
    
    //计算耗时
    std::chrono::duration<double, std::milli> elapsed_ms = end_time - start_time;
    std::cout << "obs_points_body size: " << obs_points_body.size() << std::endl;

    std::cout << "------------------------------------------------" << std::endl;
    std::cout << "Total Loop Time: " << elapsed_ms.count() << " ms" << std::endl;
    std::cout << "Avg Time per Point: " << elapsed_ms.count() / obs_points_body.size() << " ms" << std::endl;

    rc_map.visualizeEsdf(footprint);

    return 0;
}