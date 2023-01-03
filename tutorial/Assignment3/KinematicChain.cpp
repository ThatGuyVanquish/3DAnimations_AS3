//
// Created by Lior Levi on 25/12/2022.
//

#pragma once

#include "Eigen/Geometry"
#include "Model.h"
#include <Eigen/Core>
#include <unsupported/Eigen/EulerAngles>
#include <iostream>
#define M_PI       3.14159265358979323846   // pi

static void getZXZRotationMatrices(const Eigen::Matrix3f &rotation,
                            Eigen::Matrix3f &phiZ,
                            Eigen::Matrix3f &thetaX,
                            Eigen::Matrix3f &psiZ)
{
    Eigen::Vector3f angles = rotation.eulerAngles(2, 0, 2);
    std::cout << "rotation.eulerAngles(2, 0, 2):\n" << 180.0f/M_PI * angles << std::endl;
    float phi = angles[0];
    float theta = angles[1];
    float psi = angles[2];

    phiZ <<
            cos(phi), -sin(phi), 0,
            sin(phi), cos(phi), 0,
            0, 0, 1;
    thetaX <<
            1, 0, 0,
            0, cos(theta), -sin(theta),
            0, sin(theta), cos(theta);
    psiZ <<
            cos(psi), -sin(psi), 0,
            sin(psi), cos(psi), 0,
            0, 0, 1;
}

static void rotateInX(const Eigen::Matrix3f &rotation,
                      const float &angle,
                      Eigen::Matrix3f &phiZ,
                      Eigen::Matrix3f &thetaX,
                      Eigen::Matrix3f &psiZ)
{
    Eigen::Vector3f angles = rotation.eulerAngles(2, 0, 2);
    std::cout << "rotation.eulerAngles(2, 0, 2):\n" << 180.0f/M_PI * angles << std::endl;
    float phi = angles[0];
    float theta = angles[1] + angle;
    float psi = angles[2];
    phiZ <<
         cos(phi), -sin(phi), 0,
            sin(phi), cos(phi), 0,
            0, 0, 1;
    thetaX <<
           1, 0, 0,
            0, cos(theta), -sin(theta),
            0, sin(theta), cos(theta);
    psiZ <<
         cos(psi), -sin(psi), 0,
            sin(psi), cos(psi), 0,
            0, 0, 1;
}

static bool inGimbalLock(Eigen::Matrix3f rotation)
{
    return rotation(0,0) == 0 &&
           rotation(0,1) == 0 &&
           rotation(0,2) == 1 &&
           rotation(1,2) == 0 &&
           rotation(2,2) == 0;
}

static Eigen::Vector3f getTipPosition(int cyl, float offset, const std::vector<std::shared_ptr<cg3d::Model>>& cyls)
{
    Eigen::Vector3f center = cyls[cyl]->GetAggregatedTransform().col(3).head(3);
    Eigen::Vector3f translation{ 0, 0, offset };
    Eigen::Vector3f tipPosition = center + cyls[cyl]->GetRotation() * translation;
    return tipPosition;
}

static bool cyclicCoordinateDescent(const std::vector<std::shared_ptr<cg3d::Model>> &cyls,
                                    const std::vector<std::shared_ptr<cg3d::Model>> &axis,
                                    Eigen::Vector3f destination, const float delta, int &index,
                                    const std::shared_ptr<cg3d::Movable>& root)
{
    if (index == -1) index = cyls.size() - 1;
    std::cout << "index is " << index << std::endl;
    // Calculate R and E
    Eigen::Vector3f R = getTipPosition(index, -0.8f, cyls);
    Eigen::Vector3f E = getTipPosition(cyls.size() - 1, 0.8f, cyls);
    Eigen::Vector3f RE = (E - R).normalized();
    Eigen::Vector3f RD = (destination - R).normalized();
    
    Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(RE, RD);
    //q = q.slerp(0.9f, Eigen::Quaternionf::Identity());

    cyls[index]->Rotate(q);

    // calculate new delta
    E = getTipPosition(0, 0.8f, cyls);
    Eigen::Vector3f DE = destination - E;
    std::cout << "distance is " << DE.norm() << std::endl;
    if (DE.norm() < delta)
    {
        index = 0;
        return false;
    }
    index--;
//    return true;
    // for testing each step
    return false;
}

static void printAllTips(const std::vector<std::shared_ptr<cg3d::Model>>& cyls)
{
    for (int i = 0; i < cyls.size(); i++)
        std::cout << "Cyl " << i << "\n" << getTipPosition(i, 0.8, cyls) << std::endl;
}