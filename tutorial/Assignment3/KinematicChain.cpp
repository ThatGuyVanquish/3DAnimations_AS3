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
    Eigen::Quaternionf q(rotation);
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

static void calcTipsPosition(const Eigen::Vector4f& initial_tip_pos,
    std::vector<Eigen::Vector4f>& tips_position,
    const std::shared_ptr<cg3d::Movable>& root,
    const std::shared_ptr<cg3d::Movable>& armRoot,
    const std::vector<std::shared_ptr<cg3d::Model>>& cyls)
{
    tips_position[0] = root->GetAggregatedTransform().inverse() * armRoot->GetAggregatedTransform() * Eigen::Vector4f(0, 0, 0, 1);
    tips_position[1] = root->GetAggregatedTransform().inverse() * cyls[0]->GetAggregatedTransform() * initial_tip_pos;
    tips_position[2] = root->GetAggregatedTransform().inverse() * cyls[1]->GetAggregatedTransform() * initial_tip_pos;
    tips_position[3] = root->GetAggregatedTransform().inverse() * cyls[2]->GetAggregatedTransform() * initial_tip_pos;
}

static bool cyclicCoordinateDescent(std::vector<std::shared_ptr<cg3d::Model>> cyls, 
                                    std::vector<Eigen::Vector4f> tips,
                                    Eigen::Vector3f destination, const float delta, int &index,
                                    const std::shared_ptr<cg3d::Movable>& root,
                                    const std::shared_ptr<cg3d::Movable>& armRoot
                                    /*,Eigen::Quaternionf &q*/)
{
    if (index == -1) index = cyls.size() - 1;
    // Calculate R and E
    Eigen::Vector3f R = tips[index].head(3);
    Eigen::Vector3f E = tips[index + 1].head(3);
    Eigen::Vector3f RE = E - R;
    Eigen::Vector3f RD = destination - R;
    //float a = acos(RD.dot(RE));
    Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(RE, RD);
    q = q.slerp(0.9f, Eigen::Quaternionf::Identity());
    cyls[index]->Rotate(q);

    // calculate new delta
    calcTipsPosition({ 0,0,0.81f }, tips, root, armRoot, cyls);
    E = tips[index + 1].head(3);
    Eigen::Vector3f DE = destination - E;
    if (DE.norm() < delta) 
    {
        index = cyls.size() - 1;
        return false;
    }
    index--;
    return true;
}