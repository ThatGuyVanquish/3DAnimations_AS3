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

static void calcTipsPosition(const Eigen::Vector4f& initial_tip_pos,
                             std::vector<Eigen::Vector4f>& tips_position,
                             const std::shared_ptr<cg3d::Movable>& root,
                             const std::vector<std::shared_ptr<cg3d::Model>>& cyls)
{
    tips_position[0] = /*root->GetAggregatedTransform().inverse() * */ cyls[0]->GetAggregatedTransform() * Eigen::Vector4f(0, 0, -0.8, 1);
    for (int i = 1; i < tips_position.size(); i++)
    {
        tips_position[i] = /*root->GetAggregatedTransform().inverse() * */ cyls[i-1]->GetAggregatedTransform() * initial_tip_pos;
    }
}

static bool inGimbalLock(Eigen::Matrix3f rotation)
{
    return rotation(0,0) == 0 &&
           rotation(0,1) == 0 &&
           rotation(0,2) == 1 &&
           rotation(1,2) == 0 &&
           rotation(2,2) == 0;
}

static bool cyclicCoordinateDescent(std::vector<std::shared_ptr<cg3d::Model>> cyls,
                                    std::vector<std::shared_ptr<cg3d::Model>> axis,
                                    std::vector<Eigen::Vector4f> tips,
                                    Eigen::Vector3f destination, const float delta, int &index,
                                    const std::shared_ptr<cg3d::Movable>& root)
{
    calcTipsPosition({ 0,0,0.8f,1 }, tips, root, cyls);
    if (index == -1) index = cyls.size() - 1;
    std::cout << "index is " << index << std::endl;
    // Calculate R and E
    Eigen::Vector3f R = tips[index].head(3);
    Eigen::Vector3f E = tips[cyls.size()].head(3);
    Eigen::Vector3f RE = (E - R);//.normalized();
    Eigen::Vector3f RD = (destination - R);//.normalized();
    std::shared_ptr<cg3d::Movable> parent = index == 0 ? root : cyls[index-1];

    std::cout << "RE before is " << RE.transpose() << std::endl;
    std::cout << "RD before is " << RD.transpose() << std::endl;

    RE = (parent->GetAggregatedTransform() * Eigen::Vector4f({RE[0], RE[1], RE[2], 1})).head(3);
    RD = (parent->GetAggregatedTransform() * Eigen::Vector4f({RD[0], RD[1], RD[2], 1})).head(3);

    std::cout << "RE after is " << RE.transpose() << std::endl;
    std::cout << "RD after is " << RD.transpose() << std::endl;

//    int dot = RD.normalized().dot(RE.normalized());
//    float a = acos(dot);
//    Eigen::Vector3f normal = RE.normalized().cross(RD.normalized());
//    cyls[index]->Rotate(a/10.0f, normal);

    Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(RE, RD);

    Eigen::AngleAxisf from_q;
    from_q = q;

    std::cout << "before slerp AngleAxis angle:\n" << from_q.angle() * (180.0/3.141592653589793238463) << std::endl;
    std::cout << "before slerp AngleAxis axis:\n" << from_q.axis() << std::endl;

//    q = q.slerp(0.9f, Eigen::Quaternionf::Identity());

    from_q = q;
    std::cout << "after slerp AngleAxis angle:\n" << from_q.angle() * (180.0/3.141592653589793238463) << std::endl;
    std::cout << "after slerp AngleAxis axis:\n" << from_q.axis()  << std::endl;

    cyls[index]->Rotate(q);
//    cyls[index]->Rotate(from_q.angle(), from_q.axis());

    if (inGimbalLock(cyls[index]->GetRotation()))
    {
        std::cout << "inGimbalLock" << std::endl;
    }
    // calculate new delta
    calcTipsPosition({ 0,0,0.8f,1 }, tips, root, cyls);
    E = tips[cyls.size()].head(3);
    Eigen::Vector3f DE = destination - E;
    std::cout << "distance is " << DE.norm() << std::endl;
    if (DE.norm() < delta)
    {
        index = cyls.size() - 1;
        return false;
    }
    index--;
//    return true;
    // for testing each step
    return false;
}

