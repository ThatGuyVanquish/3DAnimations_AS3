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
//    float phi = atan2(rotation(2, 1), rotation(2, 2));
//    float theta = acos(rotation(2, 0));
//    float psi = atan2(rotation(1, 0), rotation(0, 0));
//    std::cout << "eulerAngles from calc:\n" << 180.0f/M_PI * phi << " " << 180.0f/M_PI * theta << " " << 180.0f/M_PI * psi << std::endl;
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

static Eigen::Vector3f calculateEndPoint(Eigen::Vector3f start, float length, float angleX, float angleZ)
{
    Eigen::Vector3f endPoint;
    endPoint << start[0] + length * sin(angleZ) * cos(angleX),
                start[1] + length * sin(angleZ) * sin(angleX),
                start[2] + length * cos(angleZ);
}

static Eigen::Vector3f quaternion2EulerAngles(const Eigen::Quaternionf &q)
{
    float test = q.x()*q.y() + q.z()*q.w();
    if (test > 0.499)
    {

    }
}

static void printTransform(const Eigen::Matrix4f &transform, const Eigen::Matrix3f &rotation)
{
    Eigen::Vector3f u, v, w, c;
    Eigen::RowVector3f init_tip(0,0,1.6f);
    u = Eigen::RowVector3f(1,0,0) * rotation;
    v = Eigen::RowVector3f(0,1,0) * rotation;
    w = Eigen::RowVector3f(0,0,1) * rotation;
    c = init_tip * rotation;
    std::cout << "transformation using u, v, w, c:\n" << u.x() << " " << v.x() << " " << w.x() << " 0" << "\n"
              << u.y() << " " << v.y() << " " << w.y() << " 0" << "\n"
              << u.z() << " " << v.z() << " " << w.z() << " 0" << "\n"
              << (-1 * c).dot(u) << " " << (-1 * c).dot(v) << " " <<(-1 * c).dot(w) << " 1" << std::endl;
    std::cout << "transformation:\n" << transform << std::endl;
}

static Eigen::Vector3f getAxis(const Eigen::Matrix3f &rotation, Eigen::Vector3f &globalAxis)
{
    return rotation.transpose() * globalAxis;
}

static float getAngleWithAxis(const Eigen::Matrix3f &rotation, Eigen::Vector3f globalAxis)
{
    return acos(getAxis(rotation, globalAxis).dot(globalAxis));
}

static void cyclicCoordinateDescent(std::vector<std::shared_ptr<cg3d::Model>> cyls, Eigen::Vector3f destination, float delta)
{

}

static void calcTipsPosition(const Eigen::Vector4f &initial_tip_pos,
                             std::vector<Eigen::Vector4f> &tips_position,
                             const std::shared_ptr<cg3d::Movable> &root,
                             const std::shared_ptr<cg3d::Movable> &armRoot,
                             const std::vector<std::shared_ptr<cg3d::Model>> &cyls)
{
    tips_position[0] = root->GetAggregatedTransform().inverse() * armRoot->GetAggregatedTransform() * Eigen::Vector4f(0,0,0,1);
    tips_position[1] = root->GetAggregatedTransform().inverse() * cyls[0]->GetAggregatedTransform() * initial_tip_pos;
    tips_position[2] = root->GetAggregatedTransform().inverse() * cyls[1]->GetAggregatedTransform() * initial_tip_pos;
    tips_position[3] = root->GetAggregatedTransform().inverse() * cyls[2]->GetAggregatedTransform() * initial_tip_pos;
}