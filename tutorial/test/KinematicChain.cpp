#pragma once

#include <Eigen/Core>
#include "Model.h"

#define M_PI       3.14159265358979323846 

static Eigen::Vector3f getTipPosition(int cyl, float offset, const std::vector<std::shared_ptr<cg3d::Model>>& cyls)
{
    Eigen::Vector3f center = cyls[cyl]->GetAggregatedTransform().col(3).head(3);
    Eigen::Vector3f translation{ 0, 0, offset };
    Eigen::Vector3f tipPosition = center + cyls[cyl]->GetRotation() * translation;
    return tipPosition;
}

static void getZXZRotationMatrices(const Eigen::Matrix3f &rotation,
                                    Eigen::Matrix3f &phiZ,
                                    Eigen::Matrix3f &thetaX,
                                    Eigen::Matrix3f &psiZ)
{
    Eigen::Vector3f angles = rotation.eulerAngles(2, 0, 2);
    std::cerr << "rotation.eulerAngles(2, 0, 2):\n" << 180.0f/M_PI * angles << std::endl;
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

static void rotateInZXZ(const Eigen::Matrix3f& rotation,
                        const Eigen::Vector3f& rotationAngles,
                        Eigen::Matrix3f& newRotation)
{
    Eigen::Vector3f rotationsEulerAngles = rotation.eulerAngles(2, 0, 2);
    Eigen::Vector3f test1 = rotationsEulerAngles;
    auto tz1 = Eigen::AngleAxisf(test1[0], Eigen::Vector3f::UnitZ());
    auto tx = Eigen::AngleAxisf(test1[1], Eigen::Vector3f::UnitX());
    auto tz2 = Eigen::AngleAxisf(test1[2], Eigen::Vector3f::UnitZ());

    rotationsEulerAngles = rotationsEulerAngles + rotationAngles;
    
    auto rotationInZ0 = Eigen::AngleAxisf(rotationsEulerAngles[0], Eigen::Vector3f::UnitZ());
    auto rotationInX = Eigen::AngleAxisf(rotationsEulerAngles[1], Eigen::Vector3f::UnitX());
    auto rotationInZ2 = Eigen::AngleAxisf(rotationsEulerAngles[2], Eigen::Vector3f::UnitZ());
    newRotation = rotationInZ0 * rotationInX * rotationInZ2;
}

static void rotateBasedOnEulerAngles(const std::vector<std::shared_ptr<cg3d::Model>>& cyls,
                                     int index,
                                     const Eigen::Vector3f angles)
{
    Eigen::Matrix3f newRotation;
    rotateInZXZ(cyls[index]->GetTout().rotation(), angles, newRotation);
    Eigen::Affine3f newTout{ Eigen::Affine3f::Identity() };
    newTout.pretranslate(cyls[index]->GetTout().translation());
    newTout.rotate(newRotation);
    cyls[index]->SetTout(newTout);
}

static bool cyclicCoordinateDescent(const std::vector<std::shared_ptr<cg3d::Model>>& cyls,
                                    const std::vector<std::shared_ptr<cg3d::Model>>& axis,
                                    Eigen::Vector3f& dest, const float delta, int& index,
                                    const std::shared_ptr<cg3d::Movable>& root)
{
    if (index == -1) index = (int)cyls.size() - 1;
    // Calculate R and E
    float startOfCylOffset = -0.8f, tipOfCylOffset = 0.8f;
    Eigen::Vector3f R = getTipPosition(index, startOfCylOffset, cyls); // Start of current moving cylinder
    Eigen::Vector3f E = getTipPosition((int)cyls.size() - 1, tipOfCylOffset, cyls); // Tip of the arm
    Eigen::Vector3f RE = (E - R).normalized();
    Eigen::Vector3f RD = (dest - R).normalized();
    Eigen::Vector3f normal = RE.cross(RD);
    float dot = RD.dot(RE);
    if (abs(dot) > 1) dot = 1.0f;
    float theta = acos(dot) / 10;

    Eigen::Vector3f rotateAroundVector = cyls[index]->GetAggregatedTransform().block<3, 3>(0, 0).inverse() * normal;
    cyls[index]->Rotate(theta, rotateAroundVector);

    // Calculate new delta to check if should stop moving
    E = getTipPosition((int)cyls.size() - 1, tipOfCylOffset, cyls);
    Eigen::Vector3f DE = dest - E;
    float distance = abs(DE.norm());
    if (distance < delta)
    {
        index = 0;
        return false;
    }

    index--;
    return true;
}

// Functions for testing purposes:

static bool inGimbalLock(Eigen::Matrix3f rotation)
{
    return rotation(0, 0) == 0 &&
        rotation(0, 1) == 0 &&
        rotation(0, 2) == 1 &&
        rotation(1, 2) == 0 &&
        rotation(2, 2) == 0;
}

static void printAllTips(const std::vector<std::shared_ptr<cg3d::Model>>& cyls)
{
    for (int i = 0; i < cyls.size(); i++)
        std::cout << "Cyl " << i << "\n" << getTipPosition(i, 0.8, cyls) << std::endl;
}