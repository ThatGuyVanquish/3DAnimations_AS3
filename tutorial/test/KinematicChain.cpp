#pragma once

#include <Eigen/Core>
#include "Model.h"

#define M_PI       3.14159265358979323846

static bool inGimbalLock(Eigen::Matrix3f rotation);

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

static bool cyclicCoordinateDescentStep(const std::vector<std::shared_ptr<cg3d::Model>>& cyls,
                                    Eigen::Vector3f& dest, const float delta, int& index)
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
        std::cout << "distance at point of reach: " << distance << std::endl;
        index = 0;
        return false;
    }

    index--;
    return true;
}

static bool fabric(const std::vector<std::shared_ptr<cg3d::Model>>& cyls,
                   Eigen::Vector3f& dest, const float delta, const float &length) {
    float startOfCylOffset = -0.8f, tipOfCylOffset = 0.8f;
    std::vector<Eigen::Vector3f> p;
    std::vector<float> d;

    // calculate joint positions
    p.push_back(getTipPosition(0, startOfCylOffset, cyls));
    for (int i = 0; i < cyls.size(); i++) {
        p.push_back(getTipPosition(i, tipOfCylOffset, cyls));
    }

    // calculate joint distances
    for (int i = 0; i < p.size() - 1; i++) {
        d.push_back((p[i + 1] - p[i]).norm());
    }

    // calculate distance
    float dist = (dest - p[0]).norm();
    float sum_of_elems = std::accumulate(d.begin(), d.end(), 0.0);

    // set as b the initial position of the joint p[0]
    Eigen::Vector3f b = p[0];

    //  Check whether the distance between the end effector p[n] and dest is greater than delta
    float dif = (dest - p[p.size() - 1]).norm();
    while (dif > delta) {
        // STAGE 1: FORWARD REACHING

        // Set the end effector p[n] as dest
        p[p.size() - 1] = dest;

        for (int i = p.size() - 2; i >= 0; i--) {
            //  Find the distance ri between the new joint position p[i+1] and the joint p[i]
            float ri = (p[i + 1] - p[i]).norm();
            float lambda_i = d[i] / ri;

            // Find the new joint positions p[i]
            p[i] = (1 - lambda_i) * p[i + 1] + lambda_i * p[i];
        }

        // STAGE 2: BACKWARD REACHING

        // Set the root p[0] its initial position.
        p[0] = b;

        for (int i = 0; i < p.size() - 2; i++) {
            //  Find the distance ri between the new joint position p[i] and the joint p[i+1]
            float ri = (p[i + 1] - p[i]).norm();
            float lambda_i = d[i] / ri;

            // Find the new joint positions p[i+1]
            p[i + 1] = (1 - lambda_i) * p[i] + lambda_i * p[i + 1];
        }

        dif = (dest - p[p.size() - 1]).norm();
    }

    // transform cyls based on new positions
    for (int i = p.size() - 1; i > 0; i--) {
        //Rotation
        Eigen::Vector3f d;
        if (i != 1) {
            d = (getTipPosition(i - 1, tipOfCylOffset, cyls) -
                 getTipPosition(i - 2, tipOfCylOffset, cyls)).normalized();
        } else {
            d = (getTipPosition(i - 1, tipOfCylOffset, cyls) -
                 getTipPosition(i - 1, startOfCylOffset, cyls)).normalized();
        }
        Eigen::Vector3f l = (p[i] - p[i - 1]).normalized();
        Eigen::Vector3f normal = d.cross(l);
        float dot = d.dot(l);
        if (abs(dot) > 1) dot = 1.0f;
        float theta = acos(dot) / 10;
        Eigen::Vector3f rotateAroundVector = cyls[i - 1]->GetAggregatedTransform().block<3, 3>(0, 0).inverse() * normal;
        cyls[i - 1]->Rotate(theta, rotateAroundVector);

        //Translation
        Eigen::Vector3f trans = p[i] - getTipPosition(i - 1, tipOfCylOffset, cyls);
        cyls[i - 1]->Translate(trans);
    }

    return false;
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