#pragma once

#include "Scene.h"
#include "IglMeshLoader.h"
#include "Renderer.h"
#include "KinematicChain.cpp"
#include <PickVisitor.h>
#include <memory>
#include <utility>
#include <iomanip>

#define M_PI_2     1.57079632679489661923   // pi/2

class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;

    void nextCyclicDescentStep();
    bool doCyclicDescent = false;

private:
    std::shared_ptr<cg3d::Movable> root;
    std::shared_ptr<cg3d::Model> sphere;
    std::vector<std::shared_ptr<cg3d::Model>> cyls, axis;
    std::shared_ptr<cg3d::Model> GLOBAL_AXIS;
    float scaleFactor = 1;
    int numOfCyls = 4;
    const Eigen::Vector3f INITIAL_SPHERE_POS = { 5, 0, 0 };
    const float DELTA = 0.5f;

    int pickedIndex = -1;
    int cylinderToMove;

    std::shared_ptr<cg3d::Program> phongShader;
    std::shared_ptr<cg3d::Program> pickingShader;
    std::shared_ptr<cg3d::Material> material;
    std::shared_ptr<cg3d::Material> material1;
    std::shared_ptr<cg3d::Material> material2;
    std::shared_ptr<cg3d::Mesh> sphereMesh;
    std::shared_ptr<cg3d::Mesh> cylMesh;
    std::shared_ptr<cg3d::Mesh> coordsys;

    void initialSettings();
    void initializeBaseOfArm();
    void initializeRestOfArm();
    void reset();
    void emptyCylinderVectors();
   
    Eigen::Vector3f getSpherePosition();
    bool isReachable();

    void MouseCallback(cg3d::Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[]) override;
    void pickedACylinder(int index);

    void ScrollCallback(cg3d::Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[]) override;

    void CursorPosCallback(cg3d::Viewport* viewport, int x, int y, bool dragging, int* buttonState)  override;

    void KeyCallback(cg3d::Viewport* viewport, int x, int y, int key, int scancode, int action, int mods) override;

};
