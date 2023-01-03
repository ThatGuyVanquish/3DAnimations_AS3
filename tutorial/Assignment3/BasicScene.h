#pragma once
#include "AutoMorphingModel.h"
#include "Scene.h"
#include "KinematicChain.cpp"
#include <memory>
#include <utility>

class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void initialSetting();
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;
    void MouseCallback(cg3d::Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[]) override;
    void ScrollCallback(cg3d::Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[]) override;
    void CursorPosCallback(cg3d::Viewport* viewport, int x, int y, bool dragging, int* buttonState)  override;
    void KeyCallback(cg3d::Viewport* viewport, int x, int y, int key, int scancode, int action, int mods) override;
    void nextCyclicDescentStep();
    Eigen::Vector3f GetSpherePos();
    bool isReachable();
    void reset();
    int previousMovingCyl;
    bool doCyclicDescent = false;

private:
    std::shared_ptr<Movable> root;
//    std::shared_ptr<Movable> armRoot;
    std::shared_ptr<cg3d::Model> sphere1;
    std::vector<std::shared_ptr<cg3d::Model>> cyls, axis;
    std::shared_ptr<cg3d::Model> global_axis;
    int pickedIndex = 0;
    Eigen::Vector4f initial_tip_pos = Eigen::Vector4f(0,0,0.8f,1);
    float scaleFactor;
    Eigen::Vector3f spherePos;
    int numOfCyls = 4;
    const float INITIAL_SPHERE_POS = 5;
    const float DELTA = 0.05;
};
