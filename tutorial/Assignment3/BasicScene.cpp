#include "BasicScene.h"
#include <Eigen/src/Core/Matrix.h>
#include <edges.h>
#include <memory>
#include <per_face_normals.h>
#include <read_triangle_mesh.h>
#include <utility>
#include <vector>
#include "GLFW/glfw3.h"
#include "Mesh.h"
#include "PickVisitor.h"
#include "Renderer.h"
#include "ObjLoader.h"
#include "IglMeshLoader.h"

#include "igl/per_vertex_normals.h"
#include "igl/per_face_normals.h"
#include "igl/unproject_onto_mesh.h"
#include "igl/edge_flaps.h"
#include "igl/loop.h"
#include "igl/upsample.h"
#include "igl/AABB.h"
#include "igl/parallel_for.h"
#include "igl/shortest_edge_and_midpoint.h"
#include "igl/circulation.h"
#include "igl/edge_midpoints.h"
#include "igl/collapse_edge.h"
#include "igl/edge_collapse_is_valid.h"
#include "igl/write_triangle_mesh.h"
#include <iomanip>

#define M_PI_2     1.57079632679489661923   // pi/2
// #include "AutoMorphingModel.h"

using namespace cg3d;

const int DISPLAY_WIDTH = 800;
const int DISPLAY_HEIGHT = 800;
const float CAMERA_ANGLE = 45.0f;
const float NEAR = 0.1f;
const float FAR = 120.0f;

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
    camera = Camera::Create( "camera", fov, float(width) / height, near, far);
    
    AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
    auto daylight{std::make_shared<Material>("daylight", "shaders/cubemapShader")}; 
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{Model::Create("background", Mesh::Cube(), daylight)};
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();

    auto program = std::make_shared<Program>("shaders/phongShader");
    auto program1 = std::make_shared<Program>("shaders/pickingShader");
    
    auto material{ std::make_shared<Material>("material", program)}; // empty material
    auto material1{ std::make_shared<Material>("material", program1)}; // empty material

    material->AddTexture(0, "textures/box0.bmp", 2);
    auto sphereMesh{IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj")};
    auto cylMesh{IglLoader::MeshFromFiles("cyl_igl","data/zCylinder.obj")};
    sphere1 = Model::Create( "sphere",sphereMesh, material);

    // a common invisible parent for the multi-links arm and axis
    armRoot = Movable::Create("armRoot");
    root->AddChild(armRoot);

    // scale factor for the multi-links arm
    scaleFactor = 1;

    // global Axis (for sanity checks)
    Eigen::MatrixXd vertices(6,3);
    vertices << -1,0,0,1,0,0,0,-1,0,0,1,0,0,0,-1,0,0,1;
    Eigen::MatrixXi faces(3,2);
    faces << 0,1,2,3,4,5;
    Eigen::MatrixXd vertexNormals = Eigen::MatrixXd::Ones(6,3);
    Eigen::MatrixXd textureCoords = Eigen::MatrixXd::Ones(6,2);
    std::shared_ptr<Mesh> coordsys = std::make_shared<Mesh>("coordsys",vertices,faces,vertexNormals,textureCoords);
    global_axis = Model::Create("global axis",coordsys,material1);
    global_axis->mode = 1;
    global_axis->Scale(4,Axis::XYZ);
//    root->AddChild(global_axis);
    AddChild(global_axis);

    // cylinders and their axis
    for (int i = 0; i < numOfCyls; i++)
    {
        axis.push_back(Model::Create("axis" + std::to_string(i), coordsys, material1));
        cyls.push_back(Model::Create("cyl" + std::to_string(i), cylMesh, material));
        if (i == 0)
        {
            armRoot->AddChild(axis[0]);
            armRoot->AddChild(cyls[0]);
        }
        if (i > 0) // dependency on previous cylinder axis
        {
            cyls[i - 1]->AddChild(cyls[i]);
            cyls[i - 1]->AddChild(axis[i]);
        }
    }

    // initial transformations for cylinders and their axis
    cyls[0]->Scale(scaleFactor,Axis::Z);
    cyls[0]->Translate({0,0,0.8f*scaleFactor});
    cyls[0]->SetCenter(Eigen::Vector3f(0,0,-0.8f*scaleFactor));
    axis[0]->mode = 1;
    axis[0]->Scale(2.0f*1.6f*scaleFactor);
    for(int i = 1; i < numOfCyls; i++)
    {
        cyls[i]->Scale(scaleFactor,Axis::Z);
        cyls[i]->Translate(1.6f*scaleFactor,Axis::Z);
        cyls[i]->SetCenter(Eigen::Vector3f(0,0,-0.8f*scaleFactor));
        // cylinder i axis system according to the previous cylinder
        axis[i]->mode = 1;
        axis[i]->Scale(2.0f*1.6f*scaleFactor);
        axis[i]->Translate(0.8f*scaleFactor,Axis::Z);
    }

    // rotate arm to align with Y axis

    // init tips position
    tips_position.push_back(armRoot->GetAggregatedTransform() * Eigen::Vector4f(0, 0, 0, 1));
    for (int i = 1; i < numOfCyls + 1; i++) {
        tips_position.push_back(cyls[i - 1]->GetAggregatedTransform() * initial_tip_pos);
    }
    
    //tips_position.push_back(cyls[0]->GetAggregatedTransform() * initial_tip_pos);
    //tips_position.push_back(cyls[1]->GetAggregatedTransform() * initial_tip_pos);
    //tips_position.push_back(cyls[2]->GetAggregatedTransform() * initial_tip_pos);

    // sphere
    sphere1->showWireframe = true;
    sphere1->Translate({INITIAL_SPHERE_POS, 0, 0});

    camera->Translate(50, Axis::X);
    camera->cg3d::Movable::Rotate((M_PI_2), Axis::Y);
    root->AddChild(sphere1);

    previousMovingCyl = cyls.size() - 1;
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 0.8f, 0.3f, 0.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 0.3f, 0.6f, 1.0f);
    program.SetUniform4f("Kdi", 0.5f, 0.5f, 0.0f, 1.0f);
    program.SetUniform1f("specular_exponent", 5.0);
    program.SetUniform4f("light_position", 0.0, 15.0, 0.0, 1.0);

    if (isReachable())
    {
        doCyclicDescent = false;
    }
}

void BasicScene::MouseCallback(Viewport* viewport, int x, int y, int button, int action, int mods, int buttonState[])
{
    // note: there's a (small) chance the button state here precedes the mouse press/release event

    if (action == GLFW_PRESS) { // default mouse button press behavior
        PickVisitor visitor;
        visitor.Init();
        renderer->RenderViewportAtPos(x, y, &visitor); // pick using fixed colors hack
        auto modelAndDepth = visitor.PickAtPos(x, renderer->GetWindowHeight() - y);
        renderer->RenderViewportAtPos(x, y); // draw again to avoid flickering
        pickedModel = modelAndDepth.first ? std::dynamic_pointer_cast<Model>(modelAndDepth.first->shared_from_this()) : nullptr;
        pickedModelDepth = modelAndDepth.second;
        camera->GetRotation().transpose();
        xAtPress = x;
        yAtPress = y;

        // if (pickedModel)
        //     debug("found ", pickedModel->isPickable ? "pickable" : "non-pickable", " model at pos ", x, ", ", y, ": ",
        //           pickedModel->name, ", depth: ", pickedModelDepth);
        // else
        //     debug("found nothing at pos ", x, ", ", y);

        if (pickedModel && !pickedModel->isPickable)
            pickedModel = nullptr; // for non-pickable models we need only pickedModelDepth for mouse movement calculations later

        if (pickedModel)
            pickedToutAtPress = pickedModel->GetTout();
        else
            cameraToutAtPress = camera->GetTout();
    }
}

void BasicScene::ScrollCallback(Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[])
{
    // note: there's a (small) chance the button state here precedes the mouse press/release event
    auto system = camera->GetRotation().transpose();
    if (pickedModel) {
        if(std::find(cyls.begin(), cyls.end(), pickedModel) != cyls.end())
        {
            armRoot->TranslateInSystem(system, {0, 0, -float(yoffset)});
            pickedToutAtPress = armRoot->GetTout();
        } else {
            pickedModel->TranslateInSystem(system, {0, 0, -float(yoffset)});
            pickedToutAtPress = pickedModel->GetTout();
        }
    } else {
        camera->TranslateInSystem(system, {0, 0, -float(yoffset)});
        cameraToutAtPress = camera->GetTout();
    }
}

void BasicScene::CursorPosCallback(Viewport* viewport, int x, int y, bool dragging, int* buttonState)
{
    if (dragging) {
        auto system = camera->GetRotation().transpose() * GetRotation();
        auto moveCoeff = camera->CalcMoveCoeff(pickedModelDepth, viewport->width);
        auto angleCoeff = camera->CalcAngleCoeff(viewport->width);
        if (pickedModel) {
            //pickedModel->SetTout(pickedToutAtPress);
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE)
                if(std::find(cyls.begin(), cyls.end(), pickedModel) != cyls.end())
                {
                    armRoot->TranslateInSystem(system, {-float(xAtPress - x) / moveCoeff, float(yAtPress - y) / moveCoeff, 0});
                } else {
                    pickedModel->TranslateInSystem(system, {-float(xAtPress - x) / moveCoeff, float(yAtPress - y) / moveCoeff, 0});
                }
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Y);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
                pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Z);
                pickedModel->RotateInSystem(system, float(yAtPress - y) / angleCoeff, Axis::X);
            }
        } else {
            // camera->SetTout(cameraToutAtPress);
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE)
                root->TranslateInSystem(system, {-float(xAtPress - x) / moveCoeff/10.0f, float( yAtPress - y) / moveCoeff/10.0f, 0});
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                root->RotateInSystem(system, float(x - xAtPress) / 180, Axis::Z);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
                root->RotateInSystem(system, float(x - xAtPress) / angleCoeff, Axis::Y);
                root->RotateInSystem(system, float(y - yAtPress) / angleCoeff, Axis::X);
            }
        }
        xAtPress =  x;
        yAtPress =  y;
    }
}

void BasicScene::KeyCallback(Viewport* viewport, int x, int y, int key, int scancode, int action, int mods)
{
    auto system = camera->GetRotation().transpose();

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) // NOLINT(hicpp-multiway-paths-covered)
        {
            case GLFW_KEY_ENTER:
                armRoot->Translate(Eigen::Vector3f(1.6f, 1.6f, 1.6f));
                break;
            case GLFW_KEY_SPACE:
                if (isReachable())
                {
                    std::cout << "cannot reach.\ndestination:\n" << spherePos.transpose()
                                << "\nstart tip position:\n" << tips_position[0].head(3).transpose()
                                << "\ndistance:\n" << (tips_position[0].head(3) - spherePos).norm()
                                << std::endl;
                    doCyclicDescent = false;
                } else {
                    std::cout << "reachable!!\ndestination:\n" << spherePos.transpose()
                              << "\nstart tip position:\n" << tips_position[0].head(3).transpose()
                              << "\ndistance:\n" << (tips_position[0].head(3) - spherePos).norm()
                              << std::endl;
                    doCyclicDescent = !doCyclicDescent;
                }
                break;
            case GLFW_KEY_ESCAPE:
                glfwSetWindowShouldClose(window, GLFW_TRUE);
                break;
            case GLFW_KEY_UP:
                cyls[pickedIndex]->RotateInSystem(axis[pickedIndex]->GetTout().rotation(), M_PI_2, Axis::X);
                break;
            case GLFW_KEY_DOWN:
                cyls[pickedIndex]->RotateInSystem(axis[pickedIndex]->GetTout().rotation(), -M_PI_2, Axis::X);
                break;
            case GLFW_KEY_LEFT:
                cyls[pickedIndex]->RotateInSystem(axis[pickedIndex]->GetTout().rotation(), M_PI_2, Axis::Z);
                break;
            case GLFW_KEY_RIGHT:
                cyls[pickedIndex]->RotateInSystem(axis[pickedIndex]->GetTout().rotation(), -M_PI_2, Axis::Z);
                break;
            case GLFW_KEY_W:
                camera->TranslateInSystem(system, {0, 0.1f, 0});
                break;
            case GLFW_KEY_S:
                camera->TranslateInSystem(system, {0, -0.1f, 0});
                break;
            case GLFW_KEY_A:
                camera->TranslateInSystem(system, {-0.1f, 0, 0});
                break;
            case GLFW_KEY_D:
                camera->TranslateInSystem(system, {0.1f, 0, 0});
                break;
            case GLFW_KEY_B:
                camera->TranslateInSystem(system, {0, 0, 0.1f});
                break;
            case GLFW_KEY_F:
                camera->TranslateInSystem(system, {0, 0, -0.1f});
                break;
            case GLFW_KEY_P:
                if(std::find(cyls.begin(), cyls.end(), pickedModel) != cyls.end())
                {
                    Eigen::Matrix3f phiZ, thetaX, psiZ;
                    std::cout << "picked model: " << pickedModel->GetMesh()->name << std::endl;
                    getZXZRotationMatrices(pickedModel->GetTout().rotation(), phiZ, thetaX, psiZ);
                    std::cout << "phiZ:\n" << phiZ << std::endl;
                    std::cout << "thetaX:\n" << thetaX << std::endl;
                    std::cout << "psiZ:\n" << psiZ << std::endl;
                } else {
                    Eigen::Matrix3f phiZ, thetaX, psiZ;
                    getZXZRotationMatrices(root->GetRotation(), phiZ, thetaX, psiZ);
                    std::cout << "phiZ:\n" << phiZ << std::endl;
                    std::cout << "thetaX:\n" << thetaX << std::endl;
                    std::cout << "psiZ:\n" << psiZ << std::endl;
                }
                break;
            case GLFW_KEY_T:
                // recalculate tips position
                calcTipsPosition(initial_tip_pos, tips_position, root, armRoot, cyls);
                std::cout << "tips_position position:" << std::endl;
                for (int i = 0; i < tips_position.size(); i++)
                {
                    std::cout << "link " << i << "\n" << std::setprecision(5) << tips_position[i].head(3).transpose() << std::endl;
                }
                break;
            case GLFW_KEY_Y:
                // sphere center is (0, 0, 0) hens the translation vector is the center position in the scene
                std::cout << "destination position:\n" << GetSpherePos().transpose() << std::endl;
                break;
            case GLFW_KEY_1:
                if (pickedIndex > 0)
                    pickedIndex--;
                break;
            case GLFW_KEY_2:
                if (pickedIndex < cyls.size() - 1)
                    pickedIndex++;
                break;
            case GLFW_KEY_RIGHT_BRACKET:
                numOfCyls++;
                //Init(CAMERA_ANGLE, DISPLAY_WIDTH, DISPLAY_HEIGHT, NEAR, FAR);
                break;
            case GLFW_KEY_LEFT_BRACKET:
                if (numOfCyls > 1)
                    numOfCyls--;
                //Init(CAMERA_ANGLE, DISPLAY_WIDTH, DISPLAY_HEIGHT, NEAR, FAR);
                break;
            case GLFW_KEY_R:
                //Init(CAMERA_ANGLE, DISPLAY_WIDTH, DISPLAY_HEIGHT, NEAR, FAR);
                break;
        }
    }
}

void BasicScene::nextCyclicDescentStep()
{
    if (doCyclicDescent)
        doCyclicDescent = cyclicCoordinateDescent(cyls, axis, tips_position, GetSpherePos(), DELTA, previousMovingCyl, root, armRoot);
}

Eigen::Vector3f BasicScene::GetSpherePos()
{
    return (sphere1->GetTransform() * Eigen::Vector4f(0,0,0,1)).head(3);
//    return (root->GetAggregatedTransform() * sphere1->GetAggregatedTransform() * Eigen::Vector4f(0,0,0,1)).head(3);
}

bool BasicScene::isReachable()
{
    calcTipsPosition(initial_tip_pos, tips_position, root, armRoot, cyls);
    spherePos = GetSpherePos();
    return (tips_position[0].head(3) - spherePos).norm() > numOfCyls*1.6f*scaleFactor + DELTA;
}

//std::vector<Eigen::Vector4f> BasicScene::GetTipsPositionVec() {
//    calcTipsPosition(initial_tip_pos, tips_position, root, armRoot, cyls);
//    return tips_position;
//}

