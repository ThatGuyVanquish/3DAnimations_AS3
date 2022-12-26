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

// #include "AutoMorphingModel.h"

using namespace cg3d;

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
    root->AddChild(armRoot = Movable::Create("armRoot"));

    // scale factor for the multi-links arm
    float scaleFactor = 1;

    // global Axis
    Eigen::MatrixXd vertices(6,3);
    vertices << -1,0,0,1,0,0,0,-1,0,0,1,0,0,0,-1,0,0,1;
    Eigen::MatrixXi faces(3,2);
    faces << 0,1,2,3,4,5;
    Eigen::MatrixXd vertexNormals = Eigen::MatrixXd::Ones(6,3);
    Eigen::MatrixXd textureCoords = Eigen::MatrixXd::Ones(6,2);
    std::shared_ptr<Mesh> coordsys = std::make_shared<Mesh>("coordsys",vertices,faces,vertexNormals,textureCoords);
    axis.push_back(Model::Create("axis",coordsys,material1));
    axis[0]->mode = 1;
    axis[0]->Scale(2.0f*1.6f*scaleFactor);
    armRoot->AddChild(axis[0]);

    // cylinders
    cyls.push_back( Model::Create("cyl",cylMesh, material));
    cyls[0]->Scale(scaleFactor,Axis::X);
    cyls[0]->SetCenter(Eigen::Vector3f(0,0,-0.8f*scaleFactor));
    armRoot->AddChild(cyls[0]);

    for(int i = 1;i < 3; i++)
    { 
        cyls.push_back( Model::Create("cyl", cylMesh, material));
        cyls[i]->Scale(scaleFactor,Axis::Z);
        cyls[i]->Translate(1.6f*scaleFactor,Axis::Z);
        cyls[i]->SetCenter(Eigen::Vector3f(0,0,-0.8f*scaleFactor));
        cyls[i-1]->AddChild(cyls[i]);

        // cylinder i axis system according to the previous cylinder
        axis.push_back(Model::Create("axis",coordsys,material1));
        axis[i]->mode = 1;
        axis[i]->Scale(2.0f*1.6f*scaleFactor);
        axis[i]->Translate(0.8f*scaleFactor,Axis::Z);
        axis[i]->SetCenter(Eigen::Vector3f(0,0,-0.8f*scaleFactor));
        cyls[i-1]->AddChild(axis[i]);

    }
    cyls[0]->Translate({0,0,0.8f*scaleFactor});

    // init tips position
    init_tip = Eigen::Vector4f(0,0,1.6f, 1);
    tips.push_back(cyls[0]->GetTransform() * init_tip);
    tips.push_back(cyls[1]->GetTransform() * init_tip);
    tips.push_back(cyls[2]->GetTransform() * init_tip);


    // sphere
    sphere1->showWireframe = true;
    sphere1->Translate({5,0,0});


    camera->Translate(50, Axis::X);
    camera->Rotate(89.55f, Axis::Y);
    root->AddChild(sphere1);


}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 0.8f, 0.3f, 0.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 0.3f, 0.6f, 1.0f);
    program.SetUniform4f("Kdi", 0.5f, 0.5f, 0.0f, 1.0f);
    program.SetUniform1f("specular_exponent", 5.0);
    program.SetUniform4f("light_position", 0.0, 15.0, 0.0, 1.0);

    auto print = [&](){
        std::cout << "R*Rt:" << cyls[1]->GetRotation() * cyls[1]->GetRotation().transpose() << std::endl;
        std::cout << "R.determinant:" << cyls[1]->GetRotation().determinant() << std::endl;
    };


//    cyl->Rotate(0.001f, Axis::Y);
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
        pickedModel->TranslateInSystem(system, {0, 0, -float(yoffset)});
        pickedToutAtPress = pickedModel->GetTout();
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
                pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Z);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
                pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Y);
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
            case GLFW_KEY_ESCAPE:
                glfwSetWindowShouldClose(window, GLFW_TRUE);
                break;
            case GLFW_KEY_UP:
                cyls[pickedIndex]->RotateInSystem(system, 0.1f, Axis::X);
                break;
            case GLFW_KEY_DOWN:
                cyls[pickedIndex]->RotateInSystem(system, -0.1f, Axis::X);
                break;
            case GLFW_KEY_LEFT:
                cyls[pickedIndex]->RotateInSystem(system, 0.1f, Axis::Y);
                break;
            case GLFW_KEY_RIGHT:
                cyls[pickedIndex]->RotateInSystem(system, -0.1f, Axis::Y);
                std::cout << "R*Rt:" << cyls[1]->GetRotation() * cyls[1]->GetRotation().transpose() << std::endl;
                std::cout << "R.determinant:" << cyls[1]->GetRotation().determinant() << std::endl;
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
                    getZXZRotationMatrices(pickedModel->GetRotation(), phiZ, thetaX, psiZ);
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
                tips.push_back(cyls[0]->GetTransform() * init_tip);
                tips.push_back(cyls[1]->GetTransform() * init_tip);
                tips.push_back(cyls[2]->GetTransform() * init_tip);
                std::cout << "tips position:\n"
                            << "link 0\n" << tips[0].head(3) << "\n"
                            << "link 1\n" << tips[1].head(3) << "\n"
                            << "link 2\n" << tips[2].head(3) << "\n"
                            << std::endl;
                break;
            case GLFW_KEY_Y:
                // sphere center is (0, 0, 0) hens the translation vector is the center position in the scene
                std::cout << "destination position:\n" << sphere1->GetTranslation() << std::endl;
                break;
            case GLFW_KEY_1:
                if (pickedIndex > 0)
                    pickedIndex--;
                break;
            case GLFW_KEY_2:
                if (pickedIndex < cyls.size() - 1)
                    pickedIndex++;
                break;
            case GLFW_KEY_3:
                if (tipIndex > 0)
                {
                    tipIndex--;
                    sphere1->Translate(GetSpherePos(-1));
                }
//                {
//                  if(tipIndex == cyls.size())
//                    tipIndex--;
//                  sphere1->Translate(GetSpherePos(-1));
//                  tipIndex--;
//                }
                break;
            case GLFW_KEY_4:
                if(tipIndex < cyls.size())
                {
                    sphere1->Translate(GetSpherePos(1));
                    tipIndex++;
                }
//                {
//                    if(tipIndex < 0)
//                      tipIndex++;
//                    sphere1->Translate(GetSpherePos(1));
//                    tipIndex++;
//                }
                break;
        }
    }
}

Eigen::Vector3f BasicScene::GetSpherePos(int dir)
{
      Eigen::Vector3f l = Eigen::Vector3f(0,0,1.6f);
      Eigen::Vector3f res;
      res = cyls[tipIndex]->GetRotation()*(dir*l);
      return res;  
}
