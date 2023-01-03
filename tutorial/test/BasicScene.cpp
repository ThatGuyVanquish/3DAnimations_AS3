#include "BasicScene.h"


using namespace cg3d;

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
	camera = Camera::Create("camera", fov, float(width) / height, near, far);

	AddChild(root = Movable::Create("root")); // a common invisible parent object for all the shapes

    auto daylight{ std::make_shared<Material>("daylight", "shaders/cubemapShader") };
    daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
    auto background{ Model::Create("background", Mesh::Cube(), daylight) };
    AddChild(background);
    background->Scale(120, Axis::XYZ);
    background->SetPickable(false);
    background->SetStatic();

    // setup camera
    camera->Translate(50, Axis::X);
    camera->cg3d::Movable::Rotate((float)M_PI_2, Axis::Y);

    // setup models
    phongShader = std::make_shared <cg3d::Program>("shaders/phongShader");
    pickingShader = std::make_shared<cg3d::Program>("shaders/pickingShader");

    material = std::make_shared<cg3d::Material>("material", phongShader);
    material1 = std::make_shared<cg3d::Material>("material1", pickingShader);
    auto basicShader = std::make_shared<Program>("shaders/basicShader");
    material2 = std::make_shared<cg3d::Material>("material2", basicShader);
    material2->AddTexture(0, "textures/grass.bmp", 2);

    sphereMesh = IglLoader::MeshFromFiles("sphere_igl", "data/sphere.obj") ;
    cylMesh = IglLoader::MeshFromFiles("cyl_igl","data/zCylinder.obj") ;

    sphere = Model::Create("Sphere", sphereMesh, material);
    sphere->showWireframe = true;
    sphere->showFaces = false;
    root->AddChild(sphere);

    // Creation of axis mesh
    Eigen::MatrixXd vertices(6, 3);
    vertices << -1, 0, 0, 1, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, -1, 0, 0, 1;
    Eigen::MatrixXi faces(3, 2);
    faces << 0, 1, 2, 3, 4, 5;
    Eigen::MatrixXd vertexNormals = Eigen::MatrixXd::Ones(6, 3);
    Eigen::MatrixXd textureCoords = Eigen::MatrixXd::Ones(6, 2);
    coordsys = std::make_shared<Mesh>("coordsys", vertices, faces, vertexNormals, textureCoords);

    initialSettings();
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
    Scene::Update(program, proj, view, model);
    program.SetUniform4f("lightColor", 0.8f, 0.3f, 0.0f, 0.5f);
    program.SetUniform4f("Kai", 1.0f, 0.3f, 0.6f, 1.0f);
    program.SetUniform4f("Kdi", 0.5f, 0.5f, 0.0f, 1.0f);
    program.SetUniform1f("specular_exponent", 5.0);
    program.SetUniform4f("light_position", 0.0, 15.0, 0.0, 1.0);
}

void BasicScene::nextCyclicDescentStep()
{
    if (doCyclicDescent)
    {
        Eigen::Vector3f spherePos = getSpherePosition();
        doCyclicDescent = cyclicCoordinateDescent(cyls, axis, spherePos, DELTA, cylinderToMove, root);
    }
}

// Set-up methods

void BasicScene::initialSettings()
{
    for (int i = 0; i < numOfCyls; i++) 
    {
        // Create cylinder and axies
        cyls.push_back(Model::Create("Cyl" + std::to_string(i), cylMesh, material));
        axis.push_back(Model::Create("Axis " + std::to_string(i), coordsys, material1));
        // set dependencies
        if (i == 0) // first axis and cylinder depend on scene's root
            root->AddChildren({ axis[0], cyls[0] });
        else // rest of the axies and cylinders depend on the previous cylinder
            cyls[i - 1]->AddChildren({ axis[i], cyls[i] });
    }

    BasicScene::initializeBaseOfArm();
    BasicScene::initializeRestOfArm();

    // rotate arm to align upwards
    cyls[0]->Rotate((float)(-M_PI_2), cg3d::Movable::Axis::X);

    // move sphere to initial position
    sphere->SetTransform(Eigen::Matrix4f::Identity());
    sphere->Translate(INITIAL_SPHERE_POS);

    cylinderToMove = (int)cyls.size() - 1;
}

void BasicScene::initializeBaseOfArm()
{
    cyls[0]->Scale(scaleFactor, Axis::Z);
    cyls[0]->Translate({ 0, 0, 0.8f * scaleFactor });
    cyls[0]->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
    axis[0]->mode = 1;
    axis[0]->Scale(2.0f * 1.6f * scaleFactor);
    axis[0]->isPickable = false;
}

void BasicScene::initializeRestOfArm()
{
    for (int i = 1; i < numOfCyls; i++)
    {
        cyls[i]->Scale(scaleFactor, Axis::Z);
        cyls[i]->Translate(1.6f * scaleFactor, Axis::Z);
        cyls[i]->SetCenter(Eigen::Vector3f(0, 0, -0.8f * scaleFactor));
        // cylinder i axis system according to the previous cylinder
        axis[i]->mode = 1;
        axis[i]->Scale(2.0f * 1.6f * scaleFactor);
        axis[i]->Translate(0.8f * scaleFactor, Axis::Z);
        axis[i]->isPickable = false;
    }
}

void BasicScene::reset()
{
    emptyCylinderVectors();
    root->SetTransform(Eigen::Matrix4f::Identity());
    initialSettings();
}

void BasicScene::emptyCylinderVectors()
{
    for (int i = cyls.size() - 1; i > 0; i--) {
        cyls[i - 1]->RemoveChild(cyls[i]);
        cyls[i - 1]->RemoveChild(axis[i]);
        cyls.pop_back();
        axis.pop_back();
    }

    // remove base of arm
    root->RemoveChild(cyls[0]);
    root->RemoveChild(axis[0]);
    cyls.pop_back();
    axis.pop_back();
}

// Calculation methods
Eigen::Vector3f BasicScene::getSpherePosition()
{
    return (sphere->GetAggregatedTransform() * Eigen::Vector4f(0, 0, 0, 1)).head(3);
}

bool BasicScene::isReachable()
{
    auto spherePos = getSpherePosition();
    auto rootOfArmPos = getTipPosition(0, -0.8f, cyls);
    float dist = std::abs((spherePos - rootOfArmPos).norm());
    float maxArmLength = numOfCyls * 1.6f;
    float maxReachableDistance = maxArmLength * scaleFactor + DELTA;
    return dist <= maxReachableDistance;
}

// CALLBACKS

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

        if (pickedModel && !pickedModel->isPickable)
            pickedModel = nullptr; // for non-pickable models we need only pickedModelDepth for mouse movement calculations later

        // set pickedIndex to selected cylinder if picked a cylinder
        if (pickedModel && std::find(cyls.begin(), cyls.end(), pickedModel) != cyls.end())
        {
            BasicScene::pickedACylinder((int)(std::find(cyls.begin(), cyls.end(), pickedModel) - cyls.begin()));
        }

        if (pickedModel)
            pickedToutAtPress = pickedModel->GetTout();
        else
            cameraToutAtPress = camera->GetTout();
    }
}

void BasicScene::pickedACylinder(int index)
{
    pickedIndex = index;
    for (std::size_t i = 0; i < cyls.size(); i++)
    {
        cyls[i]->material = material;
    }
    if (index == -1) return;
    cyls[index]->material = material2;
}

void BasicScene::ScrollCallback(Viewport* viewport, int x, int y, int xoffset, int yoffset, bool dragging, int buttonState[])
{
    // note: there's a (small) chance the button state here precedes the mouse press/release event
    auto system = camera->GetRotation().transpose();
    if (pickedModel) {
        if (std::find(cyls.begin(), cyls.end(), pickedModel) != cyls.end())
        {
            cyls[0]->TranslateInSystem(system, { 0, 0, -float(yoffset) });
            pickedToutAtPress = cyls[0]->GetTout();
        }
        else {
            pickedModel->TranslateInSystem(system, { 0, 0, -float(yoffset) });
            pickedToutAtPress = pickedModel->GetTout();
        }
    }
    else {
        camera->TranslateInSystem(system, { 0, 0, -float(yoffset) });
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
            if (std::find(cyls.begin(), cyls.end(), pickedModel) != cyls.end())
            { // pickedModel is a cylinder
                if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE) // move all cylinders as one, therefore move cyls[0]
                    cyls[0]->TranslateInSystem(system, { -float(xAtPress - x) / moveCoeff, float(yAtPress - y) / moveCoeff, 0 });
                if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                    cyls[pickedIndex]->RotateInSystem(axis[pickedIndex]->GetRotation(), float(xAtPress - x) / angleCoeff, Axis::Y);
                if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE)
                {
                    // Rotate based on euler angles
                }
            }
            else
            {
                if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE)
                    pickedModel->TranslateInSystem(system, { -float(xAtPress - x) / moveCoeff, float(yAtPress - y) / moveCoeff, 0 });
                if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                    pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Y);
                if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) 
                {
                    pickedModel->RotateInSystem(system, float(xAtPress - x) / angleCoeff, Axis::Z);
                    pickedModel->RotateInSystem(system, float(yAtPress - y) / angleCoeff, Axis::X);
                }
            }
        }
        else {
            // camera->SetTout(cameraToutAtPress);
            if (buttonState[GLFW_MOUSE_BUTTON_RIGHT] != GLFW_RELEASE)
                root->TranslateInSystem(system, { -float(xAtPress - x) / moveCoeff / 10.0f, float(yAtPress - y) / moveCoeff / 10.0f, 0 });
            if (buttonState[GLFW_MOUSE_BUTTON_MIDDLE] != GLFW_RELEASE)
                root->RotateInSystem(system, float(x - xAtPress) / 180, Axis::Z);
            if (buttonState[GLFW_MOUSE_BUTTON_LEFT] != GLFW_RELEASE) {
                root->RotateInSystem(system, float(x - xAtPress) / angleCoeff, Axis::Y);
                root->RotateInSystem(system, float(y - yAtPress) / angleCoeff, Axis::X);
            }
        }
        xAtPress = x;
        yAtPress = y;
    }
}

void BasicScene::KeyCallback(cg3d::Viewport* viewport, int x, int y, int key, int scancode, int action, int mods)
{
    auto system = camera->GetRotation().transpose();
    auto tipOfArm = getTipPosition(cyls.size() - 1, -0.8f, cyls);
    float dist = std::abs((tipOfArm - getSpherePosition()).norm());
    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        switch (key)
        {
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            break;

        // Camera movement
         case GLFW_KEY_W:
            camera->TranslateInSystem(system, { 0, 0.1f, 0 });
            break;
        case GLFW_KEY_S:
            camera->TranslateInSystem(system, { 0, -0.1f, 0 });
            break;
        case GLFW_KEY_A:
            camera->TranslateInSystem(system, { -0.1f, 0, 0 });
            break;
        case GLFW_KEY_D:
            camera->TranslateInSystem(system, { 0.1f, 0, 0 });
            break;
        case GLFW_KEY_B:
            camera->TranslateInSystem(system, { 0, 0, 0.1f });
            break;
        case GLFW_KEY_F:
            camera->TranslateInSystem(system, { 0, 0, -0.1f });
            break;

        // Cylinder picking
        case GLFW_KEY_1:
            --pickedIndex;
            if (pickedIndex <= -1) pickedIndex = numOfCyls - 1;
            pickedACylinder(pickedIndex % numOfCyls);
            break;
        case GLFW_KEY_2:
            pickedACylinder((++pickedIndex) % numOfCyls);
            break;
        case GLFW_KEY_3:
            pickedACylinder(-1);
            break;

        // Cylinder Movement
        case GLFW_KEY_SPACE: // Start/stop IK animation
            std::cout << "\n\n\n";
            printAllTips(cyls);
            std::cout << "\n\n\n";

            if (isReachable())
            {
                std::cout << "Can reach the destination at " << getSpherePosition().transpose()
                    << "\nThe distance to it is: " << dist << std::endl;
                doCyclicDescent = !doCyclicDescent;
            }
            else
            {
                std::cout << "Can't reach the destination at " << getSpherePosition().transpose()
                    << "\nThe distance to it is: " << dist << std::endl;
                doCyclicDescent = false;
            }
            break;
        case GLFW_KEY_UP:
            if (pickedIndex != -1)
            {
                // Rotate around positive X axis using euler angles
                Eigen::Vector3f dir = { 0, 0.1f, 0 };
                rotateBasedOnEulerAngles(cyls, axis, pickedIndex, dir);
            }
            else
                root->Rotate(0.1f, Axis::X);
            break;
        case GLFW_KEY_DOWN:
            if (pickedIndex != -1)
            {
                // Rotate around negative X axis using euler angles
                Eigen::Vector3f dir = { 0, -0.1f, 0 };
                rotateBasedOnEulerAngles(cyls, axis, pickedIndex, dir);
            }
            else
                root->Rotate(-0.1f, Axis::X);
            break;
        case GLFW_KEY_RIGHT:
            if (pickedIndex != -1)
            {
                // Rotate around positive Z axis using euler angles
                Eigen::Vector3f dir = { 0.1f, 0, 0 };
                rotateBasedOnEulerAngles(cyls, axis, pickedIndex, dir);
            }
            else
                root->Rotate(0.1f, Axis::Z);
            break;
        case GLFW_KEY_LEFT:
            if (pickedIndex != -1)
            {
                // Rotate around negative Z axis using euler angles
                Eigen::Vector3f dir = { -0.1f, 0, 0 };
                rotateBasedOnEulerAngles(cyls, axis, pickedIndex, dir);
            }
            else
                root->Rotate(-0.1f, Axis::Z);
            break;
        
        // Change amount of cylinders
        case GLFW_KEY_RIGHT_BRACKET:
            numOfCyls++;
            reset();
            break;
        case GLFW_KEY_LEFT_BRACKET:
            if (numOfCyls > 1)
                numOfCyls--;
            reset();
            break;
        case GLFW_KEY_R:
            reset();
            break;

        // Printouts
        case GLFW_KEY_P:
            if (std::find(cyls.begin(), cyls.end(), pickedModel) != cyls.end())
            {
                Eigen::Matrix3f phiZ, thetaX, psiZ;
                std::cout << "picked model: " << pickedModel->GetMesh()->name << std::endl;
                getZXZRotationMatrices(pickedModel->GetTout().rotation(), phiZ, thetaX, psiZ);
                std::cout << "phiZ:\n" << phiZ << std::endl;
                std::cout << "thetaX:\n" << thetaX << std::endl;
                std::cout << "psiZ:\n" << psiZ << std::endl;
            }
            else {
                Eigen::Matrix3f phiZ, thetaX, psiZ;
                getZXZRotationMatrices(root->GetRotation(), phiZ, thetaX, psiZ);
                std::cout << "phiZ:\n" << phiZ << std::endl;
                std::cout << "thetaX:\n" << thetaX << std::endl;
                std::cout << "psiZ:\n" << psiZ << std::endl;
            }
            break;
        case GLFW_KEY_T:
            std::cout << "tip positions:" << std::endl;
            for (int i = 0; i < cyls.size(); i++)
            {
                std::cout << "link " << i << "\n" << std::setprecision(5) << getTipPosition(i, 0.8f, cyls).transpose() << std::endl;
            }
            break;
        case GLFW_KEY_Y:
            // sphere center is (0, 0, 0) hens the translation vector is the center position in the scene
            std::cout << "destination position:\n" << getSpherePosition().transpose() << std::endl;
            break;
        }
    }
}