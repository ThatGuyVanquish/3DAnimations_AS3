# Lior and Nave's 3D Animations Assignment 3 (Inverse Kinematics)

***********************************
The assignment is coded in the folder [Assignment3](https://github.com/ThatGuyVanquish/3DAnimations_AS3/tree/master/tutorial/Assignment3)
#    

Our implementation added a CPP file:
[KinematicChain.cpp](https://github.com/ThatGuyVanquish/3DAnimations_AS3/blob/master/tutorial/Assignment3/KinematicChain.cpp)

Which holds static functions used to calculate the position of the cylinder for moving, either by arrows\camera (using euler angles) and through the attempt to reach the sphere (cyclic descent).

### 

The function [*cyclicCoordinateDescentStep*](https://github.com/ThatGuyVanquish/3DAnimations_AS3/blob/eb88b1ff5268d3fb8253f8e4c8e373bc0689a354/tutorial/Assignment3/KinematicChain.cpp#L69) is using the equations from the article and basic trigonometry to get the angle to rotate and the vector to rotate around, then physically rotates the arm's parts one by one in order to reach the destination.

The virtual function [*nextCyclicDescentStep*](https://github.com/ThatGuyVanquish/3DAnimations_AS3/blob/master/engine/Scene.h#L39) was added to Scene.h, which is called every time the renderer calls for [*RenderAllViewports*](https://github.com/ThatGuyVanquish/3DAnimations_AS3/blob/master/engine/Renderer.cpp#L48).
         The function checks the state of a boolean variable to use cyclicCoordinateDescentStep when it can (and is instructed to by pressing *space*) to move the arm to the destination.

###
The function [*rotateBasedOnEulerAngles*](https://github.com/ThatGuyVanquish/3DAnimations_AS3/blob/master/tutorial/Assignment3/KinematicChain.cpp#L57) is the function used by the arrows to rotate cylinders around it's axis based on euler angles. 
It uses the function [*getZXZRotationMatrices*](https://github.com/ThatGuyVanquish/3DAnimations_AS3/blob/master/tutorial/Assignment3/KinematicChain.cpp#L18) to calculate the new rotation matrix and sets the rotation as per the euler angle matrices.

##
### BUTTON MAPPINGS

**H**      -> Hide\show axis

#### Camera movement
**WASDBF** -> Move the camera around

**Can also move camera with the mouse when there isn't a picked object**

#### Cylinder picking
**1, 2**   -> Cycle between cylinder indices (I.E. set the cylinder as "picked object to be moved by keyboard")

**3**      -> Unpick any cylinder previously picked

#### Amount of cylinders
**[, ]**   -> Move the number of cylinders up or down (down to at least 1). Note that this resets the scene.

**R**      -> Reset the scene

#### Printouts
**P**      -> Prints the Euler Rotation Matrices of **the picked object** or **root**

**T**      -> Prints the tip position of each link in the arm

**Y**      -> Prints the location of the sphere

#### Arrow key movements
**UP**     -> Moves *Picked **Cylinder*** around positive X axis using euler angles

**DOWN**   -> Moves *Picked **Cylinder*** around negative X axis using euler angles

**RIGHT**  -> Moves *Picked **Cylinder*** around positive Z axis using euler angles

**LEFT**   -> Moves *Picked **Cylinder*** around negative Z axis using euler angles

### **SPACE**
**Moves the arm towards the destination (sphere's center with delta DELTA) *if and only if* it can reach**
