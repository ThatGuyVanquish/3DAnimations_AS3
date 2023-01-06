# Lior and Nave's 3D Animations Assignment 3 (Inverse Kinematics)

***********************************
The assignment is coded in the folder [Assignment3](https://github.com/ThatGuyVanquish/3DAnimations_AS3/tree/master/tutorial/Assignment3)
#    

Our implementation added a CPP file:
[KinematicChain.cpp](https://github.com/ThatGuyVanquish/3DAnimations_AS3/blob/master/tutorial/Assignment3/KinematicChain.cpp)

Which holds static functions used to calculate the position of the cylinder for moving, either by arrows\camera (using euler angles) and through the attempt to reach the sphere (cyclic descent).

### 

The function [*cyclicCoordinateDescent*](https://github.com/ThatGuyVanquish/3DAnimations_AS3/blob/d3504bf15820731ea01dd67f60b3061491567ba4/tutorial/Assignment3/KinematicChain.cpp#L62) is using the equations from the article and basic trigonometry to get the angle to rotate and the vector to rotate around, then physically rotates the arm's parts one by one in order to reach the destination.

The virtual function [*nextCyclicStep*](https://github.com/ThatGuyVanquish/3DAnimations_AS3/blob/d3504bf15820731ea01dd67f60b3061491567ba4/engine/Scene.h#L39) was added to Scene.h, which is called every time the renderer calls for [*RenderAllViewports*](https://github.com/ThatGuyVanquish/3DAnimations_AS3/blob/d3504bf15820731ea01dd67f60b3061491567ba4/engine/Renderer.cpp#L48)
The function checks the state of a boolean variable to use cyclicCoordinateDescent when it can (and is instructed to by pressing *space*) to move the arm to the destination.

##
### BUTTON MAPPINGS
#### Camera movement
*WASDBF* -> Move the camera around

*Can also move camera with the mouse when there isn't a picked object*

#### Cylinder picking
*1, 2*   -> Cycle between cylinder indices (I.E. set the cylinder as "picked object to be moved by keyboard")

*3*      -> Unpick any cylinder previously picked

