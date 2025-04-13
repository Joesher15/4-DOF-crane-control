# 4-DOF-crane-control

## Steps to run

### Prerequisites
docker


### Build
```
git clone
cd 4-DOF-crane-control
docker compose build
```

### Run
```docker compose up```

Open a browser and enter:
```http://localhost:8080```

Enter values in the input boxes for motion!

## Approach
I tackled the problem in this order:
- build forward and inverse kinematics functions for a simple robot all defined in python, while checking implementations using matplotlib
- build the robot definition in a URDF file. Use library for urdf loading and forward/inverse kinematics.
- build frontend for simple websocket connection to python backend. Initially only to recieve state of join positions and print the numbers on the browser
- build the complete frontend for sending joint positions, end effector position, and receive state from the backend. Including reading of the same URDF file for visualising the crane
- Add backend and frontend logic for origin control.

## Challenges
- As it was my first time ever building a frontend web-application, there were a lot of small issues in getting the frontend up and running:
    - reading of urdf file (and other static files) in the typescript frontend from the file-system was troublesome due to Access control restrictions. Solved by serving the static files in the application via http-server and routed through a proxy-server which was configured the right set of access controls for the application to access files.
    - the above method, in the basic setup was caching the static files, which caused some confusion and time lost, when static files were changed but the browser did not reflect the changes
    - a little stupid, but the camera position of the scene in the UI, is ultra important to determine if the rendering worked! Spent quite some time puzzling why the WebGL render was just blank.
- On the control side, the most challenging task was to ensure that the URDF file was being intepreted properly.

## Highlights of implementaiont
- the robot has a single source of definition, defined in a single location, used by the frontend and backend. It used URDF, a popular open-source format for defining link, joints and the relationship between them, including joint limits on position, velocity and effort.
- the code hasn't been written with multiple robots in mind, but extension would be possible and easy due to the use of URDF files and OOP used for crane control in the python backend
- docker setup for easy deployment
## Limitations
- decided to skip the gripper open/close control and visualising as it did not add to the control problem but would've only been for visuals.
- due to time-constraints, decided to only limit velocity (not acceleration) in the project 
- due to time-constraints, in two places the proper reading of the URDF file was circumvented. It might result in unexpected behavior if more links/joints were added (especially in the definintion of the end-effector link)
- discovered too late that I had not respected the limits on joint positions (as defined in the URDF file) in the API behavior to directly move the joints from the UI. Results in faulty behavior. Limits are respected however in all forward and inverse kinematic calculations.
- some constant are not well defined in a single place, such as the tolerance used in the inverse kinematics calculations (5cms).

## Answers
- What happens when the EE velocity is smaller than the origin shift velocity:
    - the control system struggles to keep the ee stationary. In some cases, it results in instability if the velocity gradient is too high. Ultimately, if out of the range of the robot, the ee is no longer tracked to the same position.
- How to deal with a bad origin sensor?
    - Use simple filters, such as a low-pass filter for noisy/jittery measurements
    - use adaptive and model-aware filters which takes in to account a movement model of the ground vehicle/ determine the frquency of the noise using feedback