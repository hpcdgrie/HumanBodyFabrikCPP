
# FULL HUMAN BODY INVERSE KINEMATIC SIMULATION BY FABRIK 
C++ implemantionion based on https://github.com/Atiehmerikh/FABRIK_Full_Body.git
# Description:

This library is an implementation of the FABRIK method(Forward And Backward Reaching Inverse kinematic) 
for whole human body in the C++ programming language and is released under the MIT software license 
and is under developement. It can simulate the human behavior in reaching a target with consideration of 
the human joints constraints.
NOTE: This library is under developement so there is some update in future.

## Usage:
	
To use the code you should specify some values in text files (see `test` folder in the root):
First you should consider the numbering in `body_number.jpg` file and rest of input files
are based on these numbering

1. `joits_position.txt` : This is the initial position of the joints
2. `joints_constraint.txt`: This is the constraints for each human body joints. According to the joint reference plane and coordinate, you should enter four number for each joint (Adduction, Abduction, Flexion, Extension).Notice that order of these four number is according to joints orientation in its reference plane(in degree).
3. `orientation.txt`: This is the initial orientation of each joint in quaternion
4. `bone_twist_constraints.txt`: The twist limitations for each bone. The limitation for each bone should be based on the outer bone number. For anyone that you don't have data let it be on its default.
5. `constraint_type.txt`: File for getting the joints constraint ("BALL": for ball and socket) and ("hinge" : for hinge type)
6. `target.txt`: First line of this file specifies the target position and the second line represents the orientation of the target

After creating these files inside the `input` folder (see `test` folder) and creating `output` folder (it should be empty first), you can test the FABRIK method like the following:

    mkdir build
    cd build
    cmake ..
    make 
    cd test
    test.exe

Or you can include the library oin your own project:

    find_packe(fabrik)
    target_link_libraries(your_target fabrik::fabrik)


## OutPut:

The calculated joint positions are printed on command line 

## References:

The FABRIK algorithm is explained in the following research paper:

	Aristidou, A., & Lasenby, J. (2011). FABRIK: a fast, iterative solver for the inverse kinematics problem. Graphical Models, 73(5), 243-260.

## Publication:
By using this library this paper is published. Please cite it if you used this library:

Atieh Merikh-Nejadasl, Ilias El Makrini, Greet Van De Perre, Tom Verstraten, Bram Vanderborght,
A generic algorithm for computing optimal ergonomic postures during working in an industrial environment,
International Journal of Industrial Ergonomics,
Volume 84,
2021,
103145,
ISSN 0169-8141,
https://doi.org/10.1016/j.ergon.2021.103145.
(https://www.sciencedirect.com/science/article/pii/S0169814121000639)
Abstract: The present study tries to decrease the risk of work-related musculoskeletal disorders for industry workers by proposing a generic algorithm that recommends an optimal ergonomic posture for accomplishing tasks in an industrial environment. In the case of a dangerous ergonomic pose, the optimization algorithm starts by heuristically changing it to a more ergonomic one. Each recommended posture's feasibility is tested with an inverse kinematic method that can predict the worker's behavior for accomplishing a task. This iterative optimization procedure continues until the optimal ergonomic pose for the worker is achieved. The algorithm's validity is tested in thirteen cases, people with different gender (50 percent male, 50 percent female) aged between 20 and 35, and different height and body morphologies. According to studies, there is a connection between musculoskeletal disorders and the wrong posture for accomplishing tasks in industries. We suggest an optimization algorithm that can indicate the worker the optimal ergonomic pose by considering task constraints in real-time.
Keywords: Ergonomic optimization; FABRIK; REBA; Inverse kinematic; Python

##  Future Works
1. solve the algorihm for joints constraint which makes parabolic section in joints reference plane
2. target locating on the line of kinematic 
3. Handling If the target is unreachable i.e. if the distance between the root and the target is less than the length of the kinematic chain.
4. multiple end effector
5. the human feet be able to move
	
