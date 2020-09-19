This is one of my homework in last semester. 

I was required to simulate the movements of a 3 joints arm in 2D plane (as a serial link mechanism). Three joints are shoulder, elbow, wrist represented by joint angles θ1, θ2, θ3. In addition, we need know the distance from head to shoulder as l0, one from shouler to elbow as l1, one from elbow to wrist as l2, one from wrist to hand as l3. 

I'd like to neglect the parts of forward kinematics & inverse kinematics and control system theory, only to keep record of my codes. 

____________________________________________________________________________________________________________________

Fitst, I assumed the approximate values of an arm for an adult man and find a position where the end effector moves effectively. 

l0 = 0.3, l1 = 0.3, l2 = 0.27, l3 = 0.1 (unit : m)
The effective movable position of end effector is : (0.157, 0.435), where θ1 = 55°, θ2 = 90°, θ3 = 15°. 
____________________________________________________________________________________________________________________

Second, the end effector (hand) should move form initial position (xs,ys) to target position (xe,ye) with grasping an object with mass m. The viscous friction with viscosity coefficitent Bi of each joint should be considered. 

After calculating the moment of inertia for each joint and performing a force balance analysis, the equation of motions at each joint are known. 

Next, joints are controlled with proportional feedback with proportional gain Ki when the end effector moves to target position, which means joint torque is decided by τi = Ki(θei - θi).  (i = 1,2,3) 

Combining the joint torques which control systems supply and the equation of motions, and performing Laplace trans, I have the transfer function from target joint angle θei to real-time joint angle θi of each joint.

There are still something should be mentioned: each joint is controlled with "step input"; the distance is 0.1m; m = 1kg.

Now, I simulated the trajectory of the end effector form initial position to target position. 
