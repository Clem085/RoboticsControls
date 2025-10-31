#%%
# make a list of x, y, z - velocities: 
arm_vel = [2, 3, 2.5]

# print the last entry in vel: 
#print(arm_vel([3]))

# Now imagine that this is the velocity of the tip of a robot arm based on how its joints are rotating.
# If we want to add the velocity of a mobile base that it is attached to the arm to get the total velocity 
# of the robot, we can add the base velocity to the arm velocity.
base_vel = [1, 0, 0]

#%%
# add the base velocity to the arm velocity:
total_robot_vel = arm_vel + base_vel
print(total velocity is:, total_robot_vel)

# if the output is not what you expected, how you can fix it?
# TODO put your answer here as a comment: 

#%%
# Now let's introduce the numpy library to make linear algebra operations much easier.
import numpy as np

# we can make arrays from the previous lists
arm_vel_np = np.array(arm_vel)
base_vel_np = np.array(base_vel)

# add the base velocity to the arm velocity:
total_robot_vel_np = arm_vel_np + base_vel_np

print('total velocity is:', total_robot_vel_np)

#TODO answer this question as a comment below:
# did this behave as you expected?



#%%
# Now we want to calculate the magnitude of the total velocity of the tip of the robot arm.
# we can do this by calculating the norm of the total velocity vector as follows: 
total_robot_vel_magnitude = np.sqrt(total_robot_vel_np*total_robot_vel_np)

# did this give the answer you expected? If not, why not? Can you fix it? 

# TODO put your answer here.


