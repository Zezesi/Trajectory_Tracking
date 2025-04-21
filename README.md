This repository is mainly dedicated to NMPC based trajectory tracking with a simplified vehicle kinematic model considering action time delay.

NMPC_TrajTracking_CasADi.py: pure trajectory tracking. 

Results:![tt1](https://github.com/user-attachments/assets/70fbb13b-ffac-4cc1-b34f-3aa59bcbbc05)
![tt2](https://github.com/user-attachments/assets/4632336b-f675-4e73-864b-75f47fae213a)

NMPC_TrajTrackingAndObstacleAvoidance_CasADi.py: trajectory tracking and obstacle avoidance with hard constraints. 

Results:![tthc1](https://github.com/user-attachments/assets/d80665f4-0d7c-428e-a0aa-2eb8ba42a71d)
![tthc2](https://github.com/user-attachments/assets/d1644490-1db4-4e37-bc85-449bb472989f)

NMPC_TrajTrackingAndSoftObstacleAvoidance_CasADi.py: trajectory tracking and obstacle avoidance with soft constraints. 

Results:
![ttsc1](https://github.com/user-attachments/assets/89968050-1681-4634-a510-a5868662da3d)
![ttsc2](https://github.com/user-attachments/assets/cbdfc69a-1af3-42c8-9eed-699dccb68c79)

NMPC_TrajTracking_CasADi1.py: pure trajectory tracking, in the above scripts, delta actions are used to ensure smooth action transition, now jerk and delta_f_dot are
modelled in the nmpc model.

Results:![tt11](https://github.com/user-attachments/assets/52887c67-090b-4f9d-a0ef-2265229d27a2)
![tt12](https://github.com/user-attachments/assets/17f240f2-c51c-4c43-9617-d56052d0972e)

Notes: Cost function and some hard constraints can be improved using soft functions.
