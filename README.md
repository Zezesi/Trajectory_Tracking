This repository is mainly dedicated to NMPC based trajectory tracking with a simplified vehicle kinematic model considering action time delay.

NMPC_TrajTracking_CasADi.py: pure trajectory tracking. 

Results:![tt1](https://github.com/user-attachments/assets/70fbb13b-ffac-4cc1-b34f-3aa59bcbbc05)
![tt2](https://github.com/user-attachments/assets/4632336b-f675-4e73-864b-75f47fae213a)

NMPC_TrajTrackingAndObstacleAvoidance_CasADi.py: trajectory tracking and obstacle avoidance with hard constraints. 

Results:![tthc1](https://github.com/user-attachments/assets/f9eded96-0684-4352-83e1-44c98dfbddb7)
![tthc2](https://github.com/user-attachments/assets/cd71e5ec-8e47-4044-8184-0bb3b5cbf526)

NMPC_TrajTrackingAndSoftObstacleAvoidance_CasADi.py: trajectory tracking and obstacle avoidance with soft constraints. 

Results:
![ttsc1](https://github.com/user-attachments/assets/bea32ea3-48f8-4054-a24c-b834a5793320)
![ttsc2](https://github.com/user-attachments/assets/a590e2e1-eef8-4a0f-baf6-8409bce3799f)

NMPC_TrajTracking_CasADi1.py: pure trajectory tracking, in the above scripts, delta actions are used to ensure smooth action transition, now jerk and delta_f_dot are
modelled in the nmpc model.

Results:![tt11](https://github.com/user-attachments/assets/52887c67-090b-4f9d-a0ef-2265229d27a2)
![tt12](https://github.com/user-attachments/assets/17f240f2-c51c-4c43-9617-d56052d0972e)

Notes: Cost function and some hard constraints can be improved using soft functions.
