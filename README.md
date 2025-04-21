This repository is mainly dedicated to NMPC based trajectory tracking with a simplified vehicle kinematic model considering action time delay.

NMPC_TrajTracking_CasADi.py: pure trajectory tracking. 

Results:![tt1](https://github.com/user-attachments/assets/dfa4f107-9620-4991-96fe-4220fdbe4c58)
![tt2](https://github.com/user-attachments/assets/4171249d-2c63-4a04-ba38-cf8e34f807e5)

NMPC_TrajTrackingAndObstacleAvoidance_CasADi.py: trajectory tracking and obstacle avoidance with hard constraints. 

Results:![tthc1](https://github.com/user-attachments/assets/aac9a2a8-16ef-497b-89aa-3cece1bb8332)
![tthc2](https://github.com/user-attachments/assets/2d354757-dded-4745-ab92-1a51f905cd3c)

NMPC_TrajTrackingAndSoftObstacleAvoidance_CasADi.py: trajectory tracking and obstacle avoidance with soft constraints. 

Results:![ttsc1](https://github.com/user-attachments/assets/c70f905f-1040-44b3-bb74-15aa5050e9e8)
![ttsc2](https://github.com/user-attachments/assets/6083b144-fd9b-4fbb-be47-00152d48114b)

NMPC_TrajTracking_CasADi1.py: pure trajectory tracking, in the above scripts, delta actions are used to ensure smooth action transition, now jerk and delta_f_dot are
modelled in the nmpc model.

Results:![tt11](https://github.com/user-attachments/assets/e1338423-00d3-4c4b-b816-4da5dada8524)
![tt12](https://github.com/user-attachments/assets/194b59e5-17e7-4b09-86e9-e502c744da67)

LQR_TrajTracking.py: pure trajectory tracking based on a linearized vehicle kinematic model.

Results:
![lqr1](https://github.com/user-attachments/assets/8f2c2e9c-14d5-4083-8d62-ed7e9cee5a1e)
![lqr2](https://github.com/user-attachments/assets/1f82b4a5-5448-45af-943a-9baa0d8fa35c)


Notes: Cost function and some hard constraints can be improved using soft functions.
