# AS-Parallel-Serial
Simulink models and MATLAB functions associated with the manuscript "Anticipation from Sensation: Using Anticipating Synchronisation to Stabilise a System with Inherent Sensory Delay"

Contains two Simulink models that implement a simulation of a robot arm, along with control laws based
on anticipating synchronisation (AS), which were used to generate the results shown in the manuscript "Anticipation
from Sensation: Using Anticipating Synchronisation to Stabilise a System with Inherent Sensory Delay".

Both models share a series of constants, which are labelled within the Simulink view, and allow the alteration of the robot's
dynamics (link lengths and link masses), as well as the parameters of the control law (gains, coordinate transformation,
strength of AS coupling).

The MATLAB functions AnticiStep and AnticiStepDisturbance plot the anticipation/lag of the arm's end effector motion with
respect to the target, as well as the tracking error at that time difference. The result is plotted as a function of 
delay (feedback delay only or feedback and target delay) and coupling strength.
