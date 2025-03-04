Operational Space Control Example - Articulated Wheel-leg UGV
Christian Hubicki
2022 May 17

NOTE:
Model parameters are stored in "getVehicleParams.m:
Angles are CCW positive.
From double pendulum example: theta_1 is relative to horizontal. theta_2 is
 relative to theta_1.

Instructions:
1) Run: "genVehicleFunctions.m"
It will create a folder called "auto" containing files relevant
for the dynamics and the QP.
**Rerun this whenever system parameters are changed in "getParams.m"**
2) Run: "runOSCVehicle.m"
--------------------------------------------------------------------------

2023 Sep 07.
Rolling on horizontal terrain using auxiliary coordinates. 
Only one rolling wheel.
--------------------------------------------------------------------------


--------------------------------------------------------------------------
Appendix
Parent:
ORL research\Articles - author\2023.02 IROS Rolling on curves\Code & Results\From CH\\2024-02-01 2 wheels OSC - CH-v2-wheels-up