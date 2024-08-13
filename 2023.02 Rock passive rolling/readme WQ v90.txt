Simulation of a wheel rolling on various terrain.
2023 February 15. Adwait Mane.


User defined parameters in getParameters_v*.m: 
* Terrain geometry.
* par.p_start is the start location on the terrain.
* par.t_end is the sim duration.
Check that the correct version of getParameters_v*.m is loaded in the 
other scripts.

Run genFunctions_v*.mlx. This derives the augmented dynamics symbolically
and exports them as numerical functions in the 'auto' folder.
It is a Live Script which generates typeset math output. I suggest using
the 'output inline' option from the 'View' menu for better readability.

main_*_v*.m:
Specify the desired filename.
E.g.: filename = 'v90' saves the output as v90.mat.
Run this script to simulate the system and save the output.

animate_v*.m. 
Select the data file to read e.g. load('v90.mat').
Select the camera position.
------------------------------------------------------------------------

Update the terrain function in Functions/terrain.m.
Re-run genFunctions_v*.mlx if the terrain type or function is changed.

Update the controller in Functions/get_u_aug.m.

You need to re-run genFunctions_v*.mlx if par.p_start is updated because
this script calculates the initial position for the remaining coordinates.
------------------------------------------------------------------------

Wednesday 2023 Feb 15. WQ v90. Parent: WQ v30. 
Refactored. All user-defined options in getParameters_v*.m.
Animation script automatically uses correct axes limits.
------------------------------------------------------------------------

Wednesday 2023 Dec 20.
Error. Missing findIC function. findIC from the rock stabilization code 
does not work.
------------------------------------------------------------------------

