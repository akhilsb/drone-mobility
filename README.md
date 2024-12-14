# Package for simulating Vega on NS3

These are the files used to produce the simulation results using NS3. 
To utilize these files, you will need to download ns3.33: https://www.nsnam.org/releases/ns-3-33/download/

Additionally, follow the setup instructions provided here:
https://www.nsnam.org/releases/ns-3-33/documentation/

I recommend installing on a linux machine and use a mix of pip and conda to set up the environment. 

Steps for using files:
1. Once ns3 is downloaded and the environment set up, place these files directly in src/scratch.
2. Build the c files by utilizing waf
3. Run `constant-time-circular-motion.cc` also using ./waf

Available run parameters:
- `mode`:
  0= DroneZoom 
  1= DroneCycle
  2= Hybrid
- `seed`: random seed for producing different runs of the same simulation
- `start_x`: x coordinate to start (0,800)
- `start_y`: y coordinate to start (0,800)
- `start_z`: Starting height of drone (50,400)
- `dz_hu`: If running hybrid mode, this value will be used to determine the mid-point of where drone zoom would run and `hb`
- `rate`: rate of arrival of targets in a poisson process
- `radius`: optional. If not provided, then will be determined by using starting height and the bounds of the simulation, (0,800)

Visible area of the drone is determined using the starting position (x,y,z) and a 4:3 aspect ratio of the drone camera
