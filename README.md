## CLVSim
## The code of paper ["CLVSim: A comprehensive framework for crewed lunar vehicle simulation—Modeling and applications"](https://doi.org/10.1002/rob.22421).

Crewed lunar vehicle simulation (CLVSim) is a MBD-FEM-SPH co-simulation framework to simulate crewed lunar vehicle in off-road environment based on [Project Chrono](https://projectchrono.org/), including SPH soft terrain, FEM flexible wheel, vehicle suspension, motor, and driver. Each subsystem was modelled can benchmarked by some specific tests. 

A high-fidelity instance of CLVSim was modelled and validated based on [Apollo LRV](https://www.nasa.gov/history/alsj/lrvhand.html) and experimental data from [Apollo operation handbook](https://www.lpi.usra.edu/lunar/documents/NTRS/collection2/NASA_TM_X_66816.pdf) released by NASA. LRV was modelled and textured in this project.

The rendered simulation by [Blender](https://www.blender.org/) for LRV rovering on straight rugged terrain on Moon:

https://github.com/user-attachments/assets/0b5244df-9bf9-4473-8278-dbd926b0f719

It can also import any terrain, such as a rugged lunar surface with planned path. The rendered simulation with surface reconstruction using [OpenVDB](https://github.com/AcademySoftwareFoundation/openvdb) or [splashsurf](https://github.com/InteractiveComputerGraphics/splashsurf):

https://github.com/user-attachments/assets/22d2901b-0101-4f3e-b249-0d69af256b61

### Install

1. install [Project Chrono](https://projectchrono.org/), VSG, FSI, MKL modules are required
2. build CLVSim (Copy .dll files from Project Chrono)
3. unzip data.7z and put as build/data

