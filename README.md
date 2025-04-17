## CLVSim
## The code of paper ["CLVSim: A comprehensive framework for crewed lunar vehicle simulation—Modeling and applications"](https://doi.org/10.1002/rob.22421).

Crewed lunar vehicle simulation (CLVSim) is a MBD-FEM-SPH co-simulation framework to simulate crewed lunar vehicle in off-road environment based on [Project Chrono](https://projectchrono.org/), including SPH soft terrain, FEM flexible wheel, vehicle suspension, motor, and driver. Each subsystem was modelled can benchmarked by some specific tests. 

A high-fidelity instance of CLVSim was modelled and validated based on [Apollo LRV](https://www.nasa.gov/history/alsj/lrvhand.html) and experimental data from [Apollo operation handbook](https://www.lpi.usra.edu/lunar/documents/NTRS/collection2/NASA_TM_X_66816.pdf) released by NASA. LRV was modelled and textured in this project.

![image](https://github.com/user-attachments/assets/69a7c46b-9a7f-4277-b66c-80d06520630e)


## Features
- Coupled MBD–SPH–FEM simulation framework  
- Mechanical components modeled directly from [LRV engineering drawings](https://www.lpi.usra.edu/lunar/documents/NTRS/collection2/NASA_TM_X_66816.pdf)  
- Separate benchmark of soft terrain, flexible wheels, suspension, and motor against experimental data  
- Calibration of soft-terrain model via single-wheel simulation and experimental testing  
- Whole vehicle benchmarked via low-gravity simulation and experimental testing  
- Independent drive for each of the four wheels, powered by four dedicated motors  
- Motor output characterized using multi‑layer perceptron (MLP) model  
- Support for high‑quality rendering  

## Demos
The rendered simulation by [Blender](https://www.blender.org/) for LRV rovering on straight rugged terrain on Moon:

https://github.com/user-attachments/assets/0b5244df-9bf9-4473-8278-dbd926b0f719

It can also import any terrain, such as a rugged lunar surface with planned path. The rendered simulation with surface reconstruction using [OpenVDB](https://github.com/AcademySoftwareFoundation/openvdb) or [splashsurf](https://github.com/InteractiveComputerGraphics/splashsurf):

https://github.com/user-attachments/assets/22d2901b-0101-4f3e-b249-0d69af256b61

### Install

1. install [Project Chrono](https://projectchrono.org/), VSG, FSI, MKL modules are required
2. build CLVSim (Copy .dll files from Project Chrono)
3. unzip data.7z and put as build/data

