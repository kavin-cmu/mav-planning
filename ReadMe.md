# A modular OMPL-based Navigation Stack for UAVs and MAVs (WIP)

Aim of this self-learning project was to develop a flexible and modular navigation stack enabling its use for variety of applications and sensor configurations.

Salient features:
- OMPL-based global planners : enables choice of a wide variety of sampling-based motion planners which have been tested extensively
- Modular Map Interface: provides a modular global and local map interface, enables use of a wide variety of map representations like OctoMap, OpenVDB, VoxelGrid etc. Local map interfaces can use SDFs for optimization based trajectory generation, presently experimenting with OctoMap's DynamicEDT3D library.
- Flexible collision-checking: adopted FCL to enable support for a wide range of robot geometry/map representations and abstracts the low-level collision check queries

##NOTE: This package is still a work in progress and hence also has limited documentation