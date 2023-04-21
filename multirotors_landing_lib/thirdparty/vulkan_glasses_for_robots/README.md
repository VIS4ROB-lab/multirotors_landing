# Vulkan Glasses for Robots
This is library based on [Vulkan API](https://www.vulkan.org/) written by Lucas Teixeira, Vision for Robotics Lab, ETH Zurich.

An example on how to use the Vulkan renderer is in `src/example`.

## Pipeline explanation
The Vulkan renderer is a standard rendering pipeline, with just a few modification. In the following, we detail the rendering process.

1. When it is started, the renderer loads in memory the `obj` of the chosen model and the associated textures.
2. The textures are in the form of a `png` file with 4 channels: `R`, `G`, `B`, `alpha`. The renderer reads the `alpha` channel as semantic value ([here](https://github.com/VIS4ROB-lab/multirotors_landing/blob/main/multirotors_landing_lib/include/multirotors_landing_lib/common/semantics_classes.hpp) the mapping from pixel value to semantic class can be found), and generate the semantic image. Semantics are generated in real-time by reading the texture file and projecting it onto the 3D model.
3. To render RGB images at a given pose, the renderer reads the other channels of the texture file.
4. To render depth at a given pose, it peforms standard raycasting on the `obj`.
5. These images (rgb, depth, semantics) are then passed to the policy.
