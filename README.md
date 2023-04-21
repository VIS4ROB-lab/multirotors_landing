# Autonomous Emergency Landing for Multicopters

__Author__: Luca Bartolomei  
__Affiliation__: Vision For Robotics Lab, ETH Zurich  
__Contact__: Luca Bartolomei, lbartolomei@ethz.ch  

**Disclaimer**: Some numerical results can be different from the ones in the paper because we are using more recent versions of some python libraries (e.g. expect different numbers of training steps and reward values than the paper). However, performances at test time should be close to the published ones.

## Citation

If you use this code in your academic work, please cite [[PDF](https://www.research-collection.ethz.ch/handle/20.500.11850/605990)]:

    @inproceedings{bartolomei2022multi,
      title={Autonomous Emergency Landing for Multicopters using Deep Reinforcement Learning},
      author={Bartolomei, Luca and Kompis, Yves and Teixeira, Lucas and Chli, Margarita},
      booktitle={2022 {IEEE/RSJ} International Conference on Intelligent Robots and Systems ({IROS})},
      year={2022}
    }

## License
This project is released under a [GPLv3 license](.aux/license_gpl.txt).

## Video
<div href="https://www.youtube.com/watch?v=p8tpLL7Q0GE" target="_blank" align="center"><img src="https://img.youtube.com/vi/p8tpLL7Q0GE/0.jpg" alt="Mesh" width="480" height="360" border="0" /></div>

## Presentation at IROS 2022
<div href="https://www.youtube.com/watch?v=wOxZIGdsINs" target="_blank" align="center"><img src="https://img.youtube.com/vi/wOxZIGdsINs/0.jpg" alt="Mesh" width="480" height="360" border="0" /></div>

## Installation
So far, the pipeline has been tested with **python 3.6** and **Ubuntu 20.04 LTS**.

1. Install the following dependencies:
	```
	$ sudo apt update && sudo apt install build-essential cmake
	$ sudo add-apt-repository ppa:deadsnakes/ppa 
	$ sudo apt update && sudo apt install python3.6-dev python3.6-venv
	$ sudo apt install libeigen3-dev libyaml-cpp-dev libopencv-dev libpcl-dev liboctomap-dev libgoogle-glog-dev libglm-dev libvulkan-dev
	```

2. Clone the repo:
	```
	$ git clone git@github.com:VIS4ROB-lab/multirotors_landing.git
	```

3. Naigate to the main library folder:
	```
	$ cd multirotors_landing_lib
	```

4. At this point, it is strongly recommended to create a __virtual environment__:
	```
	$ python3.6 -m venv ~/venvs/landing/
	```

5. Build the project with `pip` (note: building of `opencv-python` can take a while):
	```
	$ source ~/venvs/landing/bin/activate
	$ pip install --upgrade pip wheel
	$ pip install opencv-python==4.5.2.54
	$ export ML_PATH=path_to_multirotors_landing >> ~/.bashrc  # Path without final "/"
	$ pip install .
	```

### Building without Python bindings
To build without Python bindings and use only the C++ library, run the following commands:
```
$ cd multirotors_landing_lib
$ mkdir build && cd build
$ cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPYTHON_EXECUTABLE=/usr/bin/python3.6 && make -j8
```

## 3D Model
The 3D models can be found [here](https://www.polybox.ethz.ch/index.php/s/UOU4EbZEEmTdvSO). Download them, and unzip the file in the folder `multirotors_landing/meshes`.

This can be done using the command line as follows:
```
$ cd multirotors_landing
$ mkdir meshes && cd meshes
$ wget https://www.polybox.ethz.ch/index.php/s/UOU4EbZEEmTdvSO/download
$ unzip download
```

**Note**: The models __need__ to be stored in `multirotors_landing/meshes`, as this is the coded expected location.

Every model is composed of three files:
1. `*.obj`: The 3D model geometry. This is used by the Vulkan-based renderer.
2. `*_rgbs.png`: The texture of the model. The first three channels of this image are the actual texture, while the alpha channel contains the semantic labels. This is used by the Vulkan-based renderer.
3. `*.ply`: Point cloud of the model used for collision checking during training. An OctoMap is generated from this file.

More information about the rendering pipeline can be found [here](./multirotors_landing_lib/thirdparty/vulkan_glasses_for_robots/README.md).

## Running instructions

The configurations for training and testing are stored in [multirotors_landing_lib/config](/multirotors_landing_lib/config).

### Policy Training

To train the agent using ground-truth semantics and depth maps (use flag `-h` for more information):
```
$ cd multirotors_landing/multirotors_landing_rl
$ pip install -e .
$ python3 scripts/quadrotor_landing.py
```

Note the following:
 * When training start, you should see 3 images from the drone's perspective (RGB, depth, semantics) if the parameter `common/visualization` is set to `true` in the configuration file. 
 * To use more than one environment, adjust the number of environments and threads in the [configuration file](/multirotors_landing_lib/config/quad_landing.yaml).
 * The results of the training are stored in `multirotors_landing/experiments`, and the folder name is the timestamp at the start of the training.

### Policy Testing

To plot the results from training (use flag `-h` for more information):
```
$ python3 scripts/plot_training.py -f ${path_to_training_folder}
```

To find the best policy from a training run:
```
$ python3 scripts/find_best_policy_weights.py -f ${path_to_training_folder}
```

To test a trained policy (use flag `-h` for more information):
```
$ python3 scripts/quadrotor_testing.py -w ${path_to_weights}
```

## Socket Communication

**Note**: In the current version of the code, communication sockets are __not__ used, as we use the ground-truth images from the Vulkan-based renderer. However, we include the library we implemented for communication with semantic segmentation and depth completion neural networks for completeness. Unforuntaly, the networks we used in the paper are closed-source, so we cannot provide them.

**Note**: The provided examples can be used if the C++ code is built manually.

In [this folder](./multirotors_landing_lib/tests/communication) examples on how to use socket-based communication are provided. It is possible to run a series of examples, where different types of information are exchanged between a server and a client (e.g. strings, vectors, images). The provided sockets can be used to interface the Vulkan-based renderer with any custom neural network.

1. Communication between client and server via raw sockets:

		$ ./multirotors_landing_lib/build/server_socket # Terminal 1 -- server
		$ ./multirotors_landing_lib/build/client_socket # Terminal 2 -- client

2. Communication between client and server via communicators (which provide a more complete interface than raw sockets):

		$ ./multirotors_landing_lib/build/server_communicator # Terminal 1 -- server
		$ ./multirotors_landing_lib/build/client_communicator # Terminal 2 -- client

2. Python interface, with exchange of RGB and grayscale images:

		$ python3 multirotors_landing_lib/tests/communication/python/socket_server.py # Terminal 1 -- server
		$ ./multirotors_landing_lib/build/python_socket_client # Terminal 2 -- client
	
## Contributing
Contributions that help to improve the code are welcome. In case you want to contribute, please adapt to the [Google C++ coding style](https://google.github.io/styleguide/cppguide.html) and run `bash clang-format-all .` on your code before any commit.

In particular, the nice-to-have list includes:
* Use more recent python version and libraries
* Interface the training and testing environments with open-source semantic nets
