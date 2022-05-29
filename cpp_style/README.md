# RGBD_SLAM

The `RGBD-SLAM` is a step by step building of the SLAM algorithm using input dataset of RGB-D images.

The software is tested on Ubuntu 20.04 with C++14.

## Dependencies
Most of the dependencies are standard including `Eigen3`, `OpenCV 4`, `PCL`.
* `sudo apt install libpcl-dev`
* `sudo apt-get install libopencv-dev` 
* `sudo apt-get install libeigen3-dev`

## Compling
The software is a standard CMake project. After cloninf the normal procedure for compiling a package should work which is as follows:-
1. Compile third-party libs, including DBoW2 (for loop closure), a modified version of g2o (for solving pnp), and the OrbExtractor from [orb-slam2] (https://github.com/raulmur/ORB_SLAM2).
  * They are all cmake projects, so just go into the directory and remove the build and lib folders and then type
    ```
    mkdir build
    cmake ..
    make -j2
    ```
  * For g2o you need to do the following else FindG2O.cmake will not work.
    ```
    mkdir build
    cmake ..
    sudo make install
    ```
  2. Then compile the project as follows.
  
  * Place the dataset correctly. Refer to [README](https://github.com/Rajat-Arora/RGBD-SLAM/tree/main/cpp_style/dataset) in dataset folder.
  * In root directory:
    ```
    mkdir build bin
    cd build
    cmake ..
    make
      ```
   *  Run the binaries from the bin folder
  
## Experiments
The experiments folder contains various experiments shown as follows:
1. `reading_frame.cpp` 
  * It helps to test three classes `parameterReader`, `rgbdFrame` and `frameReader`.
  * It instantiate the object of the frameReader class and reader all the frames by calling the next method.
   ![Output](https://user-images.githubusercontent.com/97186785/170851842-fb3cf952-6902-4a4a-ac03-f533f50670b7.gif)

   



