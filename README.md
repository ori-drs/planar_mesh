# PlanarMesh

**Project Website:** [dynamic.robots.ox.ac.uk/projects/planarmesh/](https://dynamic.robots.ox.ac.uk/projects/planarmesh/)

PlanarMesh is a novel, incremental, mesh-based LiDAR reconstruction system. It adaptively adjusts the mesh resolution to create compact and detailed 3D reconstructions in real-time. By combining plane modeling and meshing, it can capture both large, flat surfaces and detailed geometry. The mesh is incrementally updated, considering local surface curvature and free-space information from the LiDAR sensor.

The system uses a multi-threaded architecture with a Bounding Volume Hierarchy (BVH) for efficient data storage and fast search operations, which allows it to achieve real-time performance. Compared to state-of-the-art techniques, PlanarMesh produces significantly smaller output files while maintaining comparable or superior reconstruction accuracy.

## Citation

If you use PlanarMesh in your research, please cite the following publication:

```bibtex
@inproceedings{wang2025planarmesh,
  title={PlanarMesh: Building Compact 3D Meshes from LiDAR using Incremental Adaptive Resolution Reconstruction},
  author={Wang, Jiahao and Chebrolu, Nived and Tao, Yifu and Zhang, Lintong and Kim, Ayoung and Fallon, Maurice},
  booktitle={IEEE/RSJ Intl. Conf. on Intelligent Robots and Systems (IROS)}, 
  year={2025},
}
```

**Paper:** [https://arxiv.org/abs/2510.13599](https://arxiv.org/abs/2510.13599)

## Dependencies

The project requires the following dependencies to be installed on your system:

*   **C++17** compatible compiler
*   **CMake** (version 3.10 or higher)
*   **Point Cloud Library (PCL)** (version 1.14)
*   **Computational Geometry Algorithms Library (CGAL)**
*   **OpenMP**
*   **Git LFS**

## Build Instructions

1.  Clone the repository and fetch sample data:
    ```bash
    git clone <repository-url>
    cd PlanarMesh
    git lfs install
    git lfs pull
    ```

2.  Create a build directory and navigate into it:
    ```bash
    mkdir build
    cd build
    ```

3.  Run CMake to configure the project and then build it (replace `8` with the number of CPU cores to speed up compilation):
    ```bash
    cmake ..
    make -j8
    ```

4.  The executable will be located in the `build` directory:
    ```bash
    ./main
    ```

## Running the Application

### Dataset Configuration

All configuration, including the path to the dataset, is hardcoded in `src/MeshObject/Settings.cpp`. By default, the application is configured to use the `sample` dataset included in the repository (via Git LFS).

To run the application with your own data, you must modify this file and recompile the project.

1.  **Open the settings file:** `src/MeshObject/Settings.cpp`.
2.  **Add a new dataset entry:** Inside the `Settings::Settings()` constructor, locate the `dataset_map`. Add a new entry with a unique key for your dataset. You need to provide the path to your point cloud file folder (`pcd_file_folder`) and the corresponding pose file (`pose_file_path`).

    ```cpp
    // Example of adding a new dataset
    dataset_map["my_awesome_dataset"] = {
        "/path/to/your/pcd_files/", // pcd_file_folder
        "/path/to/your/pose_file.txt"  // pose_file_path
    };
    ```

3.  **Select your dataset:** Change the value of the `dataset` string variable to the key you just added.

    ```cpp
    // Set the active dataset
    std::string dataset = "my_awesome_dataset";
    ```

4.  **Recompile:** Save the file, navigate to your `build` directory, and run `make`.

### Execution Modes

The application can be run in two modes, controlled by the `headless_mode` boolean in `src/MeshObject/Settings.cpp`.

*   **Display Mode (`headless_mode = false;`):** This is the default mode. It launches an interactive PCL viewer where you can visualize the mesh generation process in real-time and control the application using keyboard commands.
*   **Headless Mode (`headless_mode = true;`):** This mode runs the entire processing pipeline without a GUI. The final mesh is automatically saved to disk upon completion.

### Multithreading

Due to an immature implementation, deadlocks may occur when using multithreading. A deadlock will typically manifest as the PCL visualizer freezing during processing.

It is recommended to use a higher number of threads, as this appears to reduce the likelihood of a deadlock. For context, the benchmarking in the original paper was performed on a machine with 28 cores. The more cores available, the less likely the application is to deadlock. The number of threads can be configured in `src/MeshObject/Settings.cpp`.

## Interactive Viewer Keybindings

When running in display mode, you can use the following keybindings in the PCL viewer window:

| Key(s)         | Action                                     |
| :------------- | :----------------------------------------- |
| `space`        | Process one frame of data.                 |
| `1`, `2`, `3`, `4` | Process 10, 100, 1000, 10000 points.      |
| `6`, `7`, `8`    | Process 10, 100, 1000 frames.              |
| `0`            | Process all remaining data.                |
| `r`            | Restart the processing from the beginning. |
| `,`            | Toggle visibility of the point cloud.      |
| `.`            | Toggle visibility of mesh edges.           |
| `/`            | Toggle visibility of mesh triangles.       |
| `v`            | Toggle wireframe rendering.                |
| `l`            | Toggle visibility of interior points.      |
| `numpad 0-8`   | Cycle through different color modes.       |
| `9`            | Export the current view to PLY files.      |
| `b`            | Save a simplified version of the mesh.     |

## Configuration Settings

The `src/MeshObject/Settings.cpp` file contains numerous other parameters for controlling the meshing algorithm, visualization options, and logging behavior. You can modify these values before compilation to experiment with different configurations. The corresponding `include/MeshObject/Settings.hpp` header file provides a reference for all available settings.

## Code Status

Please be aware that not all of the code in this repository is actively used by the final application. The codebase contains some legacy and development code that was part of earlier experiments and was subsequently abandoned.