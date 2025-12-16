# Optimization RGB-D Three-dimensional Reconstruction Algorithm Based on Dynamic SLAM

## Overview

This project is an enhanced version of [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) that integrates dynamic object detection using YOLO and real-time point cloud reconstruction. The system can detect and exclude dynamic objects (e.g., people) during SLAM to improve mapping accuracy and robustness.

**This work is based on ORB-SLAM2 and has been published in:**

> **Optimization RGB-D Three-dimensional Reconstruction Algorithm Based on Dynamic SLAM**  
> 2022 IEEE Conference  
> [Paper Link](https://ieeexplore.ieee.org/document/10050782)

## Demo

![Dynamic SLAM Demo](dynamic-slam.gif)

The demo shows:
- **Real-time person detection** with bounding boxes (red rectangles)
- **Feature point extraction** excluding detected person regions
- **Live 3D point cloud reconstruction** with dynamic objects excluded
- **Visualization** of the mapping process

## Features

- **Dynamic Object Detection**: Integrates YOLOv5 for real-time person detection
- **Feature Point Filtering**: Automatically excludes feature points within detected person bounding boxes
- **Point Cloud Mapping**: Real-time 3D point cloud reconstruction with PCL
- **Dynamic Object Exclusion**: Removes dynamic objects from point cloud reconstruction
- **Volume-based Clustering**: Two-stage filtering (local and global) to remove residual dynamic objects based on cluster volume
- **Visualization**: Real-time visualization of detection boxes and point cloud maps

## Dependencies

### System Requirements

- **C++14** or higher compiler (GCC 5.4+ or Clang 3.4+)
- **CMake** >= 2.8
- **Linux** (Ubuntu 16.04 or higher recommended)

### Required Libraries

1. **OpenCV** (>= 2.4.3, with **opencv_contrib**)
   ```bash
   # Install OpenCV with contrib modules
   # For Ubuntu:
   sudo apt-get install libopencv-dev libopencv-contrib-dev
   
   # Or compile from source:
   # Download OpenCV and opencv_contrib from https://github.com/opencv/opencv
   # Follow OpenCV installation guide with contrib modules enabled
   ```

2. **Eigen3** (>= 3.1.0)
   ```bash
   sudo apt-get install libeigen3-dev
   ```

3. **Pangolin**
   ```bash
   # Install dependencies
   sudo apt-get install libglew-dev libpython2.7-dev libffi-dev libssl-dev
   
   # Clone and build Pangolin
   git clone https://github.com/stevenlovegrove/Pangolin.git
   cd Pangolin
   mkdir build && cd build
   cmake ..
   make -j4
   sudo make install
   ```

4. **PCL** (Point Cloud Library, >= 1.7)
   ```bash
   sudo apt-get install libpcl-dev
   ```

5. **DBoW2** and **g2o**
   - Included in `Thirdparty/` directory
   - Will be compiled automatically

### YOLO Model Files

Place YOLOv5 ONNX model files in the project root directory:
- `yolov5s.onnx` (recommended for faster inference)
- `yolov5m.onnx`
- `yolov5l.onnx` (default, used in code)
- `yolov5x.onnx`

You can download pre-trained YOLOv5 models from [YOLOv5 releases](https://github.com/ultralytics/yolov5/releases) and convert them to ONNX format.

Also required:
- `coco.names` - COCO dataset class names file (should be in project root)

## Building

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd
   ```

2. **Build DBoW2:**
   ```bash
   cd Thirdparty/DBoW2
   mkdir build && cd build
   cmake ..
   make -j4
   cd ../../..
   ```

3. **Build g2o:**
   ```bash
   cd Thirdparty/g2o
   mkdir build && cd build
   cmake ..
   make -j4
   cd ../../..
   ```

4. **Build SLAM:**
   ```bash
   mkdir build
   cd build
   cmake ..
   make -j4
   ```
   ```

## Configuration

### OpenCV Path

If OpenCV is installed in a custom location, update the path in `CMakeLists.txt`:
```cmake
set(OpenCV_DIR /path/to/opencv/build)
```

### Camera Parameters

Configure your camera parameters in the YAML configuration file (e.g., `Examples/RGB-D/TUM1.yaml`):
```yaml
Camera.fx: <focal_length_x>
Camera.fy: <focal_length_y>
Camera.cx: <principal_point_x>
Camera.cy: <principal_point_y>
```

For point cloud mapping, add:
```yaml
PointCloudMapping.Resolution: 0.04  # Voxel grid resolution in meters
```

## Usage

### RGB-D Example

```bash
./bin/rgbd_tum Vocabulary/ORBvoc.bin Examples/RGB-D/TUM3.yaml /path/to/rgbd/dataset /path/to/association/file
```

## Key Modifications from ORB-SLAM2

1. **YOLO Integration**: Added YOLOv5 for real-time object detection
2. **Feature Point Filtering**: Modified `ORBextractor` to exclude features in detected person regions
3. **Point Cloud Mapping**: Integrated PCL for real-time 3D reconstruction
4. **Dynamic Object Exclusion**: 
   - Feature points in person bounding boxes are not extracted
   - Point cloud points in person regions are excluded
5. **Volume-based Clustering**: Two-stage filtering to remove residual dynamic objects
6. **Visualization**: Detection boxes are drawn on the frame for visualization

## File Structure

```
.
├── Examples/          # Example applications
├── include/           # Header files
├── src/               # Source files
│   ├── yolo.cpp       # YOLO detection implementation
│   └── pointcloudmapping.cc  # Point cloud mapping
├── Thirdparty/        # DBoW2 and g2o libraries
├── Vocabulary/        # ORB vocabulary file
├── CMakeLists.txt     # Build configuration
└── README.md          # This file
```

## Citation

If you use this code in your research, please cite:

```bibtex
@inproceedings{orb_slam2_dynamic_2022,
  title={Optimization RGB-D Three-dimensional Reconstruction Algorithm Based on Dynamic SLAM},
  author={...},
  booktitle={2022 IEEE Conference},
  year={2022},
  doi={10.1109/...},
  url={https://ieeexplore.ieee.org/document/10050782}
}
```

And the original ORB-SLAM2 paper:
```bibtex
@article{mur_artal_2017,
  title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
  author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
  journal={IEEE Transactions on Robotics},
  volume={33},
  number={5},
  pages={1255--1262},
  year={2017}
}
```

## License

This project is licensed under the GPLv3 license - see the [LICENSE.txt](LICENSE.txt) file for details.

## Acknowledgments

- [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) by Raúl Mur-Artal et al.
- [YOLOv5](https://github.com/ultralytics/yolov5) by Ultralytics
- [PCL](https://pointclouds.org/) Point Cloud Library

## Troubleshooting

### OpenCV not found
- Ensure OpenCV is compiled with `opencv_contrib` modules
- Set `OpenCV_DIR` in CMakeLists.txt to your OpenCV build directory

### PCL compilation errors
- Ensure PCL >= 1.7 is installed
- Check that C++14 standard is enabled (automatically handled by CMakeLists.txt)

### YOLO model not found
- Ensure YOLOv5 ONNX model files are in the project root
- Check that `coco.names` file exists in the project root

### Namespace conflicts
- The code handles OpenCV/VTK namespace conflicts by including PCL headers before OpenCV headers

## Contact

For questions and issues, please open an issue on GitHub.
