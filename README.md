# Capturing, Reconstructing, and Simulating: the UrbanScene3D Dataset

This is the official repository of our ECCV22 dataset UrbanScene3D. Please refer to our [project page](https://vcc.tech/UrbanScene3D/) for more information.

## Evaluation 
This reposity contains the source code of evaluation reconstructed models used in the [UrbanScene3D](https://vcc.tech/UrbanScene3D/) for your reference. Please download the [complied version](https://github.com/Linxius/UrbanScene3D/releases/download/v0.0.1/Evaluation.zip) for windows:x64 with real scene evaluation data.

### Install

dependency:
- [vcpkg](https://github.com/microsoft/vcpkg) for C++ library (set your proxy environment variables for boosting download speed)

install C++ libraries with vcpkg for example in bash 
```
vcpkg install --triplet=x64-windows boost eigen3 cgal tinyply tinyobjloader embree3 glog glm opencv tinyxml2 pthread
```

### Usage

with gt_points and recon_points sampled on gt models and reconstructed models:
```
path_to_exe path_to_gt_points path_to_recon_points
```

## Simulator
The [Unreal Engine](https://docs.unrealengine.com/en-US/index.html) simulator with all the virual and real scenes can be downloaded at: [Baidu](https://pan.baidu.com/s/1ROx5LV7tG3w9FQxddu3yMA?pwd=ggjw).
Install the [AirSim](https://microsoft.github.io/AirSim/apis/) plugin to capture your desired data.


## BibTeX
```
@inproceedings{UrbanScene3D,
title={Capturing, Reconstructing, and Simulating: the UrbanScene3D Dataset},
author={Liqiang Lin and Yilin Liu and Yue Hu and Xingguang Yan and Ke Xie and Hui Huang},
booktitle={ECCV},
year={2022},
}

```