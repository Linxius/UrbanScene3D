# Capturing, Reconstructing, and Simulating: the UrbanScene3D Dataset

This is the official repository of our ECCV22 dataset UrbanScene3D. Please refer to our [project page](https://vcc.tech/UrbanScene3D/) for more information.

The whole dataset can be download with [Dropbox](https://www.dropbox.com/sh/8g2urrij2fercko/AABi0GclI-f96uYsAdP0D0Yga?dl=0), [Google Drive](https://drive.google.com/drive/folders/1e91lEw56DUBbQgRTo48T3lVjo53SzEOd?usp=sharing), or [Baidu Yun](https://pan.baidu.com/s/1nqurXpbMzFo_-Cmf6eheOw?pwd=7zdg).

## Simulator
All the synthetic scene and the reconstruction of real scenes are included in the simulator.
Develop your own [Unreal Engine](https://www.unrealengine.com/) program to play with these scenes or
install the [AirSim](https://microsoft.github.io/AirSim/apis/) plugin to capture your desired data.

## Path format
```
image_name,x,y,z,pitch,roll,yaw
```
The provided path files in our dataset are in UE coordinate system, a left-handed, Z-Up coordinate system.

Each capturing of synthetic scenes comes with a path file. The captured images of real scenes are recorded with GPS information.

## Image capturing
For your convenience,
we provide seperate image capturing programs (exe) in a [zip file](https://www.dropbox.com/sh/pw09ebaa6k4phzr/AABsXdqRusZp7WEtQ7qWledOa?dl=0) for the four synthetic scenes in the benchmark. 

Setup the path file and the directory of output files in `./cg_3_zuizhong/Saved/Config/WindowsNoEditor/Game.ini`:
```ini
[targectory_path]
Path=path_to_path_file

[capture_path]
Path=direcotry_of_output_images
```
and run `cg_3_zuizhong.exe` to capture images.

## UrbanScene3D-V1
The data of UrbanScene3D version 1 are available at
[Dropbox](https://www.dropbox.com/sh/mg8pvzwmufpfug3/AADK2C8Zrtlf73tNyUvOJJCka?dl=0).

or download seperately from our nas (more friendly for Chinese users, but maybe (quite) unstable sometimes):
- [Residence](http://szuvccnas.quickconnect.cn/d/s/lSvWkTMbFjecrEwZDx3cV72M5scS2tKA/OxnMJCCChFCGAqEHfVC09VJmO_f-qrga-_LFAaeS27Ag)
- [Campus](http://szuvccnas.quickconnect.cn/d/s/lRrBh8QyqmVQnXgn6Lc41vqnpeZej5bm/Xj3MGE2nOmr9CR_q09lJzYzmtcUGc5XQ-67Hgr9-27Ag)
- [Sci-Art](http://szuvccnas.quickconnect.cn/d/s/lT61obCnx48mOc1FrPtUiuZ8eNCOrEQd/27C8eKMNd1YBpLxJTbYY-jMWU7vRHhbs-5bHAJ9227Ag)
- [Square](http://szuvccnas.quickconnect.cn/d/s/lTcdgzIR95FcFWgXkDBe92EyyjqMHsLy/8fIBdxxvlvckRk3puqWRPlFzG1-BDsU1-27Hgxdq27Ag)
- [Hospital](http://szuvccnas.quickconnect.cn/d/s/lTGZSjPziNZmEUIXnEt8uuT8RyoU44Xg/2RM7OW3HnC_1qDXzsJWXi6QN94DsSc3H-tbHgAMG27Ag)

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


## Copyright
UrbanScene3D is publicly accessible for non-commercial uses only. Permission is granted to use the data only if you agree:
- The dataset is provided "AS IS". Despite our best efforts to assure accuracy, we disclaim all liability for any mistakes or omissions;
- All works that utilize this dataset including any partial use must include a reference to it. Please correctly cite our ECCV22 publication in research papers using the information provided below;
- You refrain from disseminating this dataset or any altered variations;
- You are not permitted to utilize this dataset or any derivative work for any commercial endeavors;
- We reserve all rights that are not explicitly granted to you.

## BibTeX
```
@inproceedings{UrbanScene3D,
title={Capturing, Reconstructing, and Simulating: the UrbanScene3D Dataset},
author={Liqiang Lin and Yilin Liu and Yue Hu and Xingguang Yan and Ke Xie and Hui Huang},
booktitle={ECCV},
year={2022},
}

```