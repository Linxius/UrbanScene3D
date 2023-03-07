# Capturing, Reconstructing, and Simulating: the UrbanScene3D Dataset

This is the official repository of our ECCV22 dataset UrbanScene3D. Please refer to our [project page](https://vcc.tech/UrbanScene3D) for more information.

The whole dataset can be downloaded with [Dropbox](https://www.dropbox.com/sh/8g2urrij2fercko/AABi0GclI-f96uYsAdP0D0Yga?dl=0), [Google Drive](https://drive.google.com/drive/folders/1e91lEw56DUBbQgRTo48T3lVjo53SzEOd?usp=sharing), or [Baidu Yun](https://pan.baidu.com/s/1nqurXpbMzFo_-Cmf6eheOw?pwd=7zdg).

## Simulator
All the synthetic scenes and the reconstruction of real scenes are included in the simulator.
Develop your [Unreal Engine](https://www.unrealengine.com/) program to play with these scenes or
install the [AirSim](https://microsoft.github.io/AirSim/apis/) plugin to capture your desired data.
(Airsim uses PlayerStart as the origin.)

Open UrbanScene.uproject and rebuild the project. All the scenes are located in Contents/Maps in the Content Browser.

The models of the large-scale virtual scenes are not provided due to the copyright issue and the textured meshes can not be exported from Simulator(UE). We provide [sampled points](https://github.com/Linxius/UrbanScene3D/releases/download/v0.0.1/UrbanScene3D-virtual_cities-sampled.7z) for them for evaluation purpose.

## Path format
```
image_name,x,y,z,pitch,roll,yaw
```
The provided path files in our dataset are in UE (world) coordinate system, a left-handed, Z-Up coordinate system (in cm).

Each capture of synthetic scenes comes with a path file and a captured image set. The captured images of real scenes are recorded with GPS information.

## Image capturing
For your convenience,
we provide separate image-capturing programs (in this [zip file](https://www.dropbox.com/sh/pw09ebaa6k4phzr/AABsXdqRusZp7WEtQ7qWledOa?dl=0)) for the four synthetic scenes in the benchmark. 

fov=60 resolution=6000*4000 for synthetic scenes. Real scenes come with different cameras, a camera calibration is needed to get the camera parameters accordingly.

Setup the path file and the directory of output files in `./cg_3_zuizhong/Saved/Config/WindowsNoEditor/Game.ini`:
```ini
[targectory_path]
Path=path_to_path_file

[capture_path]
Path=direcotry_of_output_images
```
and run `cg_3_zuizhong.exe` to capture images.

## Evaluation 
Please download the [compiled version](https://github.com/Linxius/UrbanScene3D/releases/download/v0.0.1/Evaluation.zip) for windows:x64 with real scene evaluation data.

```
path_to_exe p1 path_to_recon p3 path_to_gt
```
p1,p2: "mesh" or "points"

The contained `.pointcloud` in [Evaluation_data](https://github.com/Linxius/UrbanScene3D/releases/download/v0.0.1/Evaluation_data.zip) is only for the evaluation program. Delete triangles that don't belong to the buildings and translate the reconstructed model to the corresponding bbox to evaluate it.

bbox 
```
min_x, min_y, min_z
max_x, max_y, max_z
```

for PolyTech:
```
-16.4756 -12.6849 -2.9153
196.8354 165.3059 52.3150
```

for ArtiSci:
```
-604.3266 895.3274 16.4413
-358.6385 1056.4519 58.7454
```

## UrbanScene3D-V1
The data of UrbanScene3D version 1 are available on
[Dropbox](https://www.dropbox.com/sh/mg8pvzwmufpfug3/AADK2C8Zrtlf73tNyUvOJJCka?dl=0).

or download separately from our NAS (more friendly for Chinese users, but maybe (quite) unstable sometimes):
- [Residence](http://szuvccnas.quickconnect.cn/d/s/lSvWkTMbFjecrEwZDx3cV72M5scS2tKA/OxnMJCCChFCGAqEHfVC09VJmO_f-qrga-_LFAaeS27Ag)
- [Campus](http://szuvccnas.quickconnect.cn/d/s/lRrBh8QyqmVQnXgn6Lc41vqnpeZej5bm/Xj3MGE2nOmr9CR_q09lJzYzmtcUGc5XQ-67Hgr9-27Ag)
- [Sci-Art](http://szuvccnas.quickconnect.cn/d/s/lT61obCnx48mOc1FrPtUiuZ8eNCOrEQd/27C8eKMNd1YBpLxJTbYY-jMWU7vRHhbs-5bHAJ9227Ag)
- [Square](http://szuvccnas.quickconnect.cn/d/s/lTcdgzIR95FcFWgXkDBe92EyyjqMHsLy/8fIBdxxvlvckRk3puqWRPlFzG1-BDsU1-27Hgxdq27Ag)
- [Hospital](http://szuvccnas.quickconnect.cn/d/s/lTGZSjPziNZmEUIXnEt8uuT8RyoU44Xg/2RM7OW3HnC_1qDXzsJWXi6QN94DsSc3H-tbHgAMG27Ag)


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