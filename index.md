![](assets/images/dataset.png)
<center>A glance of synthetic (top) and real (bottom) scenes in UrbanScene3D.</center>

## <center>Abstract</center>
We present UrbanScene3D, a large-scale data platform for research of urban scene perception and reconstruction. 
UrbanScene3D contains over 128k high-resolution images covering 16 scenes including 
large-scale real urban regions and synthetic cities with 136 km<sup>2</sup> area in total. 
The dataset also contains high-precision LiDAR scans and hundreds of image sets with different observation patterns, 
which provide a comprehensive benchmark to design and evaluate aerial path planning and 3D reconstruction algorithms. 
In addition, the dataset, which is built on Unreal Engine and Airsim simulator together with 
the manually annotated unique instance label for each building in the dataset, 
enables the generation of all kinds of data, e.g., 2D depth maps, 2D/3D bounding boxes, and 3D point cloud/mesh segmentations, etc. 
The simulator with physical engine and lighting system not only produce variety of data
but also enable users to simulate cars or drones in the proposed urban environment for future research.

![](assets/images/simulator.jpg)
UrbanScene3D also provides the building instance ID for each environment (top left), 
4K aerial videos that are aimed at the real scene acquisition (top right), 
and a simulator built on Unreal Engine and AirSim (bottom).

## <center>BibTeX</center>
```
@article{lin2021capturing,
  title={Capturing, Reconstructing, and Simulating: the UrbanScene3D Dataset},
  author={Lin, Liqiang and Liu, Yilin and Hu, Yue and Yan, Xingguang and Xie, Ke and Huang, Hui},
  journal={arXiv preprint arXiv:2107.04286},
  year={2022}
}
```
