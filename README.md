Evaluation program for reconstructed model.
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

### Reference
[UrbanScene3D](https://linxius.github.io/UrbanScene3D/)