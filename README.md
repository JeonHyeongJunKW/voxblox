# Voxblox

[![Build Test](https://github.com/ethz-asl/voxblox/actions/workflows/build_test.yml/badge.svg)](https://github.com/ethz-asl/voxblox/actions/workflows/build_test.yml)

![voxblox_small](https://cloud.githubusercontent.com/assets/5616392/15180357/536a8776-1781-11e6-8c1d-f2dfa34b1408.gif)

Voxblox is a volumetric mapping library based mainly on Truncated Signed Distance Fields (TSDFs). It varies from other SDF libraries in the following ways:
 * CPU-only, can be run single-threaded or multi-threaded for some integrators
 * Support for multiple different layer types (containing different types of voxels)
 * Serialization using protobufs
 * Different ways of handling weighting during merging
 * Different ways of inserting pose information about scans
 * Tight ROS integration (in voxblox_ros package)
 * Easily extensible with whatever integrators you want
 * Features an implementation of building Euclidean Signed Distance Fields (ESDFs, EDTs) directly from TSDFs.

**If you're looking for skeletonization/sparse topology or planning applications, please refer to the [mav_voxblox_planning](https://github.com/ethz-asl/mav_voxblox_planning) repo.**
**If you want to create ground truth maps from meshes or gazebo environments, please check out the [voxblox_ground_truth](https://github.com/ethz-asl/voxblox_ground_truth) pakage!**

![example_gif](http://i.imgur.com/2wLztFm.gif)

# Documentation
* All voxblox documentation can be found on [our readthedocs page](https://voxblox.readthedocs.io/en/latest/index.html)

## Table of Contents
* [Paper and Video](#paper-and-video)
* [Credits](#credits)
* [Example Outputs](https://voxblox.readthedocs.io/en/latest/pages/Example-Outputs.html)
* [Performance](https://voxblox.readthedocs.io/en/latest/pages/Performance.html)
* [Installation](https://voxblox.readthedocs.io/en/latest/pages/Installation.html)
* [Running Voxblox](https://voxblox.readthedocs.io/en/latest/pages/Running-Voxblox.html)
* [Using Voxblox for Planning](https://voxblox.readthedocs.io/en/latest/pages/Using-Voxblox-for-Planning.html)
* [Transformations in Voxblox](https://voxblox.readthedocs.io/en/latest/pages/Transformations.html)
* [Contributing to Voxblox](https://voxblox.readthedocs.io/en/latest/pages/Modifying-and-Contributing.html)
* [Library API](https://voxblox.readthedocs.io/en/latest/api/library_root.html)

# Paper and Video
A video showing sample output from voxblox can be seen [here](https://www.youtube.com/watch?v=PlqT5zNsvwM).
A video of voxblox being used for online planning on-board a multicopter can be seen [here](https://youtu.be/lrGSwAPzMOQ).

If using voxblox for scientific publications, please cite the following paper, available [here](http://helenol.github.io/publications/iros_2017_voxblox.pdf):

Helen Oleynikova, Zachary Taylor, Marius Fehr, Juan Nieto, and Roland Siegwart, “**Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning**”, in *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2017.

```latex
@inproceedings{oleynikova2017voxblox,
  author={Oleynikova, Helen and Taylor, Zachary and Fehr, Marius and Siegwart, Roland and  Nieto, Juan},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  title={Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning},
  year={2017}
}
```

# 추가적인 조사할 내용
- 내부 코드에 대한 주석 및 분석을 합니다. 
- 분석을 통해서 semantic한 정보를 가진 mesh를 생성하는 방법을 찾습니다.
- Kimera의 내부에서 point에 라벨을 부여할 수 있었던 방법을 찾습니다. 
  - 스테레오카메라로부터 생성된 3차원점에 라벨을 부여한다.
  - bundled raycasting간에서 각 ray bundle에 대해서, 번들에서 관측된 라벨에 대한 빈도로부터 확률 벡터를 얻는다. 
  - 이것을 truncated 범위(평면근처) 내에서 전파한다. 
  - 그 ray를 따라서 복셀을 경유하는 동안에, 우리는 사후 라벨확률을 각 복셀에 대해서 계산한다.
  - metric-semantic mesh는 marching cube를 사용하여 대응된다.

# 조사를 통해서 배운 내용
- tsdf_sever_node.cc가 기본적인 노드이다.
- tsdf_server.cc를 통해서 tsdf의 subscribe와 publish의 등록과 처리를 클래스형태로 구현한다.
  - 그중에서도 유심히 봐야하는 함수는 pointcloud를 삽입하는 아래의 함수이다. 
  ```
  processPointCloudMessageAndInsert
  ```
  - pcl에서 나온정보로부터 메쉬의 semantic을 정하기 위해서는 pcl::PointCloud에 semantic정보를 넣어야한다.
- tsdf로부터 mesh를 만드는 부분도 분석해보자. 분석을 통해서 mesh각각에 객체에 따른 색상을 할당해보자.


# Credits
This library was written primarily by Helen Oleynikova and Marius Fehr, with significant contributions from Zachary Taylor, Alexander Millane, and others. The marching cubes meshing and ROS mesh generation were taken or heavily derived from [open_chisel](https://github.com/personalrobotics/OpenChisel). We've retained the copyright headers for the relevant files.

![offline_manifold](https://i.imgur.com/pvHhVsL.png)
