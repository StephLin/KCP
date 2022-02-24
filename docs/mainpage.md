# KCP

[paper](https://doi.org/10.1109/LRA.2021.3140130) | [preprint](https://gpl.cs.nctu.edu.tw/Steve-Lin/KCP/preprint.pdf) | [code](https://github.com/StephLin/KCP) | [video](https://youtu.be/ZaDLEOz_yYc)

The official implementation of KCP: K-Closest Points and Maximum Clique Pruning
for Efficient and Effective 3D Laser Scan Matching, accepted for publication in
the IEEE Robotics and Automation Letters (RA-L).

![](images/snapshot.gif)

KCP is an efficient and effective local point cloud registration approach
targeting for real-world 3D LiDAR scan matching problem. A simple (and naive)
understanding is: <b>I</b>CP iteratively considers the closest point of each
source point, but <b>K</b>CP considers the <b>k</b> closest points of each
source point in the beginning, and outlier correspondences are mainly rejected
by the maximum clique pruning method. KCP is written in <b>C++</b> and we also
support <b>Python</b> binding of KCP (pykcp).

For more, please refer to our paper:

- Yu-Kai Lin, Wen-Chieh Lin, Chieh-Chih Wang, **K-Closest Points and Maximum Clique Pruning for Efficient and Effective 3-D Laser Scan Matching**. _IEEE Robotics and Automation Letters (RA-L)_, vol.7, no. 2, pp. 1471 -- 1477, Apr. 2022. ([paper](https://doi.org/10.1109/LRA.2021.3140130)) ([preprint](https://gpl.cs.nctu.edu.tw/Steve-Lin/KCP/preprint.pdf)) ([code](https://github.com/StephLin/KCP)) ([video](https://youtu.be/ZaDLEOz_yYc))

If you use this project in your research, please cite:

```bibtex
@article{lin2022kcp,
  title={K-Closest Points and Maximum Clique Pruning for Efficient and Effective 3-D Laser Scan Matching},
  author={Lin, Yu-Kai and Lin, Wen-Chieh and Wang, Chieh-Chih},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={2},
  pages={1471--1477},
  year={2022},
  doi={10.1109/LRA.2021.3140130},
}
```

and if you find this project helpful or interesting, please
[⭐Star the repository](https://github.com/StephLin/KCP). Thank you!
