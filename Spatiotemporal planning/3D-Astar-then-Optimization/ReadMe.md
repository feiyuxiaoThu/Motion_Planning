# First Search then Optimize method

We adopt the planning framework proposed by Tesla in [Tesla AI Day](https://www.bilibili.com/video/BV1PM4y1V7g5/). Namely the framework:

**First Search + then Optimize in Corridor**

The core subparts of this code work consist of:

+ 3D A* search: coupling x-y-t search to fully consider *dynamic obstacles*
+ QP optimization for smooth and comfortable trajectory generation based on 3D A* search



Main ideas are based on the following papers and **codes are heavily borrowed from the corresponding codes**  

1. Bai Li, Youmin Zhang, Yakun Ouyang, Yi Liu, Xiang Zhong, Hangjie Cen, and Qi Kong, "Fast trajectory planning for AGV in the presence of moving obstacles: A combination of 3-dim A* search and QCQP", ***33rd Chinese Control and Decision Conference (CCDC)***, pp. 7549-7554, 2021.
2. Bai Li, Qi Kong, Youmin Zhang, et al., "On-road trajectory planning with spatio-temporal RRT* and always-feasible quadratic program", ***2020 16th IEEE International Conference on Automation Science and Engineering (CASE)\***, 943-948, 2020.