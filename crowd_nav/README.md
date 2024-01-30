# crowd_nav

Package that provides a ROS 1 wrapper to brnelib that implements Bayes Rule Nash Equilibrium algorithm for social navigation.

Dependencies to be installed:
* [Catch2](https://github.com/catchorg/Catch2)
* [Armadillo](https://arma.sourceforge.net/download.html) (Depends on OpenBLAS and LAPACK)
* [OpenMP](https://www.openmp.org/)

By default catkin is unable to find the crowd_nav_interfaces package that contains required messages. To overcome this:
1. Build crowd_nav_interfaces target separately using:
```
catkin_make --only-pkg-with-deps crowd_nav_interfaces
```
2. Then rebuild the workspace:
 ```
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```
