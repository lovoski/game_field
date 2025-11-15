# 物理系统

主要的参考来自 [raw-physics](https://github.com/felipeek/raw-physics) 和他的引用，但是按照我代码库的风格，结合我对于算法的理解作了一些修改：

Collision response was implemented based on *Detailed Rigid Body Simulation with Extended Position Based Dynamics* [1]. Collision detection was implemented with the help of *GJK* [2] and *EPA* [3]. The contact manifold generation was implemented using *Sutherland-Hodgman algorithm* [4]	in 3-dimensions, *Robust Contact Creation for Physics Simulations* [5] and the *Collision Manifolds Tutorial from Newcastle University* [6].

- [1] https://dl.acm.org/doi/10.1111/cgf.14105
- [2] https://ieeexplore.ieee.org/document/2083
- [3] https://graphics.stanford.edu/courses/cs468-01-fall/Papers/van-den-bergen.pdf
- [4] https://dl.acm.org/doi/10.1145/360767.360802
- [5] http://media.steampowered.com/apps/valve/2015/DirkGregorius_Contacts.pdf
- [6] https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/previousinformation/physics5collisionmanifolds/
