# 物理系统

主要的参考来自 [raw-physics](https://github.com/felipeek/raw-physics) 和他的引用，但是按照我代码库的风格，结合我对于算法的理解作了一些修改：

Collision response was implemented based on *Detailed Rigid Body Simulation with Extended Position Based Dynamics* [1]. Collision detection was implemented with the help of *GJK* [2] and *EPA* [3]. The contact manifold generation was implemented using *Sutherland-Hodgman algorithm* [4]	in 3-dimensions, *Robust Contact Creation for Physics Simulations* [5] and the *Collision Manifolds Tutorial from Newcastle University* [6].

- [1] https://dl.acm.org/doi/10.1111/cgf.14105
- [2] https://ieeexplore.ieee.org/document/2083
- [3] https://graphics.stanford.edu/courses/cs468-01-fall/Papers/van-den-bergen.pdf
- [4] https://dl.acm.org/doi/10.1145/360767.360802
- [5] http://media.steampowered.com/apps/valve/2015/DirkGregorius_Contacts.pdf
- [6] https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/previousinformation/physics5collisionmanifolds/

## TODO List

1. 参照 raw-physics 的做法，实现一套稳定的碰撞检测系统，支持至少三种 collider（sphere，modified capsule，convex hull）
2. 参照 raw-physics 中写的 xpbd，实现一套稳定的刚体模拟，检测这个实现在有较大移动速度下的稳定性（此时应当能够看到效果比较好的刚体模拟结果，并且尝试在这个模拟的基础上编写平台跳跃玩法）
3. 参考 dynamics bone 插件，实现 spring bone 约束。应当注意，spring bone 应该加上对于角度的限制 clamping（此时应该能够简单用到尾巴，头发这类骨骼上检验效果）
4. 实现绳子的约束，保证绳子模拟准确稳定（此时可以加入一些玩法）
5. 学习 cloth physics 和 soft body simulation，思考如何把这种模拟稳定高质量地融合到 skinned mesh 的 character 上面（得到稳定并且能够最终用于展示的 demo）
