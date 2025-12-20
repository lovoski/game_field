# 开发计划

1. 我需要在引擎里面实现 dual quaternion skinning，并且让这套系统和原本的 linear blend skinning 共存，能够对于每一个 mesh 方便地转换
2. 需要有一种方法能够快速将 fbx，bvh 格式的 motion，pose 施加到特定角色的身上，考虑通过一个指定的关节之间的 mapping ，通过一个简单的 optimizer 来近似两个 pose
3. 我需要更多的 debug rendering 功能，比如说显示出一个骨骼影响的所有 skinning 顶点 weights，能够快速显示当前 mesh 的 uv，绘制出模型的法向量
4. 很可惜部分代码是没办法跨平台的，比如说选择合适的 gpu，这部分代码目前只能够在 windows 执行，既然已经有了不跨平台的代码，可以试着探索还有哪些 windows 独有的功能加入到引擎中
5. 对于希望实现的视觉效果，可以先试着在 blender 里面制作出来，再考虑是否容易移植到引擎中，是否有必要移植到引擎中，首先通过低成本的尝试，再考虑形式上的问题