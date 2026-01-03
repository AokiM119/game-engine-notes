How to achieve the animation effect in real time?
在DCC里制作的动画都是一个个离散的sequence（clip），在实时的游戏内他们是如何组合在一起，形成连续且符合逻辑的动画呢？

## Animation Blending
The technique that allows more than one animation clip to contribute to the final final pose of the animation.

Case: Walking to Running
- Assume the character walks at 1.5m/s and runs at 3.0m/s in our game
- As the character speed increase, we want to switch the animation from walking to running
上面这个case，如果没有animation blend这个机制的话，动作状态的切换就会非常生硬。

Math of Blending: LERP
Use LERP to get intermediate frame from poses of different clips.
Weight is controlled by game parameters, such as character velocity.

1D Blend Space: Directional Movement
Players can move forward from multiple angles, we can blend any angle from the 3 clips below:
- Strafe left clip
- Strafe right clip
- Run forward clip
The technique is called 1D blend space.
![400][9.2.2DBlendSpace.png]
![400][9.1.1DBlendSpace.png]
2D space是以delaunay triangulation的方式插值。有了以上的animation blend机制，使得少许的clips就能让角色在游戏内丝滑运动。

Skeleton Masked Blending
Mask机制使得有些动画只apply到骨骼的某一部分（masked parts）
![400][9.3.MaskedBlending.png]

Additive Blending
只存clip的变化量，而不是绝对量。
Add a difference clip into a regular clip to produce a new clip.
Additive blending introduces a new kind of animation called a ***difference clip***, which represents the difference between two animation clip.
![400][9.4.AdditiveBlending.png]

## Action State Machine (ASM)
用更可视化的方式，提供工具给动画师。
游戏动作**系统**中，是由很多动作状态之间不停切换。于是很多时候，我们是以状态机的形式来描述一段“角色动画”（clip之间不是blending问题，而是彼此独立却又共同构成一个完整动作的关系）。
![400][9.5.ASM_Jump.png]

ASM consists of nodes and transitions
- Node type: 
	- blend space (e.g. 前后左右如何混合，打包成一个节点),
	- clip
- Transition type:
	- simply "pop" from one state to another
	- cross-fade from one state to the next
		- smooth transition
		- frozen transition
	- spatial transition states
动画状态机核心元素1：节点
动画状态机核心元素2：（节点之间的）Transition
动画机最终产出一定是个动画的一个pose.

只有这样的一个完整的动作系统，才能保证角色的动作切换是自然流畅的。
![[9.6.ASM_Definition.png]]

Animation State Machine in Unreal
- State: a blueprint graph which outputs a pose
- Transition: control when change state and how to blend

Layered ASM
分层动作状态机：角色上半身动作由技能控制，下半身主要有玩家input的移动来控制；另外一个大层来管控角色的受击。
Different parts of a character's body to be doing different, independent or semi-independent actions simutaneously.
![[9.10.LayeredASM.png]]

## Animation Blend Tree
![[9.11.BlendTree.png]]
Blend tree - structure layered ASMs and operations as a tree. ——将ASM和操作组织成动画混合树，可以将动画混合树视为layered ASM的一个超级版本。
- inspired by expression tree
- easy to understand for animator

For a blend tree
- Non-terminal nodes and terminal nodes (leaf node)
- The result of each non-terminal nodes is a pose

动画树最核心的：transition变量的定义
**在动画树的定义中，我们要定义大量的变量variable暴露给外面的gameplay系统**。Event graph和Anim graph……

Controlled by variable or events.
Unreal Animation BP control:
Named variables as members in animation blueprint
- Can be updated through blueprint
- Can be used anywhere inside the blend tree

## IK (Inverse Kinematics)
动作要跟环境互动，而环境会对动作有很多约束。

Basic concepts
- End-effector
	- 末端效果器
	- The bone which is expected to be moved to the desired position
- IK (Inverse Kinematics)
	- The use of kinematic equations to determine the joint parameters of a manipulator, so that the end effector moves to the desired position
	- 约束end-effector，然后让你反向去解其他parameters.
- FK (Forward Kinematics)
	- The use of kinematic equations to compute the position of the end-effectors from specified values for the joint parameters.

其实从逆向动力学，解出来在3D空间的解，是在一个圆环平面上。就走路动作来说，就会有“外八”或“内八”两种走法。
所以需要艺术家指定一个reference vector确定三维空间中IK的唯一解。
![400][9.13.TwoBonesIK_1.png]
![400][9.12.TwoBonesIK.png]

IK的难度在于，如果IK链的段数过多，反向动力学就会很难。
这里分享一些IK解法的tips：
Check reachability of the target
 ![400][9.14.IK_Tips_1.png]

人体每个joint都有活动范围的，不是在3D空间中自由旋转的，所以要给每个关节设置特有的constraints.
![400][9.15.JointConstraint.png]

IK算法 - 启发式算法
Why: too many joints and constraints, difficult to solve with analysis method.
Basic idea: designed to solve problem in faster and more efficient fashion by sacrificing optimality, accuracy, precision, or completeness for speed.
- Approximation
- Global optimality is not garanteed
- Iteration is usually used with a maximum limit

算法1：CCD and Optimized CCD
![[9.16.CCD_1.png]]
![[9.17.CCD_2.png]]

 算法2：FABRIK
 ![[9.18.FABRIK_1.png]]

Multiple End-Effector
更难的地方在于，角色身上的end-effector不止一个。在他们整体驱动下，如何去解IK呢？
![[9.19.MultipleEndEffector.png]]

行业中最经典的解法：雅各比矩阵的解法
In vector calculus, the Jacobian Matrix of a vector-valued function of several variables is the matrix of all its first-order partial derivatives.
![[9.20.JacobianMatrix.png]]

![[9.21.AnimationPipelineWithIK.png]]

有了IK之后的动画管线：
1、Clip Blend pose反向算到模型坐标系、世界坐标系
2、在世界坐标系中，拿各种约束反向计算每根骨骼该如何调整
3、调整完毕后，提交动画给渲染器，进行蒙皮等一系列顶点处理