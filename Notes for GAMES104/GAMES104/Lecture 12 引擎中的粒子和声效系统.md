游戏引擎中的特效系统：粒子、声效

粒子系统 - Particle system
粒子系统最基础的构建元素：粒子 - Particle

# VFX System
## Particle Intro
A particle in game is usually a sprite or 3D model, with the following attribute:
每个粒子的属性包括：
- Position - 位置
- Velocity - 速度
- Size - 尺寸
- Color - 颜色
- Lifetime - 生命周期

粒子的生命周期：
![400][12.1.ParticleLife.png]
- Spawn
- Reaction to environment
- Death
粒子的生命周期管理要做好，否则会导致游戏资源的大量占用

Particle Emitter - 粒子发射器
Particle emitter is used to defined the particle stimulation.
- Specify the spawn rules —— 粒子被发射出来时候的规则，更多的是与event或behavior相关
- Specify the stimulation logic —— 粒子模拟规则，即在粒子在环境中的交互逻辑
- Describe how to render the particle —— 粒子的渲染规则
![300][12.2.ParticleEmitter.png]
当把很多个emitter放在一起的时候，就叫做一个particle system

Particle system
A collection of individual emitter combined.
![400][12.3.ParticleSystem.png]

参数1：Particle Spawn Position
粒子生成的位置，可以有以下这几种方式
![400][12.4.ParticleSpawn.png]

参数2：Particle Spawn Mode
粒子生成模型
- Continuous
- Burst
![[12.5.ParticleSpawnMode.png]]

当粒子生成后，在空间中的所有行为统称为“Simulation”
- Common force
	- Gravity - 重力
	- Viscous Drag - 阻力
	- Wind field - 风场
Simulation controls how the particle changes over time
![[12.6.ParticleVelocity.png]]
粒子与整个世界的collision计算，也是粒子系统非常重要的属性要求。

Particle type
- Billboard particle
	- Each particle is a sprite
	- Appears to be 3D 
	- Always facing the camera
- Mesh particle
	- Each particle is a 3D model
		- Rocks coming out from explosion
- Ribbon particle
	- A strip is created by connecting the particles and rendering the quads between the adjacent particles 
	- Smoothing with Centripedal Catmull-Rom interpolation
		- Add extra segments between particles
		- Can set the numbers of segments
		- Requires more CPU
![[12.7.ParticleType.png]]
![[12.8.RibbonParticle.png]]

## Particle System Rendering
挑战1：Alpha Blending
绘制透明物需要进行排序，即从最远处的透明物开始绘制。这样alpha blending的效果才会是正确的。排序对粒子系统来说就是个很大的问题。

Particle sort
Sorting mode
- Global sort
	- Accurate, but large performance consumption
- Hierarchy sort
	- Per system -> Per emitter -> Per particle (Within emitter)
Sort rules
- Between particles: based on particle distance with the camera
- Between systems or emitter: bounding box

Low-resolution Particle
半分辨率粒子方案
由于粒子系统普遍使用半透明渲染方案，所以性能很废。随之提出了一个low-resolution的方案：
![[12.9.LowResParticle.png]]

以降采样的方式提取场景深度值，再与particle system去做sorting and rendering，然后再把粒子rendering结果upsampling一遍，最终完成绘制。

Simulation - Process particles on GPU

Why GPU? 
- Highly parallel workload, suitable for large scale of particles
- Free your CPU to do game code
- Easy to access depth buffer to do collision

GPU
Particle pool - 数据结构
Dead list - 死的粒子列表
Alive list - 活的粒子列表
Deadd list and alive list - particle index is swapping between these two list，完成了系统的动态管理。
![[12.10.SimulateDataStructure_1.png]]
![[12.11.SimulateDataStructure_2.png]]

# Sound System

Audio
- Entertains the player, enhance the realism, establish the atmosphere

Sound fundamental 
- Volume - 音量
- Frequency - 音调
- Timbre - 音色

Digital Sound
怎么在计算机上表达声音？声音原本是连续信号，但计算机要求以离散的数据形式存储、播放。

PCM - Pulse-code Modulation 脉冲代码调制器
![[12.12.PCM.png]]

Sampling - 采样：香农定律，采样频率大于原频率的两倍，那么采样的信号就可以视为无损。
Quantizing - 量化：好像是进行amplitude度量的一个步骤。
Encoding - 编码

![[12.13.AudioFormat.png]]

3D Audio Rendering
目标：在一个三维空间，构建一个声音的场，让你感觉在这个世界发生各种各样的事情。在三维空间如何去渲染声音，很难。
声音也用一套渲染系统

声源：一段声音的source material
怎么知道我跟声源之间的关系呢？——需要指定一个listener，对整个空间进行“倾听”，就像是graphics render system里的camera一样。
对于FPS来说，Listener一般挂在主角相机上
![[12.14.AudioListener.png]]

Input: listener's position, velocity and orientation

Process: spatialization
- 两只耳朵听到声音的**音量**
- 两只耳朵听到声音的**时间**
- 两只耳朵听到声音的**音色**
要做出空间立体声的话，以上三个音效属性都是有差别的
![[12.15.Spatialization.png]]

Panning算法
当我的speaker有很多个channel通道，我通过调整不同通道上声音的大小、音色、latency，产生虚拟空间的感觉。
Distribution of an audio signal into a new stereo or multi-channel sound field.

Raw audio source -> **3D spatial (attenuation, obstruction, occlusion) algorithm** -> Output audio in the gaming realtime
游戏引擎或者音频插件（例如Wwise）主要处理的就是中间 **3D spatial algorithm**这一部分。——我猜的。

Attenuation - 声音衰减 - 另一个很复杂话题
FPS游戏，需要通过枪声判断开枪的人的方位。
![[12.16.Attenuation.png]]
离得近：高频的混响，枪声清脆
离得远：低频的震动，枪声越来越闷

Attenuation shape - sphere, capsule, box, cone, ...

Obstruction and Occlusion - 音频遮挡？
![[12.17.Obstruction&Occlusion.png]]

Reverb - 混响？
例如教堂里和NPC对话、骑马穿过桥洞……这些场景都会发生混响。混响是跟空间感、材质感关联性特别强的一种现象。
- 干音
- 回声
- 尾音
![[12.18.Reverb.png]]


Reverb in Action - Reverb Effect Control from Acoustic Parameters
在各种audio引擎里，用很简单的参数就定义混响这件事：
- HF ratio：到底是你对高频部分的混音更强一些，还是低频部分
![[12.19.ReverbParameters.png]]

Sound in motion: the Doppler effect - 多普勒效应

音频中间件：
提供给audio designer一个非常符合他们工作习惯的工作环境，让他们只专注于如何在3D世界里对声音进行组合渲染。