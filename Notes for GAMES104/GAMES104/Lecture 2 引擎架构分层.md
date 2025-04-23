
Layered Architecture of Game Engine

## 1 游戏引擎分层介绍 | Game Engine 5 Basic Layers Intro

游戏引擎的五个最基础的layer分别是：
- **Tool layer** - 工具层 例如呈现在引擎里的各个编辑器
- **Function layer** - 功能层，例如Rendering, UI, Script/FSM/AI, Physic, Animation各种系统
- **Resource layer** - 资源层，主要存储游戏内容数据
- **Core layer** - 核心层，功能层频繁调用的底层代码，如线程管理、内存分配管理
- **Platform layer** - 平台层，对发布平台、硬件设备进行适配
还有中间层/第三方库，所以一共是5+1的结构。

以动画系统为例，来解释每个层的要点，以及每一层之间如何联动。
## 2 资源层 | Resource Layer

### Resource - How to access my data
![[1.1.OfflineResourceImporting.png]]
Offline resource importing
资源导入的几个关键步骤：
#### Unify file asset by defining a meta asset file format (i.e. ast)
从resource到游戏资产（asset），有很大的不同。游戏资产会将资源数据进行数据的简化+格式的统一。例如贴图资源，游戏引擎会将PSD/PNG等格式的文件转换成DPS格式，DPS可以直接被显卡读取，所以在游戏实时运行的时候能够显示出来。
#### Build a composite asset file to refer to all resources
不同格式之间的资产通常需要关联起来。所以需要composite asset这个类别的资产来做这件事情，composite asset可以看作一个资源清单，其内容引用（Game Object）所有有关联的资产。
#### GUID is an extra protection of reference
GUID - 唯一识别号
每一个资产拥有自己的GUID，根据GUID就能精准的找到对应的资产，即使当资产的路径发生改变时候。
### Resource - Runtime Asset Manager
实时的资源管理器，根据资产的路径进行实时的管理。
A virtual file system to load/unload assets by path reference.
![400](2.2.CrossReference.png)
Manage asset life span and reference by **handle system**.
### Resource - Manage Asset Life Cycle
资源层的核心是对所有资产生命周期的管理。
![[2.3.ResourceManager.png]]
Memory management for Resources - Life cycle.
内存小，硬盘大，所以大部分游戏资产希望在玩家玩的时候去实时加载。
Different resources have different life cycles.
Limited memory requires release of loaded resources when possible.
Garbage collection and deferred loading is a critical feature.
## 3 功能层 | Function Layer

Function - How to make the world alive
每隔1/30s，把整个游戏世界的逻辑和绘制全部跑了一遍。

![300](2.4.Tick.png)

### Dive into Tick

Tick的两大“神兽” - tickLogic和tickRender。每个tick（即每一帧）的逻辑和渲染是分两步运行的。
```c++
void tickMain()
{
	while(!exit_Flag)
	{
		tickLogic(delta_time);
		tickRender(delta_time);
	}
}
```
#### tickLogic
将物理规则全部计算一遍，自动符合物理学规律的。
```c++
void tickLogic(delta_time)
{
	tickCamera(delta_time);
	tickMotor(delta_time);
	tickController(delta_time);
	tickAnimation(delta_time);
	tickPhysic(delta_time);
	/*...*/
}
```

#### tickRender
以主观视角去观察“当下的世界”，观察即渲染过程。
```c++
void tickRender()
{
	tickRenderCamera(delta_time);
	culling();
	rendering();
	postprocessing();
	present();
}
```

#### Tick for Animation (Oversimplified)
In each tick:
- Fetch animation frame of character
- Drive the skeleton and the skin of character
- Renderer process all rendering job in an iteration of render tick for each frame.

### Heavy-Duty Hotchpotch
Gameplay功能、引擎功能的区分，一般会在功能层这块打架。

总结：
Function layer provides major function modules for the game engine.
- Object system (HUGE)
Game loop updates the system periodically.
- Game loop is the key of reading codes of game engine.
Blur the boundary between engine and the game.
- Camera, character and behavior
- Design extendable engine API for programmer

### Multi-threading
现在Unity和UE这种商业引擎，会将物理运算等可以并行的计算，在多核（多线程）上进行fork&join.
Many systems in game engine are built for parallelism.
未来的趋势：将每个任务进行原子化，可以在各个线程上满满的排列。难的是在运行的dependency上。

## 4 核心层、平台层 | Core & Platform Layer
Core核心层会包括一些上层（如功能层）高频率调用的、偏底层的功能库，例如数学库、STD库等。
在游戏引擎中，这些库很有可能为了提高实时计算的效率而进行重写。
- Math Library
- Data Structure and Containers
- Memory Management (CPU的缓存空间往往是卡点)
	- Major bottlenecks of game engine performance
	- Reduce cache miss
	- Memory alignment
Core layer is the foundation of Game Engine. 
![[2.5.CoreLayer.png]]

平台层
Compatibility of different platform, provides platform-independent services or information for upper layers.
不同的硬件平台使用的文件读取方式、图形SDK可能都不一样。
例如Render Hardware Interface - RHI，重新定义的一层图形API：
- Transparent different GPU architecture and SDK
- Automatic optimization of target platform
## 5 工具层 | Tool Layer

Allow anyone to create game:
- Level editor
- Logical editor
- Shader editor
- Animation editor
- UI editor
工具层的开发框架和引擎底层结构没太大关系，它的核心在于服务于用户对游戏内容进行引擎内编辑。

## 6 为什么要采用分层架构

Decoupling and reducing complexity.
Respond to evolving demands

面对每一个功能需求的时候，要判断这个需求属于哪一层。以及在各个层需要做哪些改动。
