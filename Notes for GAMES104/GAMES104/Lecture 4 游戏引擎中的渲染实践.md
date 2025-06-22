Rendering on Graphics Theory
- Focus on representation and math correctness
- No strict performance requirement
	- Realtime or interactive or offline rendering
	- Out of core rendering

Challenges of Game Rendering
- 渲染对象种类非常多, 非常复杂：All in one scenario.
- 需要适配当代硬件：Deal with architecture of modern computer with complex combination of CPU and GPU.
- 帧率需要稳定：Commit a bullet-proof framerate.
- CPU的预算有限：Limit access to CPU bandwidth and memory footprint. Game logic, network, animaiton, physic and AI system are major consumer of CPU and memory.

Rendering on Game Engine
- Heavily optimized
- Practical software framework
- Requirements of modern hardware

Outline of Game Rendering
1. **Basics of Game Rendering**
	- **Hardware architecture**
	- **Render data organization**
	- **Render Pipeline
		- **Culling & Visibility**
2. Materials, Shaders and Lighting
	- PBR(SG, MR)
	- Shader permutation
	- Lighting
		- Point / Direction lighting
		- IBL / Simple GI
3. Special Rendering
	- Terrian
	- Sky / Fog
	- Post-process
4. Pipeline
	- Forward, deferred rendering, forward-plus
	- Real pipeline with mixed effects
	- Ring buffer and V-sync
	- Tiled-based rendering

Building blocks of rendering: Rendering data and pipeline
- Vertex data
- Triangle data
- (Million of vertices and triangles...)
- Material parameters
- Textures
- (Tens of millions of pixels with hundreds ALU and dozen of textures sampling)
![[4.1.RenderPipeline.png]]

Computation 1 - Projection and Rasterization
- Projection transformation
- Rasterization - 这一步困惑了，难道是逐屏幕的pixel、以及逐vertex edge去进行光栅化的判断？

Computation 2 - Shading
A shader sample code:
- Constant, parameters
- ALU algorithms
- Texture sampling
- Branches
![[4.2.ShaderStructure.png]]

Computation 3 - Texturing
主要是滤波+插值这一块，需要额外的运算。Mipmap的产生也会占据一定内存。
![[4.3.Texturing.png]]

GPU: the dedicated hardware to solve massive job.

SIMD (Simple Instruction Multiple Data)
![400](4.4.SIMD.png)

SIMT (Simple Instruction Multiple Thread)
![400](4.5.SIMT.png)

数据在计算机中流动，是有成本的。查询数据，以及数据换存储位置的操作，都非常慢。
Always minimize data transfer between CPU and GPU when possible.
CPU送数据至显卡，最好就完事了，尽量避免CPU从显卡里读数据。

# 1  GPU Architecture
![[4.6.GPUArch.png]]
现代GPU架构
- GPC - Cluster，集群
- SM - Multiprocesser，多处理器
- CUDA Core - Core，计算核
以上三者是从上至下包含的关系。
- Texture Units 
- Warp

## Data Flow from CPU to GPU
![[4.7.DataFlow.png]]
数据在CPU和GUP之间的流动是有成本的。
冯诺依曼架构：计算和运算分开。
尽量避免CPU和GPU之间的数据交换。维持数据单向（CPU-GPU）流通是最好的。

## Be aware of cache efficiency
Cache hit
Cache miss
数据若在缓存里，计算过程会更快。

## GPU Bounds and Performance
Application performance is limited by:
- Memory bounds
- ALU bounds
- TMU(Texture Mapping Unit) bounds
- BW(Bandwidth) bounds

渲染引擎的架构是跟硬件架构息息相关的。
手游目前常见的是Tile-Based GPUs，也就是分块渲染管线。

# 2  Render data organization

## Mesh Render Component
Game object -> Component -> Mesh render component.
引擎中组成renderable的building block.

## Mesh
形状数据
![500](4.8.MeshPrimitive.png)
Mesh上需要保留每个vertex的数据，以及每个三角形的数据。
但是常见的Mesh数据结构并不像上图所示，而是以index buffer以及vertex buffer的方式储存。

IBO & VBO
Index Buffer Object >>> Vertex Buffer Object 
一般在使用图形API搭建渲染管线的时候会常使用这种方法。
![500](4.9.Vertex&IndexBuffer.png)

关于法线，常见的是每个vertex都有自己的normal，而不是使用插值。
这样的数据组织有利于区别软硬表面。
![500](4.12.NormalPerVertex.png)
## Material
材质数据
![500](4.10.Materials.png)

渲染系统中只定义可见材质，physical material是在物理系统起作用的。

很多时候是由纹理决定一个材质的种类，而不是材质参数。
![500](4.11.Textures.png)

# 3 Render Pipeline
有了对应的渲染数据，接下来就该跑渲染流水线。
渲染流水线的结构与硬件架构息息相关，也加入了不少软件优化算法步骤。

## Upload Data to Graphics Card
![500](4.13.RenderData.png)
抽象的游戏对象，变成了一个renderable。

How to Display Different Textures on a Single Model?
你可以把之前的数据结构，想象成Array结构的。有Vertex Array, Material (Index) Array, Texture Array等等……
每个Array都对应着一块Submesh的数据。
GO的渲染也按照这些Submesh进行切分。

![[4.14.Submesh.png]]

Instance
如果场景里每个GO的渲染数据都不一样，不可复用，那么整个数据量会非常大。

## Resource Pool & Instance

用一个大的Pool去管理
![500](4.15.ResourcePool.png)

Instance
Use handler to reuse resource.
![500](4.16.Instance.png)

Instance比object更加上一层，Instance可以在object的基础上，进行一些参数的调整。

每一次改变GPU绘制（材质）参数的时候，成本非常高。
所以场景里的绘制顺序，是按照material sort的结果，顺序对submeshes进行绘制。

```C
Initialize Resource Pools
Load Resources

Sort all Submeshes by Materials

for each Materials
	Update Parameters
	Update Textures
	Update Shaders
	Update VertexBuffers
	Update IndexBuffers
	for each Submeshes
		Draw Primitives
	end
end

```

**GPU Batch Rendering**
What if group rendering all instances with identical submeshes and materials together?
场景中很多物体，网格和材质信息都一模一样，这样子就直接采用合批渲染的方式。

合批的过程，是需要CPU端去实现的。
```C
struct BatchData
{
	SubmeshHandle m_submesh_handle;
	MaterialHandle m_material_handle;
	std::vector<PerInstanceData> m_per_instance_data; // 这里应该是除了网格和材质信息之外，每个instance各异的数据，例如坐标等等
	unsigned int m_instance count;
}

Initialize Resource Pools;
Load Resources;

Collect batchData with same submesh and material

for each BatchData
	Update Parameters
	Update Textures
	Update Shaders
	Update VertexBuffers
	Update IndexBuffers
		Draw Primitives
end

```

# 4 Visibility Culling
可视性裁剪，其实也是渲染系统为了节能，而具备的一个非常底层的功能。

## CPU Culling
Culling one object: use bounding box
![[4.17.Culling.png]]

Bounding box - 包围盒
不仅是在绘制环节，包括AI、逻辑、物理等环节都需要用到包围盒。
包围盒的表示方法有很多：
![[4.18.BoundingBox.png]]

Hierarchical View Frustrum Culling
层级结构的裁剪方式
树形的数据结构能够加速场景进行Culling的时间，因为不需要对于每个GO的包围盒进行遍历。

BVH
适用于动态场景。当物体在场景中变动时，BVH树的结构使得树能够很快的进行节点重排。
![[4.19.BVH.png]]

除了以几何包围盒划分层级结构之外，还有另一个做场景剔除很著名的思想：
PVS（Potential Visibility Set）
这个思想是以关卡结构作为划分，形成层级结构的。
- 以每个Portal作为空间的划分
	- 那么如何选择Portal的顺序，当作Leaf Node呢？
![[4.20.PVS.png]]

## GPU Culling
GPU Culling（上面讲的是CPU Culling？）
Early-Z
![400](4.21.PreZ.png)
第一遍，绘制深度图；然后在逐mesh/submesh进行光栅化绘制的时候，如果发现该mesh/submesh在此像素位置的深度大于深度图，那么就不再继续进行绘制——因为是会被挡住的。

Texture
纹理压缩算法。
如果采用PNG/JPEG压缩格式的话，是不能随机访问像素的。所以在游戏引擎里使用的是基于block compression的压缩算法。
![[4.22.TextureCompression.png]]

Cluster-Based Mesh Pipeline
基于新的硬件支持的，新型模型表达管线。

**GPU-Driven Rendering Pipeline**
- Mesh Cluster Rendering
	- Arbitrary number of meshes in single drawcall.
	- GPU-Culled by cluster bounds.
	- Cluster depth sorting.
**Geometry Rendering Pipeline Architecture**
- Rendering primitives are divided as:
	- Batch: a single API draw composed as many Surfs.
	- Surf: submeshes based on materials, composed of many Clusters.
	- Cluster: 64 triangles strip.

核心思想：面对非常精细的模型时，将其分成几何簇（32或64个面片组成的）。
现代的GPU已经能够非常高效的创建很多几何细节，并不是像传统管线那样，需要把VB IB先创建好传入GPU。——很多几何已经可以凭空算出来了。
![400](4.23.Cluster.png)
当把每个簇，大小固定住之后。里面的小面片的“很多计算是一样的”。

同步的硬件升级：多了一些能够进一步做几何计算的shader（类似于compute shader / geometry shader），通过一个面片可以再generate细化的几何面片。
![[4.24.ProgrammableMeshPipeline.png]]

基于几何簇的Culling
![[4.25.ClusterCulling.png]]

Nanite
	- Hierarchical LOD clusters with seamless boundary.
	- Don't need hardware support, but use hierarchical cluster culling on the pre-computed BVH tree **by persistent thread (CS) on GPU** instead of task shader.

![[4.26.TakeAway.png]]
总结：
1. 游戏引擎的设计，与硬件架构紧密相关
2. Submesh的设计是为了支持一个模型能够拥有不同的材质 
3. 剔除算法能够减少不必要的绘制
4. GPU的算力越来越强，所以更多的工作被移至GPU端，也叫做GPU-Driven