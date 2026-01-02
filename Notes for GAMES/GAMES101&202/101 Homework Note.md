# Homework 1 
## 1 Intro
作业1~3模拟一个基于 CPU 的光栅化渲染器的简化版本。

作业1的任务是填写一个旋转矩阵和一个透视投影矩阵。给定三维下三个点 v0(2.0, 0.0, −2.0), v1(0.0, 2.0, −2.0), v2(−2.0, 0.0, −2.0)，HW1将这**三个点的坐标变换为屏幕坐标，并在屏幕上绘制出对应的线框三角形**（注意只是线框）。

代码框架中已经提供了 draw_triangle 函数，只需要补充构建变换矩阵即可（模型变换和投影变换的部分）。

程序运行结果：
![400](hw1.png)
## 2 Structure
外部库：
- Eigen
- Opencv
- Cmath

内部结构：
#### 2.1 Rasterizer
*所有的buffer数据结构都定义在Rasterizer Class里面*
- Struct
	- Primitive：图元结构
		- pos_buf_id
		- ind_buf_id
	- Buffers：缓冲数据空间
```c++
enum class Buffers
{
    Color = 1,
    Depth = 2
};

struct pos_buf_id
{
    int pos_id = 0;
};

struct ind_buf_id
{
    int ind_id = 0;
};

enum class Primitive
{
    Line,
    Triangle
};
```
- Class
	- Public //
		- Properties: 
			- width, height
		- Method:
			- set (Camera) MVP:
				- set_model
				- set_view
				- set_projection
			- load geometry:
				- set_pixel
				- load_positions
				- load_indices
			- clear
			- draw
	- Private
		- Properties:
			- **MVP matrices**
				- Eigen::Matrix4f model
				- Eigen::Matrix4f view
				- Eigen::Matrix4f projection
			- **Buffers**
				- pos_buf
				- ind_buf
				- frame_buf
				- depth_buf
		- Method:
			- draw_line
			- rasterize_wireframe
```c++
class rasterizer
{
  public:
    rasterizer(int w, int h);
    pos_buf_id load_positions(const std::vector<Eigen::Vector3f>& positions);
    ind_buf_id load_indices(const std::vector<Eigen::Vector3i>& indices);
    void set_model(const Eigen::Matrix4f& m);
    void set_view(const Eigen::Matrix4f& v);
    void set_projection(const Eigen::Matrix4f& p);
    void set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color);
    void clear(Buffers buff);
    void draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, Primitive type);
    std::vector<Eigen::Vector3f>& frame_buffer() { return frame_buf; }

  private:
    void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end);
    void rasterize_wireframe(const Triangle& t);

  private:
    Eigen::Matrix4f model;
    Eigen::Matrix4f view;
    Eigen::Matrix4f projection;
    std::map<int, std::vector<Eigen::Vector3f>> pos_buf;
    std::map<int, std::vector<Eigen::Vector3i>> ind_buf;
    std::vector<Eigen::Vector3f> frame_buf;
    std::vector<float> depth_buf;
    int get_index(int x, int y);
    int width, height;
    int next_id = 0;
    int get_next_id() { return next_id++; }
};
```

#### 2.2 Triangle
*这里的Triangle Class，可以泛化理解为几何类。当渲染系统的结构越来越复杂时，几何类的类别也会越来越多，其中具备的数据结构也会越来约复杂——通常是为了满足特定的几何算法。*
- Public properties
	- v[3]
	- color[3]
	- tex_coords[3]
	- normal[3]
- Public methods
	- setVertex
	- setNormal
	- setColor
	- setTexCoord
	- toVector4 ///转换成齐次坐标
## 3 Algorithm

### 3.1 Main
首先要加载几何数据。位置信息和顶点信息分两次加载。
```c++
pos_buf_id load_positions(const std::vector<Eigen::Vector3f>& positions);
ind_buf_id load_indices(const std::vector<Eigen::Vector3i>& indices);
```
加载方式是将position(`std::vector<Eigen::Vector3f>`)和index(`std::vector<Eigen::Vector3i>`)分别放入两张map，分别是pos_buf和ind_buf，他们的数据关系如下。
![[hw1.buffers.png]]
实际postion和index的数据如下，这些数据只包含了一个三角形。
```c++
std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};
std::vector<Eigen::Vector3i> ind{{0, 1, 2}};
```

### 3.2 Rasterizer::Draw
确定了几何数据buffer的结构之后，需要在渲染器的draw()方法里调用。
Draw方法中最主要的就是index buffer loop这个逻辑，选择遍历index buffer是因为它的容积数量就是所有三角图元的数量（total number of triangle primitives）。
- 对所有的index指向的position进行mvp变换
- 将mvp之后的顶点数据写到原来的三角形图元数据中
- 调用线框绘制的函数rasterize_wireframe(t)
```c++
void rst::rasterizer::draw(rst::pos_buf_id pos_buffer, rst::ind_buf_id ind_buffer, rst::Primitive type)
{
	//Draw方法中最主要的就是index buffer loop这个逻辑
    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        for (auto& vec : v) {
            vec /= vec.w();
        }
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }
        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }
        t.setColor(0, 255.0,  0.0,  0.0);
        t.setColor(1, 0.0  ,255.0,  0.0);
        t.setColor(2, 0.0  ,  0.0,255.0);
        rasterize_wireframe(t);
    }
}
```
线框绘制并不是正式的光栅化算法，只是在第一节里的一种根据顶点位置的简单绘制算法（经典的光栅化绘制会在homework2里面实现）：
- Bresenham's line drawing algorithm
- Code taken from a stack overflow answer: https://stackoverflow.com/a/16405254
# Homework 2
## 1 Intro
在HW1中，我们只在屏幕上画出了一个线框三角形，HW2我们继续推进一步——在屏幕上画出一个实心三角形，换言之，栅格化一个三角形。
- 栅格绘制
- 深度检测
这一次，你需要自己（在光栅化类定义中）填写并调用函数 rasterize_triangle(const Triangle& t)。
程序运行结果：
![400](hw2.png)
## 2 Structure
**主要改动是在rasterizer.cpp中的rasterize_triangle(const Triangle& t)中**，更新后该函数的流程如下：
- **创建三角形的 2 维 bounding box**（这一步看似不起眼，但实际对性能至关重要，否则你要遍历整个屏幕像素进行栅格化测试）。
- **遍历此 bounding box 内的所有像素（使用其整数索引）**。然后，使用像素中心的屏幕空间坐标来检查中心点是否在三角形内。
	- 如果在内部，则将其位置处的插值深度值 (interpolated depth value) 与深度缓冲区 (depth buffer) 中的相应值进行比较。
		- 如果当前点更靠近相机，请设置像素颜色并更新深度缓冲区 (depth buffer)。
## 3 Algorithm
### 3.1 rst::rasterizer::rasterize_triangle
rasterize_triangle的算法步骤大致如上述，MSAA的部分详细解释一下：
在以boundingbox得到的边界内，以整数index遍历屏幕像素时，每个像素要进行四次采样并把结果加权平均。
关于MSAA需要注意的：
- 像素最终色彩值：取四个采样点的平均值（在包围盒内的前提下）。
- 像素最终深度值：取四个采样点的中的最小值（在包围盒内的前提下）。

```c++
void rst::rasterizer::rasterize_triangle(const Triangle& t)
{
    auto v = t.toVector4();

    // Bounding Box
    int min_x = std::min(v[0].x(), std::min(v[1].x(), v[2].x())), 
	    min_y = std::min(v[0].y(), std::min(v[1].y(), v[2].y())), 
		max_x = std::max(v[0].x(), std::max(v[1].x(), v[2].x())), 
		max_y = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));

    // MSAA
    int pid = 0;// 当前像素的id
    float z_pixel = 0;// 当前像素的深度
    const float dx[4] = { 0.25, 0.25, 0.75, 0.75 }, 
			    dy[4] = { 0.25, 0.75, 0.25, 0.75 };// 四个样本中心偏移量
    for (int x = min_x; x <= max_x; x++)
        for (int y = min_y; y <= max_y; y++)
        {
            pid = get_index(x, y) * 4;
            z_pixel = 0x3f3f3f3f;
            // 对每个样本进行Z-Buffer算法
            for (int k = 0; k < 4; k++)
            {
                if (insideTriangle(x + dx[k], y + dy[k], t.v))
                {
                    // Z-Buffer算法
                    // 获取当前样本的深度
                    auto tmp = computeBarycentric2D(x + dx[k], y + dy[k], t.v);
                    float alpha, beta, gamma;
                    std::tie(alpha, beta, gamma) = tmp;
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    // 深度测试
                    if (z_interpolated < depth_sample[pid + k])// 当前像素在前面
                    {
                        depth_sample[pid + k] = z_interpolated;// 更新样本深度缓存
                        frame_sample[pid + k] = t.getColor() / 4;// 更新样本颜色缓存
                    }
                    z_pixel = std::min(z_pixel, depth_sample[pid + k]);// 更新像素深度为最前面样本深度
                }
            }
            Vector3f point = { (float)x, (float)y, z_pixel }, 
	            color = frame_sample[pid] + frame_sample[pid + 1] + frame_sample[pid + 2] + frame_sample[pid + 3];
            depth_buf[get_index(x, y)] = z_pixel;// 更新深度缓存
            set_pixel(point, color);
        }
}
```

# Homework 3
## 1 Intro
这次编程任务中，我们会进一步模拟现代图形技术。代码框架中添加了Object Loader(用于加载三维模型)，Vertex Shader 与 Fragment Shader，并且支持了纹理映射。

程序运行结果：
Using Blinn-Phong Shader
![400](hw3.png)
## 2 Structure
头文件：
- global.hpp
- OBJ_Loader.h
- rasterizer.hpp
- Shader.hpp
- Texture.hpp
- Triangle.hpp
源文件：
- main.cpp
- rasterizer.cpp
- Texture.cpp
- Triangle.cpp
### 2.1 Main
主程序包含的重要步骤有：
1. **Shader Initiative** - Shader数据初始化。会将shader必备数据结构（payload）传入，进行shader内容的定义。
	1. vertex_shader
	2. normal_fragment_shader
	3. texture_fragment_shader
	4. phong_fragment_shader
	5. displacement_fragment_shader
	6. bump_fragment_shader
2. **Load OBJ File** - 加载OBJ格式的几何数据。使用到OBJ_Loader.h的（外部）头文件，将OBJ数据解析，使得能够被自定义光栅渲染器使用。
3. **Loop "Set MVP Matrix and Rasterize"** - 在main进行循环，传入实时的MVP数据，调用光栅渲染器的Draw函数进行绘制。
*注：这里灯光相关的数据是直接在shader code里hard defined的（灯光类是在main.cpp中定义）。但实际渲染引擎中，灯光应是个独立的类，其数据对于渲染器来说，应该是全局可访问的。*

下面主要分析OBJ Loader，以及几个shader的算法。
## 3 Algorithm
### 3.1 OBJ Loader
OBJ Loader的作用主要是为了将OBJ几何数据解析成可供渲染器解读的数据结构。

OBJ Loader的调用出现在主函数中。

主函数中对几何图形数据的加载、解析的代码如下：
1、读取模型路径
```c++
objl::Loader Loader;
std::string obj_path = "C:/Games/Games101/Games101-3/models/spot/";
```

2、Load OBJ
```c++
bool loadout = Loader.LoadFile(obj_path + "spot_triangulated_good.obj");
```

3、将Load数据转换成自定义的Triangle数据结构
```c++
for(auto mesh:Loader.LoadedMeshes)
{
	for(int i = 0; i < mesh.Vertices(); i+=3)
	{
		Triangle* t = new Triangle();
		for(int j=0;j<3;j++) //Put the data of every triangle's vertex into buffers
		{
            //Buffers: Vertex, Normal, TexCoord
            //Set Vertex Buffer
            t->setVertex(j, Vector4f(
            mesh.Vertices[i+j].Position.X,
            mesh.Vertices[i+j].Position.Y,
            mesh.Vertices[i+j].Position.Z,
            1.0));
            //Set Normal Buffer
            t->setNormal(j, Vector3f(
            mesh.Vertices[i+j].Normal.X,
            mesh.Vertices[i+j].Normal.Y,
            mesh.Vertices[i+j].Normal.Z
            ));
            //Set Texcoord Buffer
            t->SetTexCoord(j, Vector2f(
            mesh.Vertices[i+j].TextureCoordinate.X,
            mesh.Vertices[i+j].TextureCoordinate.Y));
		}
		TriangleList.push_back(t); //Place at the end of TriangleList
	}
}
```
- 一个OBJ格式的模型可能包含多个网格（multiple meshes），所以第一层剥离是针对不同的meshes
	- 每个mesh，要把它整合成三角形图元列表（Triangle List）
		- 对于每个mesh，OBJ Loader会将它的每个三角形图元的所有顶点按照顺序排列到mesh.Vertices[]顶点列表中。
		- 我们要做的是将顶点列表整合成三角形列表，即：以三角形图元（Triangle）为单位，将它的三个顶点的Position Normal TexCoord数据，全部放到三角形数据结构t中。
OBJ的每个mesh最终被整合到一个Triangle List里面。

4、传递TriangleList给了渲染器
```c++
//main.cpp
r.draw(TriangleList);
```

5、在渲染器中，遍历TriangleList进行绘制
```c++
void rst::rasterizer::draw(std::vector<Triangle *> &TriangleList)
{
	Eigen::Matrix4f mvp = projection * view * model;
	for(const auto& t:TriangleList)
	{
		//MVP transform for each vertex
		//MVP transform for Normal 
		//rasterize triangle(also pass vertex's view space position to rasterizer)
		rasterize_triangle(newtri, viewspace_pos);
	}
}
```

### 3.2 Rasterizer
*注1：作业3的rasterizer光栅器要比之前作业2更复杂。原因是此处要对每个片元做额外数据的插值，例如color, normal, texcoord甚至view pos——都是为了传递到shader计算使用。*
*注2：法线向量的MVP与顶点位置的不同。*
![500](hw3.4.Normal_MVP.png)

*注3：对每个图元（triangle）进行光栅化时，处理的每个片元（fragment）时产生的插值数据，这个程序里叫做fragment_shader_payload。在真实的光栅化流水线中，是在GPU内产生、传输的。*
*注4：从模型（.obj）到图元（triangle）数据的处理，是在CPU上进行的。*

*注5：作业三与作业一不同。作业一使用了更为性能友好的postion buffer & index buffer作为几何数据结构；而作业三使用的TriangleList，每个triangle之间的顶点数据会有重合和冗余。*
### 3.3 Different Shaders
#### 3.3.1 Blinn-Phong Shader
最最最原始的一种模拟材质的着色模型。
```c++
Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
	//自定义Blinn Phong材质参数
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);
	
	//自定义灯光参数
    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};
    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    
    Eigen::Vector3f eye_pos{0, 0, 10};
    float p = 150;
    
    //读取片元payload中的各种数据
    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    //Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);


    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        //0 Unpack light data
        Eigen::Vector3f I = light.intensity;
        float r2 = (light.position - point).dot(light.position - point);

        //1 Ambient light
        Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);

        //2 Diffuse light
        //2.1 Get light direction
        Eigen::Vector3f L = (light.position - point).normalized(); 

        //2.2 NdotL
        Eigen::Vector3f Ld = kd.cwiseProduct(I / r2) * std::max(0.0f, normal.dot(L));

        //3 Specular light
        //3.1 Half direction vector
        Eigen::Vector3f V = (eye_pos - point).normalized();
        Eigen::Vector3f H = (L + V).normalized();

        //3.2 NdotH
        Eigen::Vector3f Ls = ks.cwiseProduct(I / r2) * std::pow(std::max(0.0f, normal.dot(H)), p); 

        result_color += La + Ld + Ls;
        
    }

    return result_color * 255.f;
}
```
#### 3.3.2 Bump & Displacement Mapping Shader

Bump Shading VS Displacement Mapping = “表面浮雕vs立体雕塑”

**Bump Shading** 和 **Displacement Mapping** 的原理、效果区别以及它们各自的优缺点。它们都是用于在低多边形模型上模拟高分辨率表面细节的技术，但实现方式和效果有本质不同。

**核心目标相同：** 欺骗人眼，让一个相对简单的几何模型看起来拥有复杂的表面细节（如凹凸、划痕、砖缝、皱纹等），而无需实际建模这些细节，从而节省内存和计算资源（尤其是顶点处理）。

##### 3.3.2.1 Bump Shader
- **核心原理：** **纯光学欺骗**，只修改**表面法线**的方向，**完全不改变模型的几何形状（顶点位置）**。
- **关键数据：**
    
    - **法线贴图 (Normal Map)：** 这是最常用和现代的凹凸着色实现方式。它是一张纹理图（RGB），每个纹素的颜色值 (R, G, B) 编码了该点在**切线空间 (Tangent Space)** 下的法线向量 (X, Y, Z)。想象这张图告诉你模型表面每个微小区域应该“朝向”哪个方向。
        
    - **高度贴图 (Height Map / Bump Map)：** 更早的技术，是一张灰度图（通常只用R通道）。较亮的区域表示“凸起”，较暗的区域表示“凹陷”。它本身不直接存储法线，需要在着色器中通过计算相邻纹素值的差异（差分）来**近似推导**出法线的扰动。
    
- **工作流程 (通常在片元/像素着色器中)：**
    
    1. 从法线贴图（或通过高度贴图计算）中读取或计算出**扰动后的法线向量**（在切线空间）。
        
    2. 将扰动法线从切线空间转换到进行光照计算所需的空间（通常是世界空间或视图空间）。这需要模型的**切线 (Tangent)** 和**副切线 (Bitangent)** 向量信息（通常作为顶点属性传递）。
        
    3. 使用这个**扰动后的法线**代替模型原始的（插值得到的）法线进行**光照计算**（漫反射、镜面反射等）。

![300](hw3.1.png)
本作业中，使用TBN矩阵，将切线空间中的法线，转化到了模型空间内。并且将其以RGB的形式渲染出来。
>**✅ 是的，将切线空间（Tangent Space）的法线乘以 TBN 矩阵，得到的是模型空间（Model Space）的法线值！**

TBN矩阵
```c++
    float x = normal.x();
    float y = normal.y();
    float z = normal.z();
    Eigen::Vector3f t {x* y / sqrt(x * x + z * z), sqrt(x* x + z * z), z* y / sqrt(x * x + z * z)};
    Eigen::Vector3f b = normal.cross(t);
    Eigen::Matrix3f TBN;
    TBN << t.x(), b.x(), normal.x(),
        t.y(), b.y(), normal.y(),
        t.z(), b.z(), normal.z();
```

由于是针对片元的操作，此shader还需要对Normal Map的采样值进行插值（这一步在基于GPU的渲染管线中是不需要手动运算的）
```c++
//kn kh均为自定义的bump值
float kh = 0.2, kn = 0.1;

float u = payload.tex_coords.x(), v = payload.tex_coords.y();
float w = payload.texture->width, h = payload.texture->height;

// dU = kh * kn * (h(u+1/w,v)-h(u,v))
float du = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() 
			- payload.texture->getColor(u, v).norm());
// dV = kh * kn * (h(u,v+1/h)-h(u,v))
float dv = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() 
			- payload.texture->getColor(u, v).norm());
// Vector ln = (-dU, -dV, 1)
Eigen::Vector3f ln = { -du, -dv, 1.0f };
normal = TBN * ln;
```
##### 3.3.2.2 Displacement Mapping Shader
- **核心原理：** **真正改变几何形状**。它根据一张贴图，在渲染过程中**沿着法线方向移动模型的顶点位置**。
    
- **关键数据：**
    
    - **高度贴图 (Height Map / Displacement Map)：** 一张灰度图。白色（高值）表示顶点沿法线方向**向外移动**（凸起），黑色（低值）表示**向内移动**（凹陷）。值的大小决定了移动的距离。
        
- **工作流程 (通常在顶点着色器或曲面细分阶段)：**
    
    1. **顶点着色器 (简单模型)：** 对于顶点密度足够高的模型，可以在顶点着色器中采样高度贴图，然后根据高度值沿着顶点法线方向**偏移该顶点的位置**。这需要高度贴图的分辨率和模型的顶点密度匹配才能获得细节。
        
    2. **曲面细分管线 (复杂细节，主流方式)：**
        
        - **Hull Shader (外壳着色器)：** 根据摄像机距离、高度图预期变化程度等因素，决定对原始图元（如三角形）进行多少**细分 (Tessellation)**。预期细节丰富的区域细分更多。
            
        - **Tessellator (曲面细分器)：** 根据Hull Shader的指令，在域（Domain，即原始图元）内部生成新的顶点。
            
        - **Domain Shader (域着色器)：** **这是核心！** 对于曲面细分器生成的每个新顶点：
            
            - 计算它在原始图元上的位置坐标 (UV)。
                
            - 使用这个 UV 坐标**采样高度贴图**，得到高度值。
                
            - 根据高度值，沿着该位置插值得到的**法线方向**（或更精确的，根据细分位置计算出的新法线），**偏移该顶点的最终位置**。
                
    3. 位移后的顶点被送入后续的几何/片元着色器进行光栅化和着色。

![300](hw3.3.png)
本作业中的Displacement Shader和之前的Bump Shader相比，唯一增多的是对于片元position的修改。
*——没错，这里采用简化的方式在片元着色器中进行position displacement。如果考虑性能的话，会按照上述采用曲面细分管线，在顶点着色器这一步完成displacement，再进行光栅化。*
![500](hw3.2.png)

对应代码：
```c++
//Displace: change the location of vertices
point += (kn * normal * payload.texture->getColor(u, v).norm()); //extrude along the normal direction
```
# Homework 5
## 1 Intro
《光线与三角形相交》

作业五采用了基础的（Whitted-Style）光线追踪框架。
光线追踪渲染器的程序结构与软光栅的有很大区别。由于它采用更接近真实的渲染算法，所以它包含更多必须类；对场景中的**几何**、灯光等类的定义也会更复杂。

关于光线追踪算法的简要概述：
>在光线追踪中最重要的操作之一就是找到光线与物体的交点。一旦找到光线与物体的交点，就可以执行着色并返回像素颜色。在这次作业中，我们需要实现两个部分：光线的生成和光线与三角的相交。本次代码框架的工作流程为：
1. 从 main 函数开始。我们定义场景的参数，添加物体（球体或三角形）到场景中，并设置其材质，然后将光源添加到场景中。
2. 调用 Render(scene) 函数。在遍历所有像素的循环里，生成对应的光线并将返回的颜色保存在帧缓冲区（framebuffer）中。在渲染过程结束后，帧缓冲区中的信息将被保存为图像。
3. 在生成像素对应的光线后，我们调用 CastRay 函数，该函数调用 trace 来查询光线与场景中最近的对象的交点。
4. 然后，我们在此交点执行着色。我们设置了三种不同的着色情况，并且已经为你提供了代码。

Output:
![400](hw5.output.png)
## 2 Structure
头文件：
- global.hpp: 包含了整个框架中会使用的基本函数和变量。
- Vector.hpp: 由于我们不再使用 Eigen 库，因此我们在此处提供了常见的向量操作，例如：dotProduct，crossProduct，normalize。
- **Object.hpp: 渲染物体的父类。Triangle 和 Sphere 类都是从该类继承的。**
- Scene.hpp: 定义要渲染的场景。包括设置参数，物体以及灯光。
- **Renderer.hpp: 渲染器类，它实现了所有光线追踪的操作。**
源文件：
- main.cpp
- Renderer.cpp
![[hw5.data.struct.png]]
### 2.1 渲染物体类
总结下来，光追渲染框架中的物体类，属性即几何体本身的属性参数，方法有3个主要方法：
1. intersect：判断该object是否与光线相交
2. getSurfaceProperties：获取相交处的材质信息
3. evalDiffuseColor：**（这个暂时没搞清楚）**
#### 2.1.1 Object 父类
Properties:
```c++
materialType(DIFFUSE_AND_GLOSSY)
ior(1.3)
Kd(0.8)
Ks(0.2)
diffuseColor(0.2)
specularExponent(25)
```
Methods:
```c++
bool intersect(const Vector3f&, const Vector3f&, float&, uint32_t&, Vector2f&)
void getSurfaceProperties(const Vector3f&, const Vector3f&, const uint32_t&, const Vector2f&, Vector3f&,vector2f&)
Vector3f evalDiffuseColor(const Vector2f&)
```
##### 2.1.1.1 Triangle类
Properties:
- meshTriangle
Method:
- intersect()
- getSurfaceProperties()
##### 2.1.1.2 Sphere类
Properties:
- center
- radius1, radius2
Method:
- intersect()
- getSurfaceProperties()
```c++
#include "Object.hpp"
#include "Vector.hpp"

class Sphere : public Object
{
	public:
    Sphere(const Vector3f& c, const float& r)
        : center(c)
        , radius(r)
        , radius2(r * r)
    {}
    
    bool intersect(const Vector3f& orig, const Vector3f& dir, float& tnear, uint32_t&, Vector2f&) const override
    {
        // analytic solution
        Vector3f L = orig - center;
        float a = dotProduct(dir, dir);
        float b = 2 * dotProduct(dir, L);
        float c = dotProduct(L, L) - radius2;
        float t0, t1;
        if (!solveQuadratic(a, b, c, t0, t1))
            return false;
        if (t0 < 0)
            t0 = t1;
        if (t0 < 0)
            return false;
        tnear = t0;

        return true;
    }
    
    void getSurfaceProperties(const Vector3f& P, const Vector3f&, const uint32_t&, const Vector2f&, Vector3f& N, Vector2f&) const override
    {
        N = normalize(P - center);
    }
    
    Vector3f center;
    float radius, radius2;
};

```
### 2.2 灯光类
```c++
#include "Vector.hpp"
class Light
{
	public:
	    Light(const Vector3f& p, const Vector3f& i)
	        : position(p)
	        , intensity(i)
	    {}
	    virtual ~Light() = default;
	    Vector3f position;
	    Vector3f intensity;
};
```
灯光属性：
- position
- intensity
### 2.3 Scene类

Properties:
```c++
//Size and fov params of the scene
int width = 1280;
int height = 960;
double fov = 90;
Vector3f backgroundColor = Vector3f(0.235294, 0.67451, 0.843137);

//Ray Tracing params of the scene
int maxDepth = 5;
float epsilon = 0.00001;

//Objects and Lights inside the scene
std::vector<std::unique_ptr<Object> > objects;
std::vector<std::unique_ptr<Light> > lights;
```
Methods:
```c++
add and get lights;
add and get objects;
```

## 3 Algorithm
### 3.1 Main()
In the main function of the program, we create the scene (create objects and lights)
as well as set the options for the render (image width and height, maximum recursion
depth, field-of-view, etc.). 
We then call the render function().
Main函数中，我们创造了Scene对象、Renderer对象，并且设置了这些对象的参数。
然后调用render()函数。
```c++
int main()
{
	//创建场景
    Scene scene(1280, 960);
	//创建Sphere
    auto sph1 = std::make_unique<Sphere>(Vector3f(-1, 0, -12), 2);
    sph1->materialType = DIFFUSE_AND_GLOSSY;
    sph1->diffuseColor = Vector3f(0.6, 0.7, 0.8);
    auto sph2 = std::make_unique<Sphere>(Vector3f(0.5, -0.5, -8), 1.5);
    sph2->ior = 1.5;
    sph2->materialType = REFLECTION_AND_REFRACTION;
    //将Sphere加入Scene
    scene.Add(std::move(sph1));
    scene.Add(std::move(sph2));
    //创建Quad Mesh
    Vector3f verts[4] = {{-5,-3,-6}, {5,-3,-6}, {5,-3,-16}, {-5,-3,-16}};
    uint32_t vertIndex[6] = {0, 1, 3, 1, 2, 3};
    Vector2f st[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    auto mesh = std::make_unique<MeshTriangle>(verts, vertIndex, 2, st);
    mesh->materialType = DIFFUSE_AND_GLOSSY;
    //将Quad加入Scene
    scene.Add(std::move(mesh));
    //将灯光加入Scene
    scene.Add(std::make_unique<Light>(Vector3f(-20, 70, 20), 0.5));
    scene.Add(std::make_unique<Light>(Vector3f(30, 50, -12), 0.5));    
    //创建渲染器
    Renderer r;
    //调用Render()
    r.Render(scene);
    return 0;
}
```
### 3.2 Render(scene)
首先解析Renderer.cpp。渲染整个过程中，有三个环节的算法值得一提：
1. **发射光线**：遍历frame发射eye-pixel方向的光线（光路可逆原理）
2. **光路追踪，判断相交**：根据发射的光线，追踪光线在场景里的传播及相交情况。每个几何体判断是否与光线相交的方式都不一样（几何算法）
3. **递归追踪**：根据相交点的材质，判断下一段光路，递归进行追踪，最终采样到这条光路最末端的incident radiance（递归算法）
*注：HW6是对【环节2】的几何算法进行了优化，采用BVH加速；HW7是对【环节3】的递归算法进行了优化，采用Russian Roulette及BRDF算法节约算力、逼近真实物理效果。*
![[hw5.algorithm.png]]
#### 3.2.1 发射光线
遍历像素，发射eye-pixel ray，即进行①次调用castRay()。
```c++
    // Use this variable as the eye position to start your rays.
    Vector3f eye_pos(0);
    int m = 0;
    for (int j = 0; j < scene.height; ++j)
    {
        for (int i = 0; i < scene.width; ++i)
        {
            // generate primary ray direction
            float x = 0;
            float y = 0;
            // TODO: Find the x and y positions of the current pixel to get the direction
            // vector that passes through it.
            // Also, don't forget to multiply both of them with the variable *scale*, and
            // x (horizontal) variable with the *imageAspectRatio*   
            x = (2 * (i + 0.5) / scene.width - 1) * imageAspectRatio * scale;
            y = (1 - 2 * (j + 0.5) / scene.height) * scale;
            Vector3f dir = Vector3f(x, y, -1); // Don't forget to normalize this direction!
            dir = normalize(dir);
            framebuffer[m++] = castRay(eye_pos, dir, scene, 0);
        }
        UpdateProgress(j / (float)scene.height);
    }
    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (char)(255 * clamp(0, 1, framebuffer[i].x));
        color[1] = (char)(255 * clamp(0, 1, framebuffer[i].y));
        color[2] = (char)(255 * clamp(0, 1, framebuffer[i].z));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
```
#### 3.2.2 追踪光路，判断相交

首先，每一根eye-pixel ray会跟场景里每个物体进行相交判断，最后根据tnear判断究竟最先与哪个物体相交。
castRay() -> trace()
trace()里面就会遍历场景中所有的object

```c++
std::optional<hit_payload> trace(
        const Vector3f &orig, const Vector3f &dir,
        const std::vector<std::unique_ptr<Object> > &objects)
{
    float tNear = kInfinity;
    std::optional<hit_payload> payload;
    for (const auto & object : objects)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (object->intersect(orig, dir, tNearK, indexK, uvK) && tNearK < tNear)
        {
            payload.emplace();
            payload->hit_obj = object.get();
            payload->tNear = tNearK;
            payload->index = indexK;
            payload->uv = uvK;
            tNear = tNearK;
        }
    }

    return payload;
}
```

接下来是判断是否相交的算法细节，最后的目的是确定tnear。再根据tnear算出相交点的各个数据：位置、材质信息……这些信息都会被装载到上一步的hit_payload中。

光线与球体的判断相交
```c++
//Inside Sphere.hpp
bool intersect(const Vector3f& orig, const Vector3f& dir, float& tnear, uint32_t&, Vector2f&) const override
{
    // analytic solution
    Vector3f L = orig - center;
    float a = dotProduct(dir, dir);
    float b = 2 * dotProduct(dir, L);
    float c = dotProduct(L, L) - radius2;
    float t0, t1;
    if (!solveQuadratic(a, b, c, t0, t1))
		return false;
    if (t0 < 0)
	    t0 = t1;
    if (t0 < 0)
	    return false;
    tnear = t0;
    return true;
}
```

光线与三角形的判断相交
```c++
//Inside Triangle.hpp
bool intersect(const Vector3f& orig, const Vector3f& dir, float& tnear, uint32_t& index, Vector2f& uv) const override
{
    bool intersect = false;
    for (uint32_t k = 0; k < numTriangles; ++k)
    {
        const Vector3f& v0 = vertices[vertexIndex[k * 3]];
        const Vector3f& v1 = vertices[vertexIndex[k * 3 + 1]];
        const Vector3f& v2 = vertices[vertexIndex[k * 3 + 2]];
        float t, u, v;
        if (rayTriangleIntersect(v0, v1, v2, orig, dir, t, u, v) && t < tnear)
        {
            tnear = t;
            uv.x = u;
            uv.y = v;
            index = k;
            intersect |= true;
        }
    }
    return intersect;
}

bool rayTriangleIntersect(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2, const Vector3f& orig, const Vector3f& dir, float& tnear, float& u, float& v)
{
    // TODO: Implement this function that tests whether the triangle
    // that's specified bt v0, v1 and v2 intersects with the ray (whose
    // origin is *orig* and direction is *dir*)
    // Also don't forget to update tnear, u and v.
    
    //1 First compute the barycentric coords of the intersect
    
    //2 Test if the intersection is inside the triangle
    //Condition1: u, v, 1-u-v ~ [0,1]
    //Condition2: tnear >0
    
    Vector3f E1 = v1 - v0, E2 = v2 - v0, S = orig - v0;
    Vector3f S1 = crossProduct(dir, E2), S2 = crossProduct(S, E1);
    float S1E1 = dotProduct(S1, E1);
    tnear = 1.0 / S1E1 * dotProduct(S2, E2);
    u = 1.0 / S1E1 * dotProduct(S1, S);
    v = 1.0 / S1E1 * dotProduct(S2, dir);
    
    return tnear > 0 && u >= 0 && u <= 1 && v >= 0 && v <= 1 && 1 - u - v >= 0 && 1 - u - v <= 1;
}
```
#### 3.2.3 迭代追踪

在castRay()中，假如得到了一次hit_payload，需要解析hit_payload中的表面材质信息，根据这个材质信息判断：
- 是否进行下一轮的光线追踪？
- 下一轮的光线追踪，光线方向是怎样的（即新的光线方程是什么）？
	- 反射材质：反射方向
	- 折射材质：折射方向

```c++
Vector3f castRay(
        const Vector3f &orig, const Vector3f &dir, const Scene& scene,
        int depth)
{
    if (depth > scene.maxDepth) {
        return Vector3f(0.0,0.0,0.0);
    }

    Vector3f hitColor = scene.backgroundColor;
    if (auto payload = trace(orig, dir, scene.get_objects()); payload)
    {
        Vector3f hitPoint = orig + dir * payload->tNear;
        Vector3f N; // normal
        Vector2f st; // st coordinates
        payload->hit_obj->getSurfaceProperties(hitPoint, dir, payload->index, payload->uv, N, st);
        switch (payload->hit_obj->materialType) {
            case REFLECTION_AND_REFRACTION:
            {
                Vector3f reflectionDirection = normalize(reflect(dir, N));
                Vector3f refractionDirection = normalize(refract(dir, N, payload->hit_obj->ior));
                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                                             hitPoint - N * scene.epsilon :
                                             hitPoint + N * scene.epsilon;
                Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ?
                                             hitPoint - N * scene.epsilon :
                                             hitPoint + N * scene.epsilon;
                Vector3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1);
                Vector3f refractionColor = castRay(refractionRayOrig, refractionDirection, scene, depth + 1);
                float kr = fresnel(dir, N, payload->hit_obj->ior);
                hitColor = reflectionColor * kr + refractionColor * (1 - kr);
                break;
            }
            case REFLECTION:
            {
                float kr = fresnel(dir, N, payload->hit_obj->ior);
                Vector3f reflectionDirection = reflect(dir, N);
                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                                             hitPoint + N * scene.epsilon :
                                             hitPoint - N * scene.epsilon;
                hitColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1) * kr;
                break;
            }
            default:
            {
                // [comment]
                // We use the Phong illumation model int the default case. The phong model
                // is composed of a diffuse and a specular reflection component.
                // [/comment]
                Vector3f lightAmt = 0, specularColor = 0;
                Vector3f shadowPointOrig = (dotProduct(dir, N) < 0) ?
                                           hitPoint + N * scene.epsilon :
                                           hitPoint - N * scene.epsilon;
                // [comment]
                // Loop over all lights in the scene and sum their contribution up
                // We also apply the lambert cosine law
                // [/comment]
                for (auto& light : scene.get_lights()) {
                    Vector3f lightDir = light->position - hitPoint;
                    // square of the distance between hitPoint and the light
                    float lightDistance2 = dotProduct(lightDir, lightDir);
                    lightDir = normalize(lightDir);
                    float LdotN = std::max(0.f, dotProduct(lightDir, N));
                    // is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
                    auto shadow_res = trace(shadowPointOrig, lightDir, scene.get_objects());
                    bool inShadow = shadow_res && (shadow_res->tNear * shadow_res->tNear < lightDistance2);

                    lightAmt += inShadow ? 0 : light->intensity * LdotN;
                    Vector3f reflectionDirection = reflect(-lightDir, N);

                    specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, dir)),
                        payload->hit_obj->specularExponent) * light->intensity;
                }

                hitColor = lightAmt * payload->hit_obj->evalDiffuseColor(st) * payload->hit_obj->Kd + specularColor * payload->hit_obj->Ks;
                break;
            }
        }
    }
    
    return hitColor;
}
```

*请注意：递归算法的开头一定要有gatekeeping的判断条件。在该作业中，判断条件有二：
1：是否达到迭代最大次数（该数值为hardcoded）？
2：是否有真正的hit？*

# Homework 6
## 1 Intro
在之前的编程练习中，我们实现了基础的光线追踪算法，具体而言是光线传输、光线与三角形求交。**我们采用了这样的方法寻找光线与场景的交点**：遍历场景中的所有物体，判断光线是否与它相交。在场景中的物体数量不大时，该做法可以取得良好的结果，但当物体数量增多、模型变得更加复杂，该做法将会变得非常低效。
代码示意：
```c++
//inside trace() method in render()
for (const auto & object : objects){......}
```

因此，我们需要加速结构来加速求交过程。在本次练习中，我们重点关注物体划分算法 Bounding Volume Hierarchy (BVH)。本练习要求你实现 Ray-Bounding Volume 求交与 BVH 查找。

此次作业涉及到的代码改动：
1. BVH几何数据结构生成部分——引入Bounds3.hpp和BVH.hpp类
2. 判断三角形与光线相交部分——Bounds3.hpp和BVH.hpp类中提供对应的intersection method
为什么有Bounding类，还有BVH类？可以把BVH（Bounding Volume Hierarchy）理解为更高于Bounding包围盒的一个类

最终输出：
![400](hw6.output.png)
## 2 Structure
作业6修改了代码框架中的如下内容：
- Material.hpp: 我们从将材质参数拆分到了一个单独的类中，现在每个物体实例都可以拥有自己的材质。
- **Intersection.hpp**: 这个数据结构包含了相交相关的信息（相当于将作业五里定义在render class里的hit payload独立成了一个类）。
- Ray.hpp: 光线类，包含一条光的源头、方向、传递时间 t 和范围 range.
- **Bounds3.hpp**: 包围盒类，每个包围盒可由 pMin 和 pMax 两点描述（请思考为什么）。Bounds3::Union 函数的作用是将两个包围盒并成更大的包围盒。与材质一样，场景中的每个物体实例都有自己的包围盒。
- **BVH.hpp**: BVH 加速类。场景 scene 拥有一个 BVHAccel 实例。从根节点开始，我们可以递归地从物体列表构造场景的 BVH。
![500](hw6.GeoStruct.png)
### 2.1 Intersection.hpp
Intersection作为中间结构，隔离了更新后的几何数据结构（BVH）和光追的castRay方法：
- 作业五中的castRay通过递归调用trace()，遍历场景中的几何数据。trace()及后续返回的hit_payload被弃用；
- 作业六直接读取调用bvh.Intersect()方法返回的Intersection对象，获取光线与场景的相交信息。

```c++
class Object;
class Sphere;

struct Intersection
{
    Intersection(){
        happened=false;
        coords=Vector3f();
        normal=Vector3f();
        distance= std::numeric_limits<double>::max();
        obj =nullptr;
        m=nullptr;
    }
    bool happened;
    Vector3f coords;
    Vector3f normal;
    double distance;
    Object* obj;
    Material* m;
};
```
### 2.2 Bounds3.hpp
Bounds3可以看作Object类中的一个重要属性。

Bounds3的属性：
```c++
    Vector3f pMin, pMax; // two points to specify the bounding box
    Bounds3(const Vector3f p1, const Vector3f p2)
    {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }
```
两个重要点：pMax和pMin，就能确定包围盒的边界。

Bounds3主要有两类方法：
- 核心方法：判断包围盒是否与Ray相交
	- 核心思想：光线（全面）进入包围盒的时间，应该小于光线（全面）出于包围盒的时间。
```c++
inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
                                const std::array<int, 3>& dirIsNeg) const
{
    Vector3f tmin = (pMin - ray.origin) * invDir, tmax = (pMax - ray.origin) * invDir;

    //Check if ray go in any negative direction
    //If so, we should swap the min and max value
    Vector3f dir = ray.direction;
    if (dir.x < 0) std::swap(tmin.x, tmax.x);
    if (dir.y < 0) std::swap(tmin.y, tmax.y);
    if (dir.z < 0) std::swap(tmin.z, tmax.z);

    float t_enter = fmax(tmin.x, fmax(tmin.y, tmin.z)), t_exit = fmin(tmax.x, fmin(tmax.y, tmax.z));

    return t_enter < t_exit && t_exit >= 0;
}
```
- 普遍方法：针对包围盒的几何运算
```c++
inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
Vector3f Diagonal() const { return pMax - pMin; }
int maxExtent() const
double SurfaceArea() const
Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }
Bounds3 Intersect(const Bounds3& b)
Vector3f Offset(const Vector3f& p) const
bool Overlaps(const Bounds3& b1, const Bounds3& b2)
bool Inside(const Vector3f& p, const Bounds3& b)
inline const Vector3f& operator[](int i) const{return (i == 0) ? pMin : pMax;}
```
### 2.3 BVH.hpp

#### 2.3.1 BVH Build Node
BVH节点结构
```c++
struct BVHBuildNode {
    Bounds3 bounds;
    BVHBuildNode *left;
    BVHBuildNode *right;
    Object* object;

public:
    int splitAxis=0, firstPrimOffset=0, nPrimitives=0;
    // BVHBuildNode Public Methods
    BVHBuildNode(){
        bounds = Bounds3();
        left = nullptr;right = nullptr;
        object = nullptr;
    }
};
```

#### 2.3.2 BVH Accel

BVH树结构

**Public Method and Properties**
```c++
BVHAccel(std::vector<Object*> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE);
Bounds3 WorldBound() const;
~BVHAccel();

Intersection Intersect(const Ray &ray) const;
Intersection getIntersection(BVHBuildNode* node, const Ray& ray)const;
bool IntersectP(const Ray &ray) const;
    
BVHBuildNode* root;
```

**Private Method and Properties**
```c++
// BVHAccel Private Methods
BVHBuildNode* recursiveBuild(std::vector<Object*>objects);
BVHBuildNode* recursiveBuildSAH(std::vector<Object*>objects);
double computeSize(std::vector<Object*> objects);

// BVHAccel Private Properties
const int maxPrimsInNode;
const SplitMethod splitMethod;
std::vector<Object*> primitives;
```

*注意：SAH是一种进阶的包围盒算法。它在生成子包围盒节点的时候，不是直接从中点划分，而是按照一定的几何比例。*
## 3 Algorithm
本次作业主要的算法主要集中于BVH树的构建，以及光线与BVH树的相交判断。

### 3.1 BVHAccel::recursiveBuild
以下就是BVH树生成的过程
1. 遍历所有三角形，生成对应该三角形列表的最大一个包围盒
2. 处理极端场景：如果该三角形列表中，只有一个或两个三角形的情况
3. 处理普遍场景：如果列表含有多个三角形，
	1. 需要将三角形的位置排序（xyz方向上轮流进行）
	2. 根据排序结果（beginning, middling, ending），将各个三角形列表分成2组（leftshapes和rightshapes）
	3. 递归对leftshapes和rightshapes中的三角形们进行BVH结构化处理。处理后返回的根节点，作为此处BHV节点的左右子节点
	4. 最后返回最开始的节点，作为此BVH树的根节点。
```c++
BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();
    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
        
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});
        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }
        
        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();
        
        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);
        
        //Test
        assert(objects.size() == (leftshapes.size() + rightshapes.size()));
        
        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);
        
        node->bounds = Union(node->left->bounds, node->right->bounds);
    }
    return node;
}
```

### 3.2 BVHAccel::getIntersection
总体来说也是个递归算法。
1. 首先判断该节点的包围盒和光线是否相交。如果不相交，直接返回默认的intersection struct.
2. 接着，判断该节点是否为叶子节点。如果为叶子节点，则直接与节点里的三角形本身判断相交。
3. 最后的情况，该节点包含左右子节
4. 点，那么分别向左右两部分递归调用intersection算法，最后对比左右两部分返回的距离，哪个离光线起点最近，哪个即为相交点。
```c++
Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    
    //No intersection
    if (!node->bounds.IntersectP(ray, ray.direction_inv, std::array<int, 3>{ray.direction.x > 0, ray.direction.y > 0, ray.direction.z > 0}))
        return Intersection();
        
    //Intersect with leaf node
    if (node->left == nullptr && node->right == nullptr)
        return node->object->getIntersection(ray);
        
    //Intersect with internal nodes
    auto u = getIntersection(node->left, ray);
    auto v = getIntersection(node->right, ray);
    
    //Return the closer intersection
    return u.distance < v.distance ? u : v; 
}
```

# Homework 7
## 1 Intro
在之前的练习中，我们实现了 Whitted-Style Ray Tracing 算法，并且用 BVH等加速结构对于求交过程进行了加速。在本次实验中，我们将在上一次实验的基础上实现完整的 Path Tracing 算法。

最终输出：

1SSP -> 2SSP -> 16SSP
![200](hw7.binary.jpg) ![200](hw7.binary2SSP.jpg) ![200](hw7.binary16SSP.jpg)

## 2 Structure
Path Tracing基于Whitted-Style光线追踪，多了以下的feature:
1. 多根eye-pixel ray的平均采样
2. 在Material类中引入Monte Carlo式BRDF计算模型，使得castRay()方法能够根据材质类型细化光路轨迹和着色变化
	1. AreaLight Sample接口
3. 在castRay()中引入Russian Roulette概率模型，使得场景中的递归光线传播能在合理范围内停止。

在本次实验中，你只需要修改这一个函数: **castRay(const Ray ray, int depth)** in Scene.cpp，在其中实现 Path Tracing 算法。

可能用到的函数有：
- intersect(const Ray ray)in Scene.cpp: 求一条光线与场景的交点
- sampleLight(Intersection pos, float pdf) in Scene.cpp: 在场景的所有光源上按面积 uniform 地 sample 一个点，并计算该 sample 的概率密度
- sample(const Vector3f wi, const Vector3f N) in Material.cpp: 按照该材质的性质，给定入射方向与法向量，用某种分布采样一个出射方向
- pdf(const Vector3f wi, const Vector3f wo, const Vector3f N) in Material.cpp: 给定一对入射、出射方向与法向量，计算 sample 方法得到该出射方向的概率密度
- eval(const Vector3f wi, const Vector3f wo, const Vector3f N) in Material.cpp: 给定一对入射、出射方向与法向量，计算这种情况下的 f_r 值
可能用到的变量有：
- RussianRoulette in Scene.cpp: P_RR, Russian Roulette 的概率

## 3 Algorithm
Introducing Global Illumination based on Monte Carlo
```
shade(p, wo)
	Randomly choose N direction wi~pdf
	L0 = 0
	For each wi
		Trace a ray r(p,wi)
		If ray r hit the light
			Lo += (1/N) * L_i * f_r * cosine / pdf(wi)
		Else if the ray r hits an object at q
			Lo += (1/N) * shade(q,-wi) * f_r * cosine / pdf(wi)
	return Lo
```

Path Tracing - Reducing the total N samples at one point to 1 
- Mitigation: use Russian Roulette
- Mitigation: generate multiple sample ray from one pixel, and do average then.
否则会在循环中出现次数的指数级爆炸

RR:
```
shade(p, wo)
	
	//RR
	Mannually specify a probability P_RR
	Randomly select ksi in a uniform dist. in [0,1]
	If(ksi > P_RR) return 0;
	
	Randomly choose N direction wi~pdf
	L0 = 0
	For each wi
		Trace a ray r(p,wi)
		If ray r hit the light
			Lo += (1/N) * L_i * f_r * cosine / pdf(wi)
		Else if the ray r hits an object at q
			Lo += (1/N) * shade(q,-wi) * f_r * cosine / pdf(wi)
	return Lo
```

Ray Generation:
```
ray_generation(camPos, pixel)
	Uniformly choose N sample positions within the pixel
	pixel_radiance = 0
	For each sample in the pixel
		Shoot a ray r(camPos, cam_to_sample)
		If the ray r hit the scene at p
		pixel_radiance += 1 / N * shade(p, sample_to_cam)
	Return pixel_radiance
```

Making sampling more efficient - Sampling the light(直接在光源上采样)
If we uniformly sample the hemisphere at the shading point, amount of sample ray will be wasted, since they will not hit the the light source.

delta(半球上的立体角) = delta(光源面积) * cosine(theta) *|| x' - x||

改变积分域，变成从光源出发的积分，作为**直接照明（Direct Illumination）**
![400](hw7.SampleLight.png)

![400](hw7.CompleteShading.png)