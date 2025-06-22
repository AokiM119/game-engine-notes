Outline of Game Rendering
1. Basics of Game Rendering
	- Hardware architecture
	- Render data organization
	- Render Pipeline
		- Culling & Visibility
2. **Materials, Shaders and Lighting**
	- **PBR(SG, MR)**
	- **Shader permutation**
	- **Lighting**
		- **Point / Direction lighting**
		- **IBL / Simple GI**
3. Special Rendering
	- Terrian
	- Sky / Fog
	- Post-process
4. Pipeline
	- Forward, deferred rendering, forward-plus
	- Real pipeline with mixed effects
	- Ring buffer and V-sync
	- Tiled-based rendering

渲染第二讲，讲述绘制细节。也就是图元经过顶点着色器之后发生的事情。
![500](5.1.RenderComponent.png)

The Rendering Equation
![500](5.2.RenderEquation.png)

现实生活中，光照情况非常复杂。导致Kajiya的渲染方程并没有那么容易应用。
![500](5.3.ComplexLighting.png)

在渲染流水线中apply Kajiya equation的难点在于以下三个：

**1st Challenge: How to get incoming radiance for any direcion given**
- 1a Visibility to Lights
- 1a Light Source Complexity
**2nd Challenge: Integral of lighting and scattering function on hemisphere is expensive**
- How to do Integral Efficiently on Hardware（使用现代GPU进行积分，即着色过程）
	- Brute-force way sampling
	- Smarter sampling
	- Derive fast analytical solution
		- Simplify the f_r
			- Assumptions the optical properties of materials
			- Mathematical representation of materials
		- Simplify the L_i
			- Deal with directional light, point light and spot light only
			- A mathematical representation of incident light sampling on a hemisphere, e.g. IBL and SH.
**3rd Challenge: To evaluate incoming radiance, we have to compute yet another integral, i.e. rendering equation is recursive**
- Any matter will be light source 

# Render in Game Engine
Starting from simple.

**1 - 首先对光源进行简化**
- Use simple light source as main light
	- Directional light in most cases
	- Point and spot light in special cases
- Use ambient light to hack others
	- A constant to represent mean of complex hemisphere irradiance
- Support in graphics API
**2 - Environment Map Reflection**
- Using environment map to enhance glossary surface reflection
- Using environment mipmap to represent roughness of surface

总结：上述的简化实际上是将半球面的光场简化成以下项
- Main light
	- Dominant Light
- Ambient light
	- Low-frequency of irradiance sphere distribution
- Environment map
	- High-frequency of irradiance sphere distribution

**3 - 材质模型**
Blinn-Phong Material
- 基于光可叠加模型
![[5.4.BlinnPhong.png]]

Problem of Blinn-Phong
- Not energy conservative: Unstable in ray tracing
- Hard to model complex material

**4 - Shadow Map**
![[5.5.ShadowMap.png]]
Problem of Shadow Map
- Resolution is limited on texture
- Depth precision is limited on texture

# Pre-computed Global Illumination
由于场景中大部分的是静态物体，所以可以使用到预计算的思想。

GI is really important, so how to represent indirect light?
- Good compression rate
	- We need to store millions of radiance probes inside a level.
- Easy to do integration with material function
	- Use polynomial calculation to convolute with material BRDF
		- 所以，其实在这里是对GI光照数据**进行多项式拆分**（傅里叶分解），避免计算积分时还需要对Li进行无穷的采样。

Convolution theory
Spatial (domain) convolution = Frequency (domain) sampling?

## 1 - Spherical Harmonics
球谐函数是一系列正交的基函数。
Spherical Harmonics, a mathematical system analogous to the Fourier transform but defined across the surface of a sphere.
![500](5.6.SphericalHarmonics.png)
该基函数的性质有：
- 是基于（球面）空间角的函数
- 二阶导为0，说明表面非常光滑，有利于空间角度分布（distribution）的呈现

Spherical Harmonics Encoding
![500](5.7.Encoding.png)

**SH Store and Shading**
Store: Use 4 RGB textures to store 4\*3=12 SH coefficients
Reconstruct Irradiance In Shader
```
SHL1 shEvaluateL1(vec3 p)
{
	float Y0 = 0.28f; //sqrt(1/4pi)
	float Y1 = 0.48f; //sqrt(3/4pi)
	SHL1 sh;
	sh[0] = Y0;
	sh[1] = Y1*p.x;
	sh[2] = Y1*p.y;
	sh[3] = Y1*p.z;
}
```

**SH Lightmap: Precomputed GI**
- Place (static) irradiance probes into the scene
- **Compress those irradiance probes into SH coefficients**
- **Store SH coefficients into 2D Atlas lightmap textures**

Lightmap UV的设置有很多注意事项，会影响后续的烘焙结果。
![500](5.8.LightmapUV.png)

Lightmap的烘焙算法：其实是光线追踪的一种（progressive），光追的过程中默认的是漫反射材质。
然后把场景中每一点的间接光（indirect
）和直接光（direct），以SH的形式存储下来（存储到光照贴图中）。
Games202中PRT作业，应该也是存储了环境在mesh表面的SH数据，然后在shading的时候复现。只是没有采用存储在光照贴图的方式。

后续场景材质shader需要将lightmap中的SH数据复原出来。
![500](5.9.Baking.png)

**Dynamic lightprobes - 动态光照探针** 
空间体素化，把lightprobe存在的空间以此为点生成许多四面体（需要某种算法实现）。物体在每个四面体中不同的位置时，就可以用四个顶点上的lightprobe值进行插值。
![[5.10.Lightprobe.png]]
- Lightprobe point generation - lightprobe自动生成

**Reflection Probe - 反射探针**
之前无论是lightmap还是动态光照探针，它存储的SH数据都比较低阶，会损失很多环境光中高阶的信息。所以需要反射探针。

# PBR Material

## Microfacet Theory
微平面理论
![[5.11.Microfacet.png]]
微平面的数学建模 - BRDF
![[5.12.BRDF.png]]
对CookTorrance的拆解 - D项
![[GAMES104/Pictures of GAMES104/Ch5/5.13.CookTorrance.D.png]]
对CookTorrance的拆解 - G项
![[5.14.CookTorrance.G.png]]