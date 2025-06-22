
# CHAPTER 01. INTRODUCTION
### 1.1 LITERATE PROGRAMMING
介绍本书采用的伪代码。
The +≡ symbol after the fragment name shows that we have added to a previously defined fragment.
In this way, we can decompose complex functions into logically distinct parts, making them much easier to understand.
![400](1.1.+=.png)
这里符号，能够让我们对***Do something interesting***这个函数片段进行拓展。InitializeSomethingInteresting()后面的脚标486意味着这个函数的定义出现在第486页。

### 1.2 PHOTOREALISTIC RENDERING AND THE RAY TRACING ALGORITHM
The goal of photorealistic rendering is to create an image of a 3D scene that is indistinguishable from a photograph of the same scene.
PBR渲染的目的，是创造出与真实场景摄影几乎无异的图片。

Single-minded focus on realistic simulation of light.
追求真实的对光的模拟。

In this sense, our physical understanding has come full circle: once we turn to very small scales, light again betrays a particle-like behavior that coexists with its overall wave-like nature.
PBR的理念也是将光“分割”为“单位”，在物理学上与光的波粒性相一致。

Ray tracing is conceptually a simple algorithm; it is based on following the path of a ray of light through a scene as it interacts with and bounces off objects in an environment.
光线追踪是概念上相对简单的一种算法，它的核心在于：
- 光在场景中的传播路径
- 光在场景中与物体的交互

All such systems simulate at least the following objects and phenomena:
光追渲染器中最重要的对象：
1. **Cameras**: A camera model determines how and from where the scene is being viewed, including how an image of the scene is recorded on a sensor. Many rendering systems generate viewing rays starting at the camera that are then traced into the scene to determine which objects are visible at each pixel. 
2. **Ray–object intersections**: We must be able to tell precisely where a given ray intersects a given geometric object. In addition, we need to determine certain properties of the object at the intersection point, such as a surface normal or its material. Most ray tracers also have some facility for testing the intersection of a ray with multiple objects, typically returning the closest intersection along the ray. 
3. **Light sources:** Without lighting, there would be little point in rendering a scene. A ray tracer must model the distribution of light throughout the scene, including not only the locations of the lights themselves but also the way in which they distribute their energy throughout space. 
4. **Visibility**: In order to know whether a given light deposits energy at a point on a surface, we must know whether there is an uninterrupted path from the point to the light source. Fortunately, this question is easy to answer in a ray tracer, since we can just construct the ray from the surface to the light, find the closest ray object intersection, and compare the intersection distance to the light distance. 
5. **Light scattering at surfaces**: Each object must provide a description of its appearance, including information about how light interacts with the object’s surface, as well as the nature of the reradiated (or scattered) light. Models for surface scattering are typically parameterized so that they can simulate a variety of appearances. 
6. **Indirect light transport:** Because light can arrive at a surface after bouncing off or passing through other surfaces, it is usually necessary to trace additional rays to capture this effect. 
7. **Ray propagation:** We need to know what happens to the light traveling along a ray as it passes through space. If we are rendering a scene in a vacuum, light energy remains constant along a ray. Although true vacuums are unusual on Earth, they are a reasonable approximation for many environments. More sophisticated models are available for tracing rays through fog, smoke, the Earth’s atmosphere, and so on.

光追算法链路：
Light source -> Ray propagation -> Ray-Object Intersection -> Light scattering at surfaces -> Indirect light transport -> Visibility -> Cameras

#### 1.2.1 CAMERAS AND FILM
#### 1.2.2 RAY–OBJECT INTERSECTIONS
#### 1.2.3 LIGHT DISTRIBUTION
#### 1.2.4 VISIBILITY
#### 1.2.5 LIGHT SCATTERING AT SURFACES
#### 1.2.6 INDIRECT LIGHT TRANSPORT
#### 1.2.7 RAY PROPAGATION

### 1.3 pbrt: SYSTEM OVERVIEW
#### 1.3.1 PHASES OF EXECUTION
#### 1.3.2 pbrt’S main() FUNCTION
#### 1.3.3 INTEGRATOR INTERFACE
#### 1.3.4 ImageTileIntegrator AND THE MAIN RENDERING LOOP
#### 1.3.5 RayIntegrator IMPLEMENTATION
#### 1.3.6 RANDOM WALK INTEGRATOR
