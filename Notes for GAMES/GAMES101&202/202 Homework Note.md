
# 1 Shadow Map/PCF/PSSR
对于场景中的每个物体都会**默认附带一个 ShadowMaterial 材质**，并会在调用 **loadOBJ** 时添加到 WebGLRenderer 的 shadowMeshes[] 中。
```javascript
//inside loadOBJ.js
switch (objMaterial) 
{
	case 'PhongMaterial':
	material = buildPhongMaterial(colorMap, mat.specular.toArray(), light, Translation, Scale, "./src/shaders/phongShader/phongVertex.glsl", "./src/shaders/phongShader/phongFragment.glsl");
	shadowMaterial = buildShadowMaterial(light, Translation, Scale, "./src/shaders/shadowShader/shadowVertex.glsl", "./src/shaders/shadowShader/shadowFragment.glsl");
	break;
}
material.then((data) => {
	let meshRender = new MeshRender(renderer.gl, mesh, data);
	renderer.addMeshRender(meshRender);
});

shadowMaterial.then((data) => {
	let shadowMeshRender = new MeshRender(renderer.gl, mesh, data);
	renderer.addShadowMeshRender(shadowMeshRender);
});
```

```javascript
//inside engine.js
loadOBJ(renderer, 'assets/mary/', 'Marry', 'PhongMaterial', obj1Transform);
loadOBJ(renderer, 'assets/mary/', 'Marry', 'PhongMaterial', obj2Transform);
loadOBJ(renderer, 'assets/floor/', 'floor', 'PhongMaterial', floorTransform);

//Loop renderer
function mainLoop(now) {
	cameraControls.update();

	renderer.render();
	requestAnimationFrame(mainLoop);
}
requestAnimationFrame(mainLoop);
```

当光源参数hasShadowMap 为 true 时，作业框架将开启 Shadow Map 功能，在**正式渲染场景之前会以 ShadowMaterial 材质先渲染一遍 shadowMeshes[] 中的物体**，从而生成我们需要的 ShadowMap。
```javascript
//inside WebGLRenderer.js render() method
for (let l = 0; l < this.lights.length; l++) {
	// Draw light
	// TODO: Support all kinds of transform
	this.lights[l].meshRender.mesh.transform.translate = this.lights[l].entity.lightPos;
	this.lights[l].meshRender.draw(this.camera);
	
	// Shadow pass
	if (this.lights[l].entity.hasShadowMap == true) 
	{
	    for (let i = 0; i < this.shadowMeshes.length; i++) 
	    {
	        this.shadowMeshes[i].draw(this.camera);
	    }
	}
	
	// Camera pass
	for (let i = 0; i < this.meshes.length; i++)
	{
	    this.gl.useProgram(this.meshes[i].shader.program.glShaderProgram);
	    this.gl.uniform3fv(this.meshes[i].shader.program.uniforms.uLightPos, this.lights[l].entity.lightPos);
	    this.meshes[i].draw(this.camera);
	}
}
```

如何区分shadow material和phong material？这里没有引入material ID的概念，而是非常临时的一个判断条件：如果有特定绑定到framebuffer（shadow map）上，即为shadow material；如果没有特定指定framebuffer，那就是默认渲染到屏幕上，即为phong material。
```javascript
//inside MeshRender.js draw() method
	draw(camera) {
		const gl = this.gl;

		//对于PhongShader来说，也会执行bindFrameBuffer
		//但绑定到空的FrameBuffer上相当于绑定到默认FrameBuffer（屏幕）上，所以最终绘制结果是在屏幕
		gl.bindFramebuffer(gl.FRAMEBUFFER, this.material.frameBuffer);

		if (this.material.frameBuffer != null) { //说明不是shadow material
			// Shadow map
			gl.viewport(0.0, 0.0, resolution, resolution);
		} else {
			gl.viewport(0.0, 0.0, window.screen.width, window.screen.height);
		}

		gl.useProgram(this.shader.program.glShaderProgram);

		// Bind geometry information
		this.bindGeometryInfo();

		// Bind Camera parameters
		this.bindCameraParameters(camera);

		// Bind material parameters
		this.bindMaterialParameters();

		// Draw
		{
			const vertexCount = this.mesh.count;
			const type = gl.UNSIGNED_SHORT;
			const offset = 0;
			gl.drawElements(gl.TRIANGLES, vertexCount, type, offset);
		}
	}
```

# 2 SSR
# 3 PRT
# 4 Denoise


