/*
	Copyright 2011-2020 Daniel S. Buckstein

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

/*
	animal3D SDK: Minimal 3D Animation Framework
	By Daniel S. Buckstein
	
	passTangentBasis_morph5_transform_vs4x.glsl
	Calculate and pass tangent basis with morphing.
*/

#version 450

// single keyframe in morphing animation
struct sMorphTarget
{
	vec4 position, normal, tangent;
};

layout (location = 0) in sMorphTarget aMorphTarget[5];

/*
layout (location = 0) in vec4 aPosition;
layout (location = 1) in vec4 aNormal;		// usually 2
layout (location = 2) in vec4 aTangent;		// usually 10
layout (location = 2) in vec4 aBitangent;	// usually 11
*/

layout (location = 15) in vec4 aTexcoord;	// usually 8

uniform mat4 uP;
uniform mat4 uMV, uMV_nrm;
uniform mat4 uAtlas;
uniform double uTime;

out vbVertexData {
	mat4 vTangentBasis_view;
	vec4 vTexcoord_atlas;
};

flat out int vVertexID;
flat out int vInstanceID;

vec4 lerp(in vec4 v0, in vec4 v1, in float u);
vec4 nlerp(in vec4 v0, in vec4 v1, in float u);
vec4 CatmullRom(in vec4 vP, in vec4 v0, in vec4 v1, in vec4 vN, in float u);
vec4 nCatmullRom(in vec4 vP, in vec4 v0, in vec4 v1, in vec4 vN, in float u);

sMorphTarget lerp(in sMorphTarget m0, in sMorphTarget m1, in float u)
{
	sMorphTarget m;
	m.position = lerp(m0.position, m1.position, u);
	m.normal = nlerp(m0.normal, m1.normal, u); // normals, tangents, bitangents must always be length 1, so normalize.
	m.tangent = nlerp(m0.tangent, m1.tangent, u); // nlerp does lerp and then normalizes. 
	return m;
}

sMorphTarget CatmullRom(in sMorphTarget mP, in sMorphTarget m0, in sMorphTarget m1, in sMorphTarget mN, in float u)
{
	sMorphTarget m;
	m.position = CatmullRom(mP.position, m0.position, m1.position, mN.position, u);
	m.normal = nCatmullRom(mP.normal, m0.normal, m1.normal, mN.normal, u); // normals, tangents, bitangents must always be length 1, so normalize.
	m.tangent = nCatmullRom(mP.tangent, m0.tangent, m1.tangent, mN.tangent, u); // nlerp does lerp and then normalizes. 
	return m;
}

void main()
{
	// DUMMY OUTPUT: directly assign input position to output position
	//	gl_Position = aPosition;

	float t = float(uTime *0.5);
	float u = fract(t);
	int keyframeCount = 5; //5 morphtargets
	int i0 = int(t) % keyframeCount; // i0 for current index
	int i1 = (i0 + 1) % keyframeCount; // i1 for next index
	int iN = (i1 + 1) % keyframeCount; // iN for next next index
	int iP = (i0 - 1 + keyframeCount) % keyframeCount; // iP for previous index

	sMorphTarget k; // k for keyframe

	sMorphTarget kP =aMorphTarget[iP];
	sMorphTarget k0 =aMorphTarget[i0];
	sMorphTarget k1 =aMorphTarget[i1];
	sMorphTarget kN =aMorphTarget[iN];

	// step
	//k = k0;

	// nearest
	//k = u < 0.5 ? k0 : k1;

	// lerp
	k = lerp(k0,k1,u);

	// CatmullRom
	k = CatmullRom(kP, k0, k1, kN, u);

	vTangentBasis_view = uMV_nrm * mat4(
	k.tangent,
	vec4(cross(k.normal.xyz, k.tangent.xyz), 0.0),
	k.normal,
	vec4(0.0));
	vTangentBasis_view[3] = uMV * k.position;

	//vTangentBasis_view = uMV_nrm * mat4(aTangent, aBitangent, aNormal, vec4(0.0));
	//vTangentBasis_view[3] = uMV * aPosition;
	gl_Position = uP * vTangentBasis_view[3];
	
	vTexcoord_atlas = uAtlas * aTexcoord;

	vVertexID = gl_VertexID;
	vInstanceID = gl_InstanceID;
}
