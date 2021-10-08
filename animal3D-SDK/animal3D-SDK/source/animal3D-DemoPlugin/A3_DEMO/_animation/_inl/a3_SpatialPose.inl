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

	modified by Rory Beebout, Jonathan Deleon
	
	a3_SpatialPose.inl
	Implementation of inline spatial pose operations.
*/


#ifdef __ANIMAL3D_SPATIALPOSE_H
#ifndef __ANIMAL3D_SPATIALPOSE_INL
#define __ANIMAL3D_SPATIALPOSE_INL


//-----------------------------------------------------------------------------

// set rotation values for a single node pose
inline a3i32 a3spatialPoseSetRotation(a3_SpatialPose* spatialPose, const a3f32 rx_degrees, const a3f32 ry_degrees, const a3f32 rz_degrees)
{
	if (spatialPose)
	{
		a3vec4 out = {rx_degrees,ry_degrees,rz_degrees, 1};
		spatialPose->rotate_euler = out;
	}
	return -1;
}

// scale
inline a3i32 a3spatialPoseSetScale(a3_SpatialPose* spatialPose, const a3f32 sx, const a3f32 sy, const a3f32 sz)
{
	if (spatialPose)
	{
		a3vec4 out = {sx,sy,sz, 1};
		spatialPose->scale = out;
	}
	return -1;
}

// translation
inline a3i32 a3spatialPoseSetTranslation(a3_SpatialPose* spatialPose, const a3f32 tx, const a3f32 ty, const a3f32 tz)
{
	if (spatialPose)
	{
		a3vec4 out = {tx,ty,tz, 1};
		spatialPose->translation = out;
	}
	return -1;
}


//-----------------------------------------------------------------------------

// reset single node pose
inline a3i32 a3spatialPoseReset(a3_SpatialPose* spatialPose)
{
	if (spatialPose)
	{
		spatialPose->transform = a3mat4_identity;	// mul | diagonal 1s,
		spatialPose->rotate_quaternion = a3vec4_w;  // mul | multiply rotation times 1
		spatialPose->rotate_euler = a3vec4_zero;	// add | add 0 degres to each axis
		spatialPose->translation = a3vec4_zero;		// mul | move 0 units in each direction
		spatialPose->scale = a3vec4_one;			// add | multiply size times 1;

		// done
		return 0;
	}
	return -1;
}

// convert single node pose to matrix
inline a3i32 a3spatialPoseConvert(a3mat4* mat_out, const a3_SpatialPose* spatialPose_in, const a3_SpatialPoseChannel channel, const a3_SpatialPoseEulerOrder order)
{
	if (mat_out && spatialPose_in)
	{
		a3mat4 rotMat = a3mat4_identity;
		a3mat4 xRotMat;
		a3mat4 yRotMat;
		a3mat4 zRotMat;
		a3real4x4SetRotateX(xRotMat.m, spatialPose_in->rotate_euler.x);
		a3real4x4SetRotateY(yRotMat.m, spatialPose_in->rotate_euler.y);
		a3real4x4SetRotateZ(zRotMat.m, spatialPose_in->rotate_euler.z);
		switch (order)
		{
		case a3poseEulerOrder_xyz:
			a3real4x4Product(rotMat.m, xRotMat.m, yRotMat.m);
			a3real4x4Product(rotMat.m, rotMat.m, zRotMat.m);
			break;
		case a3poseEulerOrder_yzx:
			a3real4x4Product(rotMat.m, yRotMat.m, zRotMat.m);
			a3real4x4Product(rotMat.m, rotMat.m, xRotMat.m);
			break;
		case a3poseEulerOrder_zxy:
			a3real4x4Product(rotMat.m, zRotMat.m, xRotMat.m);
			a3real4x4Product(rotMat.m, rotMat.m, yRotMat.m);
			break;
		case a3poseEulerOrder_yxz:
			a3real4x4Product(rotMat.m, yRotMat.m, xRotMat.m);
			a3real4x4Product(rotMat.m, rotMat.m, zRotMat.m);
			break;
		case a3poseEulerOrder_xzy:
			a3real4x4Product(rotMat.m, xRotMat.m, zRotMat.m);
			a3real4x4Product(rotMat.m, rotMat.m, yRotMat.m);
			break;
		case a3poseEulerOrder_zyx:
			a3real4x4Product(rotMat.m, zRotMat.m, yRotMat.m);
			a3real4x4Product(rotMat.m, rotMat.m, xRotMat.m);
			break;
		}
		// set scale and translation -> technically out of order but I don't think that matters?
		a3mat4 out = {spatialPose_in->scale.x, 0.0f, 0.0f, 0.0f,
						0.0f, spatialPose_in->scale.y, 0.0f, 0.0f,
						0.0f, 0.0f, spatialPose_in->scale.z, 0.0f,
						spatialPose_in->translation.x, spatialPose_in->translation.y, spatialPose_in->translation.z, 1.0f};

		//combine
		//a3real4x4Concat(rotMat.m, out.m);

		*mat_out = out;

	}

	return -1;
}

// copy operation for single node pose
inline a3i32 a3spatialPoseCopy(a3_SpatialPose* spatialPose_out, const a3_SpatialPose* spatialPose_in)
{
	if (spatialPose_out && spatialPose_in)
	{
		spatialPose_out->rotate_euler = spatialPose_in->rotate_euler;
		spatialPose_out->scale = spatialPose_in->scale;
		spatialPose_out->translation = spatialPose_in->translation;
		spatialPose_out->transform = spatialPose_in->transform;
	}
	return -1;
}

// concatenate/combine two node poses.
inline a3i32 a3spatialPoseConcat(a3_SpatialPose* spatialPose_out, const a3_SpatialPose* spatialPose_lh, const a3_SpatialPose* spatialPose_rh,
	const a3boolean usingQuaternions)
{
	if (spatialPose_out && spatialPose_lh && spatialPose_rh)
	{
		if (usingQuaternions)
			spatialPose_out->rotate_quaternion; // Quat: (1h*rh) = (w_l + v_l)(w_r + v_r)
												//				 = w_l*w_r - v_l . v_r) + (w_l*v_r + w_r*v_l + v_l x v_r)
		else
		{
			// Euler: add ->validate(lh+rh) -> constrain sum to rotation domain (+-360 degrees)
			a3real3Sum(spatialPose_out->rotate_euler.v, spatialPose_lh->rotate_euler.v, spatialPose_rh->rotate_euler.v);
		}

		// multiply (lh * rh) -> component-wise.
		a3real3ProductComp(spatialPose_out->scale.v, spatialPose_lh->scale.v, spatialPose_rh->scale.v);

		// (lh + rh)
		a3real3Sum(spatialPose_out->translation.v, spatialPose_lh->translation.v, spatialPose_rh->translation.v);
	}
	return -1;
}

// lerp
inline a3i32 a3spatialPoseLerp(a3_SpatialPose* spatialPose_out,
	const a3_SpatialPose* spatialPose0, const a3_SpatialPose* spatialPose1, const a3real u,
	const a3boolean usingQuaternions)
{
	if (spatialPose_out && spatialPose0 && spatialPose1)
	{
		// spatialPose_out->transform; // No matrices yet, won't do anything, also matrix-lerps destroy space fabric - Dan
		if (usingQuaternions)
		{
			spatialPose_out->rotate_quaternion; // Quat: 3 options...
												// (1) slerp (q0, q1, u)
												// = (sin([1-t]yq0 + sin([t]y)q1) / sin(y)
												//		y = acos(q0 . q1)
												// (2) lerp: non-unit-length -> uniform scale ||| Free squash and stretc(?) |||
												//			s = |q|^2
												// (3) nlerp = normalize(lerp(...)) ||| Free easing |||
		}
		else
		{
			spatialPose_out->rotate_euler; // Euler: lerp(p0,p1,u) -> (p1-p0)u + p0;
		}
			
		spatialPose_out->scale; // lerp is ok... but really... exponent_lerp() -> (p1(p0^(-1)))^u * p0
		spatialPose_out->translation; // lerp(p0,p1,u)

		spatialPose_out->rotate_euler = spatialPose0->rotate_euler;
		spatialPose_out->scale = spatialPose0->scale;
		spatialPose_out->translation = spatialPose0->translation;

		return 0;
	}

	return 1;
}

//-----------------------------------------------------------------------------


#endif	// !__ANIMAL3D_SPATIALPOSE_INL
#endif	// __ANIMAL3D_SPATIALPOSE_H