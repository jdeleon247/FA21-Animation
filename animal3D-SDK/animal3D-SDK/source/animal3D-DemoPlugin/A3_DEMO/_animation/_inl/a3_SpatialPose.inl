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

	}
	return -1;
}

// scale
inline a3i32 a3spatialPoseSetScale(a3_SpatialPose* spatialPose, const a3f32 sx, const a3f32 sy, const a3f32 sz)
{
	if (spatialPose)
	{

	}
	return -1;
}

// translation
inline a3i32 a3spatialPoseSetTranslation(a3_SpatialPose* spatialPose, const a3f32 tx, const a3f32 ty, const a3f32 tz)
{
	if (spatialPose)
	{

	}
	return -1;
}


//-----------------------------------------------------------------------------

// reset single node pose
inline a3i32 a3spatialPoseReset(a3_SpatialPose* spatialPose)
{
	if (spatialPose)
	{
		spatialPose->transform = a3mat4_identity; // diagonal 1s,
		spatialPose->rotation = a3vec3_zero; // add 0 degres to each axis
		spatialPose->translation = a3vec3_zero; // move 0 units in each direction
		spatialPose->scale = a3vec3_one; // multiply size times 1;

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
		// RST -> mat4

		// M = T * ((R * R * R) * S) -> a3_SpatialPoseEulerOrder for rotation order

		//	  |      tx |
		//	M |	 RS  ty |
		//	  |		 tz |
		//	  | 0 0 0 1 |

		//    | x		|
		//	S |	   y 	|
		//	  |		  z	|

		// Rx | 1  0  0  0 |
		//	  | 0  c -s  0 |
		//	  | 0 +s  c  0 |
		//	  | 0  0  0  1 |

		// Ry | c  0 +s  0 |
		//	  | 0  1  0  0 |
		//	  |-s  0  c  0 |
		//	  | 0  0  0  1 |

		// Rz | c -s  0  0 |
		//	  | +s c  0  0 |
		//	  | 0  0  1  0 |
		//	  | 0  0  0  1 |

		//	  |	1	  x |
		//	T |	  1   y |
		//	  |	    1 z |
		//	  | 0 0 0 1 |

		// set scale and translation -> technically out of order but I don't think that matters?
		a3mat4 out = {spatialPose_in->scale.x, 0, 0, 0,
						0, spatialPose_in->scale.y, 0, 0,
						0, 0, spatialPose_in->scale.z, 0,
						spatialPose_in->translation.x, spatialPose_in->translation.y, spatialPose_in->translation.z, 0};

		// use euler order?
		a3mat4 rotMat = a3mat4_identity;
		a3real4x4SetRotateX(rotMat.m, spatialPose_in->rotation.x);
		a3real4x4SetRotateY(rotMat.m, spatialPose_in->rotation.y);
		a3real4x4SetRotateZ(rotMat.m, spatialPose_in->rotation.z);

		a3real4x4Concat(rotMat.m, out.m);

		*mat_out = out;

	}
	return -1;
}

// copy operation for single node pose
inline a3i32 a3spatialPoseCopy(a3_SpatialPose* spatialPose_out, const a3_SpatialPose* spatialPose_in)
{
	if (spatialPose_out && spatialPose_in)
	{

	}
	return -1;
}

// concatenate/combine two node poses.
inline a3i32 a3spatialPoseConcat(a3_SpatialPose* spatialPose_out, const a3_SpatialPose* spatialPose_lh, const a3_SpatialPose* spatialPose_rh)
{
	if (spatialPose_out && spatialPose_lh && spatialPose_rh)
	{
		// spatialPose_out->transform; // No matrices yet, won't do anything - Dan
		spatialPose_out->rotation; // Euler: add ->validate(lh+rh) -> constrain sum to rotation domain (+-360 degrees)
		a3real3Add(spatialPose_out->rotation.v, spatialPose_lh->rotation.v);
		a3real3Add(spatialPose_out->rotation.v, spatialPose_rh->rotation.v);
		a3clamp(-360, 360, spatialPose_out->rotation.x);
		a3clamp(-360, 360, spatialPose_out->rotation.y);
		a3clamp(-360, 360, spatialPose_out->rotation.z);

		// multiply (lh * rh) -> component-wise.
		a3real3ProductComp(spatialPose_out->scale.v, spatialPose_lh->scale.v, spatialPose_rh->scale.v);
		spatialPose_out->scale; 

		// (lh + rh)
		a3real4Add(spatialPose_out->translation.v, spatialPose_lh->translation.v); 
		a3real4Add(spatialPose_out->translation.v, spatialPose_rh->translation.v);
	}
	return -1;
}

// lerp
inline a3i32 a3spatialPoseLerp(a3_SpatialPose* spatialPose_out,
	const a3_SpatialPose* spatialPose0, const a3_SpatialPose* spatialPose1, const a3real u)
{
	if (spatialPose_out && spatialPose0 && spatialPose1)
	{
		// spatialPose_out->transform; // No matrices yet, won't do anything, also matrix-lerps destroy space fabric - Dan
		spatialPose_out->rotation; // Euler: lerp(p0,p1,u) -> (p1-p0)u + p0;
		spatialPose_out->scale; // lerp is ok... but really... exponent_lerp() -> (p1(p0^(-1)))^u * p0
		spatialPose_out->translation; // lerp(p0,p1,u)
		return 0;
	}

	return 1;
}

//-----------------------------------------------------------------------------


#endif	// !__ANIMAL3D_SPATIALPOSE_INL
#endif	// __ANIMAL3D_SPATIALPOSE_H