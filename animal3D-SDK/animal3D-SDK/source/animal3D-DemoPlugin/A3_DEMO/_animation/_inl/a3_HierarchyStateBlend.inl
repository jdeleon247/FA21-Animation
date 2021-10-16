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
	
	a3_HierarchyStateBlend.inl
	Implementation of inline hierarchical blend operations.
*/


#ifdef __ANIMAL3D_HIERARCHYSTATEBLEND_H
#ifndef __ANIMAL3D_HIERARCHYSTATEBLEND_INL
#define __ANIMAL3D_HIERARCHYSTATEBLEND_INL


//-----------------------------------------------------------------------------

// Fundamental Blend Operations
// (Pointer-based)

// reset/identity operation for single spatial pose
inline a3_SpatialPose* a3spatialPoseOpIdentity(a3_SpatialPose* pose_out)
{
	pose_out->transform = a3mat4_identity;
	pose_out->orientation = a3vec4_w;
	pose_out->angles = a3vec4_w;
	pose_out->scale = a3vec4_one;
	pose_out->translation = a3vec4_w;

	// done
	return pose_out;
}

// Equivalent to a constructor, this operation returns/sets a pose constructed using the components provided.
inline a3_SpatialPose* a3spatialPoseOpConstruct(a3_SpatialPose* pose_out, a3vec4 rotation, a3vec4 scale, a3vec4 translation)
{
	pose_out->angles = rotation;
	pose_out->scale = scale;
	pose_out->translation = translation;

	return pose_out;
}

// returns/sets the unchanged control pose
inline a3_SpatialPose* a3spatialPoseOpCopy(a3_SpatialPose* pose_out, a3_SpatialPose* pose_in)
{
	pose_out->transform = pose_in->transform;
	pose_out->angles = pose_in->angles;
	pose_out->orientation = pose_in->orientation;
	pose_out->scale = pose_in->scale;
	pose_out->translation = pose_in->translation;

	return pose_out;
}

// calculates the opposite/inverse pose description that "undoes" the control pose.
inline a3_SpatialPose* a3spatialPoseOpInvert(a3_SpatialPose* pose_out, a3_SpatialPose* pose_in)
{
	a3quatGetInverse(pose_out->orientation.v, pose_in->orientation.v);
	a3real4GetNegative(pose_out->angles.v, pose_in->angles.v); // Additive -> make negative
	a3real4DivComp(pose_out->scale.v, pose_in->scale.v); // Multiplicatve -> (?) divide (?)
	a3real4GetNegative(pose_out->translation.v, pose_in->translation.v);  // Additive -> make negative
	
	return pose_out;
}

// piecewise concatenation of each part of a spatialPose
inline a3_SpatialPose* a3spatialPoseOpConcat(a3_SpatialPose* pose_out, a3_SpatialPose* pose_lh, a3_SpatialPose* pose_rh)
{
	a3spatialPoseConcat(pose_out, pose_lh, pose_rh);
	return 0;
}

// selects one of the two control poses using nearest interpolation.
inline a3_SpatialPose* a3spatialPoseOpNearest(a3_SpatialPose* pose_out, a3_SpatialPose const* pose0, a3_SpatialPose const* pose1, a3real const u)
{
	if (u >= 0.5)
	{
		pose_out = pose0;
	}
	else
	{
		pose_out = pose1;
	}
	return pose_out;
}

// LERP operation for single spatial pose
inline a3_SpatialPose* a3spatialPoseOpLERP(a3_SpatialPose* pose_out, a3_SpatialPose const* pose0, a3_SpatialPose const* pose1, a3real const u)
{
	a3real4Slerp(pose_out->orientation.v, pose0->orientation.v, pose1->orientation.v, u);
	a3real4Lerp(pose_out->angles.v, pose0->angles.v, pose1->angles.v, u);
	a3real4Lerp(pose_out->scale.v, pose0->scale.v, pose1->scale.v, u);
	a3real4Lerp(pose_out->translation.v, pose0->translation.v, pose1->translation.v, u);
	
	return pose_out;
}

// Cubic interpolation for poses
inline a3_SpatialPose* a3spatialPoseOpCubic(a3_SpatialPose* pose_out, a3_SpatialPose const* posePrev, a3_SpatialPose const* pose0,
	a3_SpatialPose const* pose1, a3_SpatialPose const* poseNext, a3real const u)
{
	a3real4CatmullRom(pose_out->orientation.v, posePrev->orientation.v, pose0->orientation.v, pose1->orientation.v, poseNext->orientation.v, u); // Maybe not right
	a3real4CatmullRom(pose_out->angles.v, posePrev->angles.v, pose0->angles.v, pose1->angles.v, poseNext->angles.v, u);
	a3real4CatmullRom(pose_out->scale.v, posePrev->scale.v, pose0->scale.v, pose1->scale.v, poseNext->scale.v, u);
	a3real4CatmullRom(pose_out->translation.v, posePrev->translation.v, pose0->translation.v, pose1->translation.v, poseNext->translation.v, u);
	return 0;
}


//-----------------------------------------------------------------------------

// Derivative Blend Operations
// (Pointer-based)

// calculates the "difference" or "split" between the two control poses.
inline a3_SpatialPose* a3spatialPoseOpSplit(a3_SpatialPose* pose_out, a3_SpatialPose* pose_lh, a3_SpatialPose* pose_rh)
{

	return 0;
}

// calculates the "scaled" pose, which is some blend between the identity pose and the control pose.
inline a3_SpatialPose* a3spatialPoseOpScale(a3_SpatialPose* pose_out, a3_SpatialPose const* pose, a3real const u)
{

	return 0;
}

// triangular interpolation for poses.
inline a3_SpatialPose* a3spatialPoseOpTriangular(a3_SpatialPose* pose_out, a3_SpatialPose const* pose0, a3_SpatialPose const* pose1, a3_SpatialPose const* pose2, a3real const u0, a3real const u1)
{

	return 0;
}

// bilinear nearest function for poses.
inline a3_SpatialPose* a3spatialPoseOpBiNearest(a3_SpatialPose* pose_out, a3_SpatialPose const* pose10, a3_SpatialPose const* pose11,
	a3_SpatialPose const* pose00, a3_SpatialPose const* pose01, a3real const u0, a3real const u1, a3real const u)
{

	return 0;
}

// bilinear interpolation function for poses.
inline a3_SpatialPose* a3spatialPoseOpBiLinear(a3_SpatialPose* pose_out, a3_SpatialPose const* pose10, a3_SpatialPose const* pose11,
	a3_SpatialPose const* pose00, a3_SpatialPose const* pose01, a3real const u0, a3real const u1, a3real const u)
{

	return 0;
}

// bicubic interpolation algorithm for poses.
inline a3_SpatialPose* a3spatialPoseOpBiCubic(a3_SpatialPose* pose_out, a3_SpatialPose const* posePrev0, a3_SpatialPose const* pose00, a3_SpatialPose const* pose01, a3_SpatialPose const* poseNext0,
	a3_SpatialPose const* posePrev1, a3_SpatialPose const* pose10, a3_SpatialPose const* pose11, a3_SpatialPose const* poseNext1,
	a3_SpatialPose const* posePrev2, a3_SpatialPose const* pose20, a3_SpatialPose const* pose21, a3_SpatialPose const* poseNext2,
	a3_SpatialPose const* posePrev3, a3_SpatialPose const* pose30, a3_SpatialPose const* pose31, a3_SpatialPose const* poseNext3,
	a3real const u0, a3real const u1, a3real const u2, a3real const u3, a3real const u)
{

	return 0;
}

//-----------------------------------------------------------------------------

// data-based reset/identity
inline a3_SpatialPose a3spatialPoseDOpIdentity()
{
	a3_SpatialPose const result = { a3mat4_identity, a3vec4_w, a3vec4_w, a3vec4_one, a3vec4_w /*, ...*/ };
	return result;
}

// data-based LERP
inline a3_SpatialPose a3spatialPoseDOpLERP(a3_SpatialPose const pose0, a3_SpatialPose const pose1, a3real const u)
{
	a3_SpatialPose result = { 0 };
	// ...

	// done
	return result;
}


//-----------------------------------------------------------------------------

// pointer-based reset/identity operation for hierarchical pose
inline a3_HierarchyPose* a3hierarchyPoseOpIdentity(a3_HierarchyPose* pose_out)
{

	// done
	return pose_out;
}

// pointer-based LERP operation for hierarchical pose
inline a3_HierarchyPose* a3hierarchyPoseOpLERP(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose0, a3_HierarchyPose const* pose1, a3real const u)
{

	// done
	return pose_out;
}


//-----------------------------------------------------------------------------


#endif	// !__ANIMAL3D_HIERARCHYSTATEBLEND_INL
#endif	// __ANIMAL3D_HIERARCHYSTATEBLEND_H