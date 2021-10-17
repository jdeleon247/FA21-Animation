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
	return pose_out;
}

// selects one of the two control poses using nearest interpolation.
inline a3_SpatialPose* a3spatialPoseOpNearest(a3_SpatialPose* pose_out, a3_SpatialPose const* pose0, a3_SpatialPose const* pose1, a3real const u)
{
	if (u >= 0.5)
	{
		a3spatialPoseCopy(pose_out, pose0);
	}
	else
	{
		a3spatialPoseCopy(pose_out, pose1);
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
	return pose_out;
}


//-----------------------------------------------------------------------------

// Derivative Blend Operations
// (Pointer-based)

// calculates the "difference" or "split" between the two control poses.
inline a3_SpatialPose* a3spatialPoseOpSplit(a3_SpatialPose* pose_out, a3_SpatialPose* pose_lh, a3_SpatialPose* pose_rh)
{
	pose_out->angles.x = a3trigValid_sind(pose_lh->angles.x - pose_rh->angles.x);
	pose_out->angles.y = a3trigValid_sind(pose_lh->angles.y - pose_rh->angles.y);
	pose_out->angles.z = a3trigValid_sind(pose_lh->angles.z - pose_rh->angles.z);

	pose_out->scale.x = pose_lh->scale.x / pose_rh->scale.x;
	pose_out->scale.y = pose_lh->scale.y / pose_rh->scale.y;
	pose_out->scale.z = pose_lh->scale.z / pose_rh->scale.z;

	pose_out->translation.x = pose_lh->translation.x - pose_rh->translation.x;
	pose_out->translation.y = pose_lh->translation.y - pose_rh->translation.y;
	pose_out->translation.z = pose_lh->translation.z - pose_rh->translation.z;

	return pose_out;
}

// calculates the "scaled" pose, which is some blend between the identity pose and the control pose.
inline a3_SpatialPose* a3spatialPoseOpScale(a3_SpatialPose* pose_out, a3_SpatialPose const* pose, a3real const u)
{
	a3spatialPoseOpLERP(pose_out, a3spatialPoseOpIdentity(pose_out), pose, u);
	return pose_out;
}

// triangular interpolation for poses.
inline a3_SpatialPose* a3spatialPoseOpTriangular(a3_SpatialPose* pose_out, a3_SpatialPose const* pose0, a3_SpatialPose const* pose1, a3_SpatialPose const* pose2, a3real const u0, a3real const u1)
{
	a3real u = 1 - u0 - u1;
	pose_out = a3spatialPoseOpConcat(pose_out, a3spatialPoseOpConcat(pose_out, a3spatialPoseOpScale(pose_out, pose0, u), a3spatialPoseOpScale(pose_out, pose1, u0)), a3spatialPoseOpScale(pose_out, pose2, u1));
	return pose_out;
}

// bilinear nearest function for poses.
inline a3_SpatialPose* a3spatialPoseOpBiNearest(a3_SpatialPose* pose_out, a3_SpatialPose const* pose00, a3_SpatialPose const* pose01,
	a3_SpatialPose const* pose10, a3_SpatialPose const* pose11, a3real const u0, a3real const u1, a3real const u)
{
	a3_SpatialPose* tmpPose0;
	a3_SpatialPose* tmpPose1;
	a3spatialPoseOpNearest(tmpPose0, pose00, pose01, u0);
	a3spatialPoseOpNearest(tmpPose1, pose10, pose11, u1);

	a3spatialPoseOpNearest(pose_out, tmpPose0, tmpPose1, u);
	return pose_out;
}

// bilinear interpolation function for poses.
inline a3_SpatialPose* a3spatialPoseOpBiLerp(a3_SpatialPose* pose_out, a3_SpatialPose const* pose00, a3_SpatialPose const* pose01,
	a3_SpatialPose const* pose10, a3_SpatialPose const* pose11, a3real const u0, a3real const u1, a3real const u)
{
	a3_SpatialPose* tmpPose0;
	a3_SpatialPose* tmpPose1;
	a3spatialPoseLerp(tmpPose0, pose00, pose01, u0);
	a3spatialPoseLerp(tmpPose1, pose10, pose11, u0);

	a3spatialPoseLerp(pose_out, tmpPose0, tmpPose1, u);
	return pose_out;
}

// bicubic interpolation algorithm for poses.
inline a3_SpatialPose* a3spatialPoseOpBiCubic(a3_SpatialPose* pose_out, a3_SpatialPose const* posePrev0, a3_SpatialPose const* pose00, a3_SpatialPose const* pose01, a3_SpatialPose const* poseNext0,
	a3_SpatialPose const* posePrev1, a3_SpatialPose const* pose10, a3_SpatialPose const* pose11, a3_SpatialPose const* poseNext1,
	a3_SpatialPose const* posePrev2, a3_SpatialPose const* pose20, a3_SpatialPose const* pose21, a3_SpatialPose const* poseNext2,
	a3_SpatialPose const* posePrev3, a3_SpatialPose const* pose30, a3_SpatialPose const* pose31, a3_SpatialPose const* poseNext3,
	a3real const u0, a3real const u1, a3real const u2, a3real const u3, a3real const u)
{
	a3_SpatialPose* tmpPose0;
	a3_SpatialPose* tmpPose1;
	a3_SpatialPose* tmpPose2;
	a3_SpatialPose* tmpPose3;

	a3spatialPoseOpCubic(tmpPose0, posePrev0, pose00, pose01, poseNext0, u0);
	a3spatialPoseOpCubic(tmpPose1, posePrev1, pose10, pose11, poseNext1, u1);
	a3spatialPoseOpCubic(tmpPose2, posePrev2, pose20, pose21, poseNext2, u2);
	a3spatialPoseOpCubic(tmpPose3, posePrev3, pose30, pose31, poseNext3, u3);

	/*
	a3spatialPoseOpCubic(tmpPose0, posePrev0, posePrev1, posePrev2, posePrev3, u0);
	a3spatialPoseOpCubic(tmpPose1, pose00, pose10, pose20, pose30, u1);
	a3spatialPoseOpCubic(tmpPose2, pose01, pose11, pose21, pose31, u2);
	a3spatialPoseOpCubic(tmpPose3, poseNext0, poseNext1, poseNext2, poseNext3, u3);
	*/

	a3spatialPoseOpCubic(pose_out, tmpPose0, tmpPose1, tmpPose2, tmpPose3, u3);

	return pose_out;
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

// Fundamental Blend Operations
// (Pointer-based)

// reset/identity operation for hierarchical pose
inline a3_HierarchyPose* a3hierarchyPoseOpIdentity(a3_HierarchyPose* pose_out, a3ui32 numNodes)
{
	for (a3index i = 0; i < numNodes; ++i)
	{
		a3spatialPoseOpIdentity(pose_out->pose + i);
	}
	return pose_out;
}

// Equivalent to a constructor, this operation returns/sets a pose constructed using the components provided.
inline a3_HierarchyPose* a3hierarchyPoseOpConstruct(a3_HierarchyPose* pose_out, a3vec4 rotation, a3vec4 scale, a3vec4 translation, a3ui32 numNodes)
{
	for (a3index i = 0; i < numNodes; ++i)
	{
		a3spatialPoseOpConstruct(pose_out->pose + i, rotation, scale, translation);
	}
	return pose_out;
}

// returns/sets the unchanged control pose
inline a3_HierarchyPose* a3hierarchyPoseOpCopy(a3_HierarchyPose* pose_out, a3_HierarchyPose* pose_in, a3ui32 numNodes)
{
	for (a3index i = 0; i < numNodes; ++i)
	{
		a3spatialPoseOpCopy(pose_out->pose + i, pose_in->pose + i);
	}
	return pose_out;
}

// calculates the opposite/inverse pose description that "undoes" the control pose.
inline a3_HierarchyPose* a3hierarchyPoseOpInvert(a3_HierarchyPose* pose_out, a3_HierarchyPose* pose_in, a3ui32 numNodes)
{
	for (a3index i = 0; i < numNodes; ++i)
	{
		a3spatialPoseOpInvert(pose_out->pose + i, pose_in->pose + i);
	}
	return pose_out;
}

// piecewise concatenation of each part of a spatialPose
inline a3_HierarchyPose* a3hierarchyPoseOpConcat(a3_HierarchyPose* pose_out, a3_HierarchyPose* pose_lh, a3_HierarchyPose* pose_rh, a3ui32 numNodes)
{
	for (a3index i = 0; i < numNodes; ++i)
	{
		a3spatialPoseOpConcat(pose_out->pose + i, pose_lh->pose + i, pose_rh->pose + i);
	}
	return pose_out;
}

// selects one of the two control poses using nearest interpolation.
inline a3_HierarchyPose* a3hierarchyPoseOpNearest(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose0, a3_HierarchyPose const* pose1, a3real const u, a3ui32 numNodes)
{
	for (a3index i = 0; i < numNodes; ++i)
	{
		a3spatialPoseOpNearest(pose_out->pose + i, pose0->pose + i, pose1->pose + i, u);
	}
	return pose_out;
}

// LERP operation for single spatial pose
inline a3_HierarchyPose* a3hierarchyPoseOpLERP(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose0, a3_HierarchyPose const* pose1, a3real const u, a3ui32 numNodes)
{
	for (a3index i = 0; i < numNodes; ++i)
	{
		a3spatialPoseOpLERP(pose_out->pose + i, pose0->pose + i, pose1->pose + i, u);
	}
	return pose_out;
}

// Cubic interpolation for poses
inline a3_HierarchyPose* a3hierarchyPoseOpCubic(a3_HierarchyPose* pose_out, a3_HierarchyPose const* posePrev, a3_HierarchyPose const* pose0,
	a3_HierarchyPose const* pose1, a3_HierarchyPose const* poseNext, a3real const u, a3ui32 numNodes)
{
	for (a3index i = 0; i < numNodes; ++i)
	{
		a3spatialPoseOpCubic(pose_out->pose + i, posePrev->pose + i, pose0->pose + i, pose1->pose + i, poseNext->pose + i, u);
	}
	return pose_out;
}


//-----------------------------------------------------------------------------

// Derivative Blend Operations
// (Pointer-based)

// calculates the "difference" or "split" between the two control poses.
inline a3_HierarchyPose* a3hierarchyPoseOpSplit(a3_HierarchyPose* pose_out, a3_HierarchyPose* pose_lh, a3_HierarchyPose* pose_rh, a3ui32 numNodes)
{
	for (a3index i = 0; i < numNodes; ++i)
	{
		a3spatialPoseOpSplit(pose_out->pose + i, pose_lh->pose + i, pose_rh->pose + i);
	}
	return pose_out;
}

// calculates the "scaled" pose, which is some blend between the identity pose and the control pose.
inline a3_HierarchyPose* a3hierarchyPoseOpScale(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose, a3real const u, a3ui32 numNodes)
{
	for (a3index i = 0; i < numNodes; ++i)
	{
		a3spatialPoseOpScale(pose_out->pose + i, pose->pose + i, u);
	}
	return pose_out;
}

// triangular interpolation for poses.
inline a3_HierarchyPose* a3hierarchyPoseOpTriangular(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose0, a3_HierarchyPose const* pose1, a3_HierarchyPose const* pose2, a3real const u0, a3real const u1, a3ui32 numNodes)
{
	for (a3index i = 0; i < numNodes; ++i)
	{
		a3spatialPoseOpTriangular(pose_out->pose + i, pose0->pose + i, pose1->pose + i, pose2->pose + i, u0, u1);
	}
	return pose_out;
}

// bilinear nearest function for poses.
inline a3_HierarchyPose* a3hierarchyPoseOpBiNearest(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose00, a3_HierarchyPose const* pose01,
	a3_HierarchyPose const* pose10, a3_HierarchyPose const* pose11, a3real const u0, a3real const u1, a3real const u, a3ui32 numNodes)
{
	for (a3index i = 0; i < numNodes; ++i)
	{
		a3spatialPoseOpBiNearest(pose_out->pose + i, pose00->pose + i, pose01->pose + i, pose10->pose + i, pose11->pose + i, u0, u1, u);
	}
	return pose_out;
}

// bilinear interpolation function for poses.
inline a3_HierarchyPose* a3hierarchyPoseOpBiLerp(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose00, a3_HierarchyPose const* pose01,
	a3_HierarchyPose const* pose10, a3_HierarchyPose const* pose11, a3real const u0, a3real const u1, a3real const u, a3ui32 numNodes)
{
	for (a3index i = 0; i < numNodes; ++i)
	{
		a3spatialPoseOpBiLerp(pose_out->pose + i, pose00->pose + i, pose01->pose + i, pose10->pose + i, pose11->pose + i, u0, u1, u);
	}
	return pose_out;
}

// bicubic interpolation algorithm for poses.
inline a3_HierarchyPose* a3hierarchyPoseOpBiCubic(a3_HierarchyPose* pose_out, a3_HierarchyPose const* posePrev0, a3_HierarchyPose const* pose00, a3_HierarchyPose const* pose01, a3_HierarchyPose const* poseNext0,
	a3_HierarchyPose const* posePrev1, a3_HierarchyPose const* pose10, a3_HierarchyPose const* pose11, a3_HierarchyPose const* poseNext1,
	a3_HierarchyPose const* posePrev2, a3_HierarchyPose const* pose20, a3_HierarchyPose const* pose21, a3_HierarchyPose const* poseNext2,
	a3_HierarchyPose const* posePrev3, a3_HierarchyPose const* pose30, a3_HierarchyPose const* pose31, a3_HierarchyPose const* poseNext3,
	a3real const u0, a3real const u1, a3real const u2, a3real const u3, a3real const u, a3ui32 numNodes)
{
	for (a3index i = 0; i < numNodes; ++i)
	{
		a3spatialPoseOpBiCubic(pose_out->pose + i,
			posePrev0->pose + i, pose00->pose + i, pose01->pose + i, poseNext0->pose + i,
			posePrev1->pose + i, pose10->pose + i, pose11->pose + i, poseNext1->pose + i,
			posePrev2->pose + i, pose20->pose + i, pose21->pose + i, poseNext2->pose + i,
			posePrev3->pose + i, pose30->pose + i, pose31->pose + i, poseNext3->pose + i,
			u0, u1, u2, u3, u);
	}
	return pose_out;
}


//-----------------------------------------------------------------------------


#endif	// !__ANIMAL3D_HIERARCHYSTATEBLEND_INL
#endif	// __ANIMAL3D_HIERARCHYSTATEBLEND_H