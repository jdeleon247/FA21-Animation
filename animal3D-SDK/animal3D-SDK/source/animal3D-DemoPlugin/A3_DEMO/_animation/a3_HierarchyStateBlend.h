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
	
	a3_HierarchyStateBlend.h
	Hierarchy blend operations.
*/

#ifndef __ANIMAL3D_HIERARCHYSTATEBLEND_H
#define __ANIMAL3D_HIERARCHYSTATEBLEND_H


#include "a3_HierarchyState.h"

#include "a3_Kinematics.h"


#ifdef __cplusplus
extern "C"
{
#else	// !__cplusplus

#endif	// __cplusplus


// ROBUST BLEND NODE for any op
typedef a3_SpatialPose* (*a3_SpatialPoseBlendOp)(
	a3_SpatialPose* pose_out,
	a3_SpatialPose const* pose_ctrl[],
	a3real const param[]
	);

typedef struct a3_SpatialPoseBlendNode
{
	a3_SpatialPoseBlendOp op;
	a3_SpatialPose* pose_out;
	a3_SpatialPose const* pose_ctrl[32];
	a3real const* param[8];
} a3_SpatialPoseBlendNode;

inline a3_SpatialPoseBlendNode* a3spatialPoseBlendNodeCall(
	a3_SpatialPoseBlendNode* b)
{
	b->op(b->pose_out, b->pose_ctrl, *b->param);
	return b;
}

// e.g. lerp
inline a3_SpatialPose* a3_SpatialPoseBlendOpLerp(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[2], a3real const* param[1])// <- [x] doesn't matter, but it's more readable 
{
	// p0 + (p1 - p0)*u
	a3_SpatialPose const* p0 = pose_ctrl[0];
	a3_SpatialPose const* p1 = pose_ctrl[1];
	a3real const* u = param[0];
	a3spatialPoseLerp(pose_out, p0, p1, *u);

	return pose_out;
}


// Do same for clipController ?

//-----------------------------------------------------------------------------

// Fundamental Blend Operations
// (Pointer-based)

// reset/identity operation for single spatial pose
a3_SpatialPose* a3spatialPoseOpIdentity(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[1], a3real const* param[1]);
//a3_SpatialPose* a3spatialPoseOpIdentity(a3_SpatialPose* pose_out);

// Equivalent to a constructor, this operation returns/sets a pose constructed using the components provided.
//a3_SpatialPose* a3spatialPoseOpConstruct(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[2], a3real const* param[1]);
a3_SpatialPose* a3spatialPoseOpConstruct(a3_SpatialPose* pose_out, a3vec4 rotation, a3vec4 scale, a3vec4 translation);

// returns/sets the unchanged control pose
a3_SpatialPose* a3spatialPoseOpCopy(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[1], a3real const* param[1]);
//a3_SpatialPose* a3spatialPoseOpCopy(a3_SpatialPose* pose_out, a3_SpatialPose* pose_in);

// calculates the opposite/inverse pose description that "undoes" the control pose.
a3_SpatialPose* a3spatialPoseOpInvert(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[1], a3real const* param[1]);
//a3_SpatialPose* a3spatialPoseOpInvert(a3_SpatialPose* pose_out, a3_SpatialPose* pose_in);

// piecewise concatenation of each part of a spatialPose
a3_SpatialPose* a3spatialPoseOpConcat(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[2], a3real const* param[1]);
//a3_SpatialPose* a3spatialPoseOpConcat(a3_SpatialPose* pose_out, a3_SpatialPose* pose_lh, a3_SpatialPose* pose_rh);

// selects one of the two control poses using nearest interpolation.
a3_SpatialPose* a3spatialPoseOpNearest(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[2], a3real const* param[1]);
//a3_SpatialPose* a3spatialPoseOpNearest(a3_SpatialPose* pose_out, a3_SpatialPose const* pose0, a3_SpatialPose const* pose1, a3real const u);

// LERP operation for single spatial pose
a3_SpatialPose* a3spatialPoseOpLERP(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[2], a3real const* param[1]);
//a3_SpatialPose* a3spatialPoseOpLERP(a3_SpatialPose* pose_out, a3_SpatialPose const* pose0, a3_SpatialPose const* pose1, a3real const u);

// Cubic interpolation for poses
a3_SpatialPose* a3spatialPoseOpCubic(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[4], a3real const* param[1]);
//a3_SpatialPose* a3spatialPoseOpCubic(a3_SpatialPose* pose_out, a3_SpatialPose const* posePrev, a3_SpatialPose const* pose0,
//	a3_SpatialPose const* pose1, a3_SpatialPose const* poseNext, a3real const u);


//-----------------------------------------------------------------------------

// Derivative Blend Operations
// (Pointer-based)

// calculates the "difference" or "split" between the two control poses.
a3_SpatialPose* a3spatialPoseOpSplit(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[2], a3real const* param[1]);
//a3_SpatialPose* a3spatialPoseOpSplit(a3_SpatialPose* pose_out, a3_SpatialPose* pose_lh, a3_SpatialPose* pose_rh);

// calculates the "scaled" pose, which is some blend between the identity pose and the control pose.
a3_SpatialPose* a3spatialPoseOpScale(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[1], a3real const* param[1]);
//a3_SpatialPose* a3spatialPoseOpScale(a3_SpatialPose* pose_out, a3_SpatialPose const* pose, a3real const u);


// triangular interpolation for poses.
a3_SpatialPose* a3spatialPoseOpTriangular(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[3], a3real const* param[2]);
//a3_SpatialPose* a3spatialPoseOpTriangular(a3_SpatialPose* pose_out, a3_SpatialPose const* pose0, a3_SpatialPose const* pose1, a3_SpatialPose const* pose2, a3real const u0, a3real const u1);

// bilinear nearest function for poses.
a3_SpatialPose* a3spatialPoseOpBiNearest(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[4], a3real const* param[3]);
//a3_SpatialPose* a3spatialPoseOpBiNearest(a3_SpatialPose* pose_out, a3_SpatialPose const* pose00, a3_SpatialPose const* pose01,
//																   a3_SpatialPose const* pose10, a3_SpatialPose const* pose11, a3real const u0, a3real const u1, a3real const u);
// bilinear interpolation function for poses.
a3_SpatialPose* a3spatialPoseOpBiLerp(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[4], a3real const* param[3]);
//a3_SpatialPose* a3spatialPoseOpBiLerp(a3_SpatialPose* pose_out, a3_SpatialPose const* pose00, a3_SpatialPose const* pose01,
//																  a3_SpatialPose const* pose10, a3_SpatialPose const* pose11, a3real const u0, a3real const u1, a3real const u);
// bicubic interpolation algorithm for poses.
a3_SpatialPose* a3spatialPoseOpBiCubic(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[16], a3real const* param[5]);
//a3_SpatialPose* a3spatialPoseOpBiCubic(a3_SpatialPose* pose_out, a3_SpatialPose const* posePrev0, a3_SpatialPose const* pose00, a3_SpatialPose const* pose01, a3_SpatialPose const* poseNext0,
//																 a3_SpatialPose const* posePrev1, a3_SpatialPose const* pose10, a3_SpatialPose const* pose11, a3_SpatialPose const* poseNext1, 
//																 a3_SpatialPose const* posePrev2, a3_SpatialPose const* pose20, a3_SpatialPose const* pose21, a3_SpatialPose const* poseNext2,
//																 a3_SpatialPose const* posePrev3, a3_SpatialPose const* pose30, a3_SpatialPose const* pose31, a3_SpatialPose const* poseNext3,
//																 a3real const u0, a3real const u1, a3real const u2, a3real const u3, a3real const u);


//-----------------------------------------------------------------------------

// data-based reset/identity
a3_SpatialPose a3spatialPoseDOpIdentity();

// data-based LERP
a3_SpatialPose a3spatialPoseDOpLERP(a3_SpatialPose const pose0, a3_SpatialPose const pose1, a3real const u);


//-----------------------------------------------------------------------------

// Fundamental Blend Operations
// (Pointer-based)

// reset/identity operation for single spatial pose
a3_HierarchyPose* a3hierarchyPoseOpIdentity(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[1], a3real const* param[1], a3ui32 numNodes);
//a3_HierarchyPose* a3hierarchyPoseOpIdentity(a3_HierarchyPose* pose_out, a3ui32 numNodes);

// Equivalent to a constructor, this operation returns/sets a pose constructed using the components provided.
//a3_HierarchyPose* a3hierarchyPoseOpConstruct(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[1], a3real const* param[1], a3ui32 numNodes);
a3_HierarchyPose* a3hierarchyPoseOpConstruct(a3_HierarchyPose* pose_out, a3vec4 rotation, a3vec4 scale, a3vec4 translation, a3ui32 numNodes);

// returns/sets the unchanged control pose
a3_HierarchyPose* a3hierarchyPoseOpCopy(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[1], a3real const* param[1], a3ui32 numNodes);
//a3_HierarchyPose* a3hierarchyPoseOpCopy(a3_HierarchyPose* pose_out, a3_HierarchyPose* pose_in, a3ui32 numNodes);

// calculates the opposite/inverse pose description that "undoes" the control pose.
a3_HierarchyPose* a3hierarchyPoseOpInvert(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[1], a3real const* param[1], a3ui32 numNodes);
//a3_HierarchyPose* a3hierarchyPoseOpInvert(a3_HierarchyPose* pose_out, a3_HierarchyPose* pose_in, a3ui32 numNodes);

// piecewise concatenation of each part of a spatialPose
a3_HierarchyPose* a3hierarchyPoseOpConcat(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[2], a3real const* param[1], a3ui32 numNodes);
//a3_HierarchyPose* a3hierarchyPoseOpConcat(a3_HierarchyPose* pose_out, a3_HierarchyPose* pose_lh, a3_HierarchyPose* pose_rh, a3ui32 numNodes);

// selects one of the two control poses using nearest interpolation.
a3_HierarchyPose* a3hierarchyPoseOpNearest(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[2], a3real const* param[1], a3ui32 numNodes);
//a3_HierarchyPose* a3hierarchyPoseOpNearest(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose0, a3_HierarchyPose const* pose1, a3real const u, a3ui32 numNodes);

// LERP operation for single spatial pose
a3_HierarchyPose* a3hierarchyPoseOpLERP(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[2], a3real const* param[1], a3ui32 numNodes);
//a3_HierarchyPose* a3hierarchyPoseOpLERP(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose0, a3_HierarchyPose const* pose1, a3real const u, a3ui32 numNodes);

// Cubic interpolation for poses
a3_HierarchyPose* a3hierarchyPoseOpCubic(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[4], a3real const* param[1], a3ui32 numNodes);
//a3_HierarchyPose* a3hierarchyPoseOpCubic(a3_HierarchyPose* pose_out, a3_HierarchyPose const* posePrev, a3_HierarchyPose const* pose0,
//	a3_HierarchyPose const* pose1, a3_HierarchyPose const* poseNext, a3real const u, a3ui32 numNodes);


//-----------------------------------------------------------------------------

// Derivative Blend Operations
// (Pointer-based)

// calculates the "difference" or "split" between the two control poses.
a3_HierarchyPose* a3hierarchyPoseOpSplit(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[2], a3real const* param[1], a3ui32 numNodes);
//a3_HierarchyPose* a3hierarchyPoseOpSplit(a3_HierarchyPose* pose_out, a3_HierarchyPose* pose_lh, a3_HierarchyPose* pose_rh, a3ui32 numNodes);

// calculates the "scaled" pose, which is some blend between the identity pose and the control pose.
a3_HierarchyPose* a3hierarchyPoseOpScale(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[1], a3real const* param[1], a3ui32 numNodes);
//a3_HierarchyPose* a3hierarchyPoseOpScale(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose, a3real const u, a3ui32 numNodes);

// triangular interpolation for poses.
a3_HierarchyPose* a3hierarchyPoseOpTriangular(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[3], a3real const* param[2], a3ui32 numNodes);
//a3_HierarchyPose* a3hierarchyPoseOpTriangular(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose0, a3_HierarchyPose const* pose1, a3_HierarchyPose const* pose2, a3real const u0, a3real const u1, a3ui32 numNodes);

// bilinear nearest function for poses.
a3_HierarchyPose* a3hierarchyPoseOpBiNearest(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[4], a3real const* param[3], a3ui32 numNodes);
//a3_HierarchyPose* a3hierarchyPoseOpBiNearest(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose00, a3_HierarchyPose const* pose01,
//	a3_HierarchyPose const* pose10, a3_HierarchyPose const* pose11, a3real const u0, a3real const u1, a3real const u, a3ui32 numNodes);
// bilinear interpolation function for poses.
a3_HierarchyPose* a3hierarchyPoseOpBiLerp(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[4], a3real const* param[3], a3ui32 numNodes);
//a3_HierarchyPose* a3hierarchyPoseOpBiLerp(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose00, a3_HierarchyPose const* pose01,
//	a3_HierarchyPose const* pose10, a3_HierarchyPose const* pose11, a3real const u0, a3real const u1, a3real const u, a3ui32 numNodes);
// bicubic interpolation algorithm for poses.
a3_HierarchyPose* a3hierarchyPoseOpIdentity(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[1], a3real const* param[1], a3ui32 numNodes);
//a3_HierarchyPose* a3hierarchyPoseOpBiCubic(a3_HierarchyPose* pose_out, a3_HierarchyPose const* posePrev0, a3_HierarchyPose const* pose00, a3_HierarchyPose const* pose01, a3_HierarchyPose const* poseNext0,
//	a3_HierarchyPose const* posePrev1, a3_HierarchyPose const* pose10, a3_HierarchyPose const* pose11, a3_HierarchyPose const* poseNext1,
//	a3_HierarchyPose const* posePrev2, a3_HierarchyPose const* pose20, a3_HierarchyPose const* pose21, a3_HierarchyPose const* poseNext2,
//	a3_HierarchyPose const* posePrev3, a3_HierarchyPose const* pose30, a3_HierarchyPose const* pose31, a3_HierarchyPose const* poseNext3,
//	a3real const u0, a3real const u1, a3real const u2, a3real const u3, a3real const u, a3ui32 numNodes);


//-----------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif	// __cplusplus


#include "_inl/a3_HierarchyStateBlend.inl"


#endif	// !__ANIMAL3D_HIERARCHYSTATEBLEND_H