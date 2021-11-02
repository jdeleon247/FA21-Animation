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
inline a3_SpatialPose* a3spatialPoseOpIdentity(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[1], a3real const* param[1])
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
inline a3_SpatialPose* a3spatialPoseOpCopy(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[1], a3real const* param[1])
{
	pose_out->transform = pose_ctrl[0]->transform;
	pose_out->angles = pose_ctrl[0]->angles;
	pose_out->orientation = pose_ctrl[0]->orientation;
	pose_out->scale = pose_ctrl[0]->scale;
	pose_out->translation = pose_ctrl[0]->translation;

	return pose_out;
}

// calculates the opposite/inverse pose description that "undoes" the control pose.
inline a3_SpatialPose* a3spatialPoseOpInvert(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[1], a3real const* param[1])
{
	a3quatGetInverse(pose_out->orientation.v, pose_ctrl[0]->orientation.v);
	a3real4GetNegative(pose_out->angles.v, pose_ctrl[0]->angles.v); // Additive -> make negative
	a3real4DivComp(pose_out->scale.v, pose_ctrl[0]->scale.v); // Multiplicatve -> (?) divide (?)
	a3real4GetNegative(pose_out->translation.v, pose_ctrl[0]->translation.v);  // Additive -> make negative
	
	return pose_out;
}

// piecewise concatenation of each part of a spatialPose
inline a3_SpatialPose* a3spatialPoseOpConcat(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[2], a3real const* param[1])
{
	a3spatialPoseConcat(pose_out, pose_ctrl[0], pose_ctrl[1]);
	return pose_out;
}

// selects one of the two control poses using nearest interpolation.
inline a3_SpatialPose* a3spatialPoseOpNearest(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[2], a3real const* param[1])
{
	if (*param[0] <= 0.5)
	{
		a3spatialPoseCopy(pose_out, pose_ctrl[0]);
	}
	else
	{
		a3spatialPoseCopy(pose_out, pose_ctrl[1]);
	}
	return pose_out;
}

// LERP operation for single spatial pose
inline a3_SpatialPose* a3spatialPoseOpLERP(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[2], a3real const* param[1])
{
	a3real4Slerp(pose_out->orientation.v, pose_ctrl[0]->orientation.v, pose_ctrl[1]->orientation.v, *param[0]);
	a3real4Lerp(pose_out->angles.v, pose_ctrl[0]->angles.v, pose_ctrl[1]->angles.v, *param[0]);
	a3real4Lerp(pose_out->scale.v, pose_ctrl[0]->scale.v, pose_ctrl[1]->scale.v, *param[0]);
	a3real4Lerp(pose_out->translation.v, pose_ctrl[0]->translation.v, pose_ctrl[1]->translation.v, *param[0]);
	
	return pose_out;
}

// Cubic interpolation for poses
inline a3_SpatialPose* a3spatialPoseOpCubic(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[4], a3real const* param[1])
{
	a3real4CatmullRom(pose_out->orientation.v, pose_ctrl[0]->orientation.v, pose_ctrl[1]->orientation.v, pose_ctrl[2]->orientation.v, pose_ctrl[3]->orientation.v, *param[0]); // Maybe not right
	a3real4CatmullRom(pose_out->angles.v, pose_ctrl[0]->angles.v, pose_ctrl[1]->angles.v, pose_ctrl[2]->angles.v, pose_ctrl[3]->angles.v, *param[0]);
	a3real4CatmullRom(pose_out->scale.v, pose_ctrl[0]->scale.v, pose_ctrl[1]->scale.v, pose_ctrl[2]->scale.v, pose_ctrl[3]->scale.v, *param[0]);
	a3real4CatmullRom(pose_out->translation.v, pose_ctrl[0]->translation.v, pose_ctrl[1]->translation.v, pose_ctrl[2]->translation.v, pose_ctrl[3]->translation.v, *param[0]);
	return pose_out;
}


//-----------------------------------------------------------------------------

// Derivative Blend Operations
// (Pointer-based)

// calculates the "difference" or "split" between the two control poses.
inline a3_SpatialPose* a3spatialPoseOpSplit(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[2], a3real const* param[1])
{
	pose_out->angles.x = a3trigValid_sind(pose_ctrl[0]->angles.x - pose_ctrl[1]->angles.x);
	pose_out->angles.y = a3trigValid_sind(pose_ctrl[0]->angles.y - pose_ctrl[1]->angles.y);
	pose_out->angles.z = a3trigValid_sind(pose_ctrl[0]->angles.z - pose_ctrl[1]->angles.z);

	pose_out->scale.x = pose_ctrl[0]->scale.x / pose_ctrl[1]->scale.x;
	pose_out->scale.y = pose_ctrl[0]->scale.y / pose_ctrl[1]->scale.y;
	pose_out->scale.z = pose_ctrl[0]->scale.z / pose_ctrl[1]->scale.z;

	pose_out->translation.x = pose_ctrl[0]->translation.x - pose_ctrl[1]->translation.x;
	pose_out->translation.y = pose_ctrl[0]->translation.y - pose_ctrl[1]->translation.y;
	pose_out->translation.z = pose_ctrl[0]->translation.z - pose_ctrl[1]->translation.z;

	return pose_out;
}

// calculates the "scaled" pose, which is some blend between the identity pose and the control pose.
inline a3_SpatialPose* a3spatialPoseOpScale(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[1], a3real const* param[1])
{
	a3_SpatialPose const* lerpControls[2];
	lerpControls[0] = a3spatialPoseOpIdentity(pose_out, pose_ctrl, param);
	lerpControls[1] = pose_ctrl[0];
	a3spatialPoseOpLERP(pose_out, lerpControls, param);
	return pose_out;
}

// triangular interpolation for poses.
inline a3_SpatialPose* a3spatialPoseOpTriangular(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[3], a3real const* param[2])
{
	a3_SpatialPose const* concatControls1[2];
	a3_SpatialPose const* concatControls2[2];

	a3real paramU = 1 - *param[0] - *param[1];
	const a3real* u[1];
	u[0] = &paramU;

	concatControls1[0] = a3spatialPoseOpScale(pose_out, &pose_ctrl[0], u);
	concatControls1[1] = a3spatialPoseOpScale(pose_out, &pose_ctrl[1], &param[0]);

	concatControls2[0] = a3spatialPoseOpConcat(pose_out, concatControls1, u);
	concatControls2[1] = a3spatialPoseOpScale(pose_out, &pose_ctrl[2], &param[1]);

	pose_out = a3spatialPoseOpConcat(pose_out, concatControls2, u);
	
	//pose_out = a3spatialPoseOpConcat(pose_out,a3spatialPoseOpConcat(pose_out,a3spatialPoseOpScale(pose_out, scaleControls1, &u), a3spatialPoseOpScale(pose_out, scaleControls2, param[0])), a3spatialPoseOpScale(pose_out, scaleControls3, param[1]));
	return pose_out;
}

// bilinear nearest function for poses.
inline a3_SpatialPose* a3spatialPoseOpBiNearest(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[4], a3real const* param[3])
{
	a3_SpatialPose* tmpPoses[2];
	a3spatialPoseOpNearest(tmpPoses[0], &pose_ctrl[0], &param[0]);
	a3spatialPoseOpNearest(tmpPoses[0], &pose_ctrl[2], &param[1]);

	a3spatialPoseOpNearest(pose_out, tmpPoses, &param[2]);
	return pose_out;
}

// bilinear interpolation function for poses.
inline a3_SpatialPose* a3spatialPoseOpBiLerp(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[4], a3real const* param[3])
{
	a3_SpatialPose tmpPose0[1];
	a3_SpatialPose tmpPose1[1];
	a3spatialPoseLerp(tmpPose0, pose_ctrl[0], pose_ctrl[1], *param[0]);
	a3spatialPoseLerp(tmpPose1, pose_ctrl[2], pose_ctrl[3], *param[1]);

	a3spatialPoseLerp(pose_out, tmpPose0, tmpPose1, *param[2]);
	return pose_out;
}

// bicubic interpolation algorithm for poses.
inline a3_SpatialPose* a3spatialPoseOpBiCubic(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[16], a3real const* param[5])
{
	a3_SpatialPose* tmpPoses[4];

	a3spatialPoseOpCubic(tmpPoses[0], &pose_ctrl[0], &param[0]);
	a3spatialPoseOpCubic(tmpPoses[1], &pose_ctrl[4], &param[1]);
	a3spatialPoseOpCubic(tmpPoses[2], &pose_ctrl[8], &param[2]);
	a3spatialPoseOpCubic(tmpPoses[3], &pose_ctrl[12], &param[3]);

	a3spatialPoseOpCubic(pose_out, tmpPoses, &param[4]);

	return pose_out;
}

//-----------------------------------------------------------------------------

// Additional Blend Operations
// (Pointer-based)

// Hermite interpolation for poses
inline a3_SpatialPose* a3spatialPoseOpSmoothstep(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[2], a3real const* param[1])
{
	a3real uParam = 3 * *param[0] * *param[0] - 2 * *param[0] * *param[0] * *param[0];
	a3real const* u = &uParam;
	

	a3spatialPoseOpLERP(pose_out, &pose_ctrl[2], &u);
	
	return pose_out;
}

// calculates the "descaled" pose, which is some blend between the identity pose and the inverted control pose.
inline a3_SpatialPose* a3spatialPoseOpDescale(a3_SpatialPose* pose_out, a3_SpatialPose const* pose_ctrl[1], a3real const* param[1])
{
	const a3_SpatialPose* tmpPoses[2];
	tmpPoses[0] = a3spatialPoseOpIdentity(pose_out, &pose_ctrl[1], &param[1]);
	tmpPoses[1] = a3spatialPoseOpInvert(pose_out, &pose_ctrl[1], &param[1]);
	a3spatialPoseOpLERP(pose_out, &tmpPoses[2], &param[1]);
	return pose_out;
}

// performs the "convert" step for a spatial/hierarchical pose (convert raw components into transforms)
inline a3_SpatialPose* a3spatialPoseOpConvert(a3_SpatialPose* pose_out)
{
	a3spatialPoseConvert(pose_out, a3poseChannel_none, a3poseEulerOrder_xyz);
	return pose_out;
}

// performs the opposite of convert (restore raw components from transforms)
inline a3_SpatialPose* a3spatialPoseOpRevert(a3_SpatialPose* pose_out)
{
	a3spatialPoseRestore(pose_out, a3poseChannel_none, a3poseEulerOrder_xyz);
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
inline a3_HierarchyPose* a3hierarchyPoseOpIdentity(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[1], a3real const* param[1], a3ui32 numNodes)
{
	a3_SpatialPose* dummy[1];
	for (a3index i = 0; i < numNodes; ++i)
	{
		a3spatialPoseOpIdentity(pose_out->pose + i, dummy, param);
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
inline a3_HierarchyPose* a3hierarchyPoseOpCopy(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[1], a3real const* param[1], a3ui32 numNodes)
{
	for (a3index i = 0; i < numNodes; ++i)
	{
		a3_SpatialPose* pose0 = pose_ctrl[0]->pose + i;
		a3spatialPoseOpCopy(pose_out->pose + i, &pose0, param);
	}
	return pose_out;
}

// calculates the opposite/inverse pose description that "undoes" the control pose.
inline a3_HierarchyPose* a3hierarchyPoseOpInvert(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[1], a3real const* param[1], a3ui32 numNodes)
{
	for (a3index i = 0; i < numNodes; ++i)
	{
		a3_SpatialPose* pose0 = pose_ctrl[0]->pose + i;
		a3spatialPoseOpInvert(pose_out->pose + i, &pose0, param);
	}
	return pose_out;
}

// piecewise concatenation of each part of a spatialPose
inline a3_HierarchyPose* a3hierarchyPoseOpConcat(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[2], a3real const* param[1], a3ui32 numNodes)
{
	a3_SpatialPose* poses[2];
	for (a3index i = 0; i < numNodes; ++i)
	{
		poses[0] = pose_ctrl[0]->pose + i;
		poses[1] = pose_ctrl[1]->pose + i;
		a3spatialPoseOpConcat(pose_out->pose + i, poses, param);
	}
	return pose_out;
}

// selects one of the two control poses using nearest interpolation.
inline a3_HierarchyPose* a3hierarchyPoseOpNearest(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[2], a3real const* param[1], a3ui32 numNodes)
{
	a3_SpatialPose* poses[2];
	for (a3index i = 0; i < numNodes; ++i)
	{
		poses[0] = pose_ctrl[0]->pose + i;
		poses[1] = pose_ctrl[1]->pose + i;
		a3spatialPoseOpNearest(pose_out->pose + i, poses, param);
	}
	return pose_out;
}

// LERP operation for single spatial pose
inline a3_HierarchyPose* a3hierarchyPoseOpLERP(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[2], a3real const* param[1], a3ui32 numNodes)
{
	a3_SpatialPose* poses[2];
	for (a3index i = 0; i < numNodes; ++i)
	{
		poses[0] = pose_ctrl[0]->pose + i;
		poses[1] = pose_ctrl[1]->pose + i;
		a3spatialPoseOpLERP(pose_out->pose + i, poses, param);
	}
	return pose_out;
}

// Cubic interpolation for poses
inline a3_HierarchyPose* a3hierarchyPoseOpCubic(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[4], a3real const* param[1], a3ui32 numNodes)
{
	a3_SpatialPose* poses[4];
	for (a3index i = 0; i < numNodes; ++i)
	{
		poses[0] = pose_ctrl[0]->pose + i;
		poses[1] = pose_ctrl[1]->pose + i;
		poses[2] = pose_ctrl[2]->pose + i;
		poses[3] = pose_ctrl[3]->pose + i;
		a3spatialPoseOpCubic(pose_out->pose + i, poses, param);
	}
	return pose_out;
}


//-----------------------------------------------------------------------------

// Derivative Blend Operations
// (Pointer-based)

// calculates the "difference" or "split" between the two control poses.
inline a3_HierarchyPose* a3hierarchyPoseOpSplit(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[2], a3real const* param[1], a3ui32 numNodes)
{
	a3_SpatialPose* poses[2];
	for (a3index i = 0; i < numNodes; ++i)
	{
		poses[0] = pose_ctrl[0]->pose + i;
		poses[1] = pose_ctrl[1]->pose + i;
		a3spatialPoseOpSplit(pose_out->pose + i, poses, param);
	}
	return pose_out;
}

// calculates the "scaled" pose, which is some blend between the identity pose and the control pose.
inline a3_HierarchyPose* a3hierarchyPoseOpScale(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[1], a3real const* param[1], a3ui32 numNodes)
{
	for (a3index i = 0; i < numNodes; ++i)
	{
		a3_SpatialPose* pose0 = pose_ctrl[0]->pose + i;
		a3spatialPoseOpScale(pose_out->pose + i, &pose0, param);
	}
	return pose_out;
}

// triangular interpolation for poses.
inline a3_HierarchyPose* a3hierarchyPoseOpTriangular(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[3], a3real const* param[2], a3ui32 numNodes)
{
	a3_SpatialPose* poses[3];
	for (a3index i = 0; i < numNodes; ++i)
	{
		poses[0] = pose_ctrl[0]->pose + i;
		poses[1] = pose_ctrl[1]->pose + i;
		poses[2] = pose_ctrl[2]->pose + i;
		a3spatialPoseOpTriangular(pose_out->pose + i, poses, param);
	}
	return pose_out;
}

// bilinear nearest function for poses.
inline a3_HierarchyPose* a3hierarchyPoseOpBiNearest(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[4], a3real const* param[3], a3ui32 numNodes)
{
	a3_SpatialPose* poses[4];
	for (a3index i = 0; i < numNodes; ++i)
	{
		poses[0] = pose_ctrl[0]->pose + i;
		poses[1] = pose_ctrl[1]->pose + i;
		poses[2] = pose_ctrl[2]->pose + i;
		poses[3] = pose_ctrl[3]->pose + i;
		a3spatialPoseOpBiNearest(pose_out->pose + i, poses, param);
	}
	return pose_out;
}

// bilinear interpolation function for poses.
inline a3_HierarchyPose* a3hierarchyPoseOpBiLerp(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[4], a3real const* param[3], a3ui32 numNodes)
{
	a3_SpatialPose* poses[4];
	for (a3index i = 0; i < numNodes; ++i)
	{
		poses[0] = pose_ctrl[0]->pose + i;
		poses[1] = pose_ctrl[1]->pose + i;
		poses[2] = pose_ctrl[2]->pose + i;
		poses[3] = pose_ctrl[3]->pose + i;
		a3spatialPoseOpBiLerp(pose_out->pose + i, poses, param);
	}
	return pose_out;
}

// bicubic interpolation algorithm for poses.
inline a3_HierarchyPose* a3hierarchyPoseOpBiCubic(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[16], a3real const* param[5], a3ui32 numNodes)
{
	a3_SpatialPose* poses[16];
	for (a3index i = 0; i < numNodes; ++i)
	{
		poses[0] = pose_ctrl[0]->pose + i;
		poses[1] = pose_ctrl[1]->pose + i;
		poses[2] = pose_ctrl[2]->pose + i;
		poses[3] = pose_ctrl[3]->pose + i;
		poses[4] = pose_ctrl[4]->pose + i;
		poses[5] = pose_ctrl[5]->pose + i;
		poses[6] = pose_ctrl[6]->pose + i;
		poses[7] = pose_ctrl[7]->pose + i;
		poses[8] = pose_ctrl[8]->pose + i;
		poses[9] = pose_ctrl[9]->pose + i;
		poses[10] = pose_ctrl[10]->pose + i;
		poses[11] = pose_ctrl[11]->pose + i;
		poses[12] = pose_ctrl[12]->pose + i;
		poses[13] = pose_ctrl[13]->pose + i;
		poses[14] = pose_ctrl[14]->pose + i;
		poses[15] = pose_ctrl[15]->pose + i;
		a3spatialPoseOpBiCubic(pose_out->pose + i, poses, param);
	}
	return pose_out;
}

// Hermite interpolation for poses
inline a3_HierarchyPose* a3hierarchyPoseOpSmoothstep(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[2], a3real const* param[1], a3ui32 numNodes)
{
	a3_SpatialPose* poses[2];
	for (a3index i = 0; i < numNodes; ++i)
	{
		poses[0] = pose_ctrl[0]->pose + i;
		poses[1] = pose_ctrl[1]->pose + i;
		a3spatialPoseOpSmoothstep(pose_out->pose + i, poses, param);
	}	return pose_out;
}
// calculates the "descaled" pose, which is some blend between the identity pose and the inverted control pose.
inline a3_HierarchyPose* a3hierarchyPoseOpDescale(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[1], a3real const* param[1], a3ui32 numNodes)
{
	for (a3index i = 0; i < numNodes; ++i)
	{
		a3_SpatialPose* pose0 = pose_ctrl[0]->pose + i;
		a3spatialPoseOpDescale(pose_out->pose + i, &pose0, param);
	}
	return pose_out;
}

// performs the "convert" step for a spatial/hierarchical pose (convert raw components into transforms)
inline a3_HierarchyPose* a3hierarchyPoseOpConvert(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[1], a3ui32 numNodes)
{
	for (a3index i = 0; i < numNodes; ++i)
	{
		a3spatialPoseOpConvert(pose_out->pose);
	}
	return pose_out;
}

// performs the opposite of convert (restore raw components from transforms)
inline a3_HierarchyPose* a3hierarchyPoseOpRevert(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose_ctrl[1], a3ui32 numNodes)
{
	for (a3index i = 0; i < numNodes; ++i)
	{
		a3spatialPoseOpRevert(pose_out->pose);
	}
	return pose_out;
}


// performs the fundamental forward kinematics operation, converting the provided local-space transform into the target object-space transform
inline a3_HierarchyPose* a3hierarchyPoseOpFK(a3_HierarchyPose* pose_out, a3_Hierarchy const* hierarchy, a3_HierarchyPose const* poseCtrl[2])
{
	const a3_HierarchyNode* itr = hierarchy->nodes;
	const a3_HierarchyNode* const end = itr + hierarchy->numNodes;
	for (; itr < end; ++itr)
	{
		if (itr->parentIndex >= 0)
			a3real4x4Product(poseCtrl[0]->pose[itr->index].transform.m,
				poseCtrl[0]->pose[itr->parentIndex].transform.m,
				poseCtrl[1]->pose[itr->index].transform.m);
		else
			poseCtrl[0]->pose[itr->index] = poseCtrl[1]->pose[itr->index];
	}
	pose_out->pose = poseCtrl[0]->pose;
	return pose_out;
}

//performs the fundamental inverse kinematics operation, converting the provided object - space transform into the target local - space transform
inline a3_HierarchyPose* a3hierarchyPoseOpIK(a3_HierarchyPose* pose_out, a3_Hierarchy const* hierarchy, a3_HierarchyPose const* poseCtrl[2])
{
	const a3_HierarchyNode* itr = hierarchy->nodes;
	const a3_HierarchyNode* const end = itr + hierarchy->numNodes;
	const a3real* param0 = 0;
	a3hierarchyPoseOpInvert(pose_out, poseCtrl, &param0, hierarchy->numNodes);

	for (; itr < end; ++itr)
	{
		if (itr->parentIndex >= 0)
			a3real4x4Product(poseCtrl[1]->pose[itr->index].transform.m,
				pose_out->pose[itr->parentIndex].transform.m,
				poseCtrl[0]->pose[itr->index].transform.m);
		else
			poseCtrl[1]->pose[itr->index] = poseCtrl[0]->pose[itr->index];
	}
	pose_out->pose = poseCtrl[1]->pose;

	return pose_out;
}


inline a3_HierarchyPose* a3hierarchyPoseClipOpLerp(a3_HierarchyPose* pose_out, a3_Clip* clip0, a3_Clip* clip1, a3real const* param[3])
{
	a3ui32 currentKeyframe = clip0->first_keyframe;
	a3f32 clipTime = clip0->duration * *param[0];
	while (clipTime > 0)
	{
		clipTime -= clip0->framePool->keyframe[currentKeyframe].duration;
		currentKeyframe++;
	}
	return pose_out;
}

inline a3_HierarchyPose* a3hierarchyPoseClipOpAdd(a3_HierarchyPose* pose_out, a3_Clip* clip0, a3_Clip* clip1, a3real const* param[3])
{
	return pose_out;
}

inline a3_HierarchyPose* a3hierarchyPoseClipOpScale(a3_HierarchyPose* pose_out, a3_Clip* clip0, a3real const* param[3])
{
	return pose_out;
}

inline a3_HierarchyPose* a3hierarchyPoseClipCtrlOpLerp(a3_HierarchyPose* pose_out, a3_ClipController* clipCtrl0, a3_ClipController* clipCtrl1, a3real const* param[1], a3ui32 numNodes)
{
	a3_Sample pose0;
	a3_Sample pose1;
	a3sampleInit(&pose0, numNodes);
	a3sampleInit(&pose1, numNodes);

	a3clipControllerEvaluate(clipCtrl0, &pose0);
	a3clipControllerEvaluate(clipCtrl1, &pose1);

	a3_HierarchyPose const* controlPoses[2];
	controlPoses[0] = pose0.pose;
	controlPoses[1] = pose1.pose;
	a3hierarchyPoseOpLERP(pose_out, controlPoses, param, numNodes);

	return pose_out;
}

inline a3_HierarchyPose* a3hierarchyPoseClipCtrlOpAdd(a3_HierarchyPose* pose_out, a3_ClipController* clipCtrl0, a3_ClipController* clipCtrl1, a3real const* param[1], a3ui32 numNodes)
{
	a3_Sample pose0;
	a3_Sample pose1;
	a3sampleInit(&pose0, numNodes);
	a3sampleInit(&pose1, numNodes);

	a3clipControllerEvaluate(clipCtrl0, &pose0);
	a3clipControllerEvaluate(clipCtrl1, &pose1);

	a3_HierarchyPose const* controlPoses[2];
	controlPoses[0] = pose0.pose;
	controlPoses[1] = pose1.pose;
	a3hierarchyPoseOpConcat(pose_out, controlPoses, param, numNodes);

	return pose_out;
}

inline a3_HierarchyPose* a3hierarchyPoseClipCtrlOpScale(a3_HierarchyPose* pose_out, a3_ClipController* clipCtrl0, a3_ClipController* clipCtrl1, a3real const* param[1], a3ui32 numNodes)
{
	a3_Sample pose0;
	a3sampleInit(&pose0, numNodes);

	a3clipControllerEvaluate(clipCtrl0, &pose0);

	a3_HierarchyPose const* controlPoses[1];
	controlPoses[0] = pose0.pose;

	a3hierarchyPoseOpScale(pose_out, controlPoses, param, numNodes);

	return pose_out;
}

//-----------------------------------------------------------------------------


#endif	// !__ANIMAL3D_HIERARCHYSTATEBLEND_INL
#endif	// __ANIMAL3D_HIERARCHYSTATEBLEND_H