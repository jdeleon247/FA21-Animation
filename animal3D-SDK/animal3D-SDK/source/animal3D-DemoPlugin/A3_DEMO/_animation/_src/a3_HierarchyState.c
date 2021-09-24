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

	a3_HierarchyState.c
	Implementation of transform hierarchy state.
*/

#include "../a3_HierarchyState.h"

#include <stdlib.h>
#include <string.h>


//-----------------------------------------------------------------------------

// initialize pose set given an initialized hierarchy and key pose count
a3i32 a3hierarchyPoseGroupCreate(a3_HierarchyPoseGroup *poseGroup_out, const a3_Hierarchy *hierarchy, const a3ui32 poseCount)
{
	// validate params and initialization states
	//	(output is not yet initialized, hierarchy is initialized)
	if (poseGroup_out && hierarchy && !poseGroup_out->hierarchy && hierarchy->nodes)
	{
		// determine memory requirements
		a3ui32 numNodes = hierarchy->numNodes;
		a3ui32 hierarchyPoseCount = poseCount;
		a3ui32 hposeSize = sizeof(a3_HierarchyPose) * hierarchyPoseCount;
		a3ui32 spatialPoseCount = hierarchyPoseCount * numNodes;
		a3ui32 spatialPoseSize = sizeof(a3_SpatialPose) * spatialPoseCount;
		a3ui32 channelSize = sizeof(a3_SpatialPoseChannel) * numNodes;
		a3ui32 memSize = hposeSize + spatialPoseSize + channelSize;

		// allocate everything (one malloc)
		poseGroup_out->HierarchyPosePool = (a3_HierarchyPose*)malloc(memSize);

		// set pointers
		poseGroup_out->hierarchy = hierarchy;
		poseGroup_out->spatialPosePool = (a3_SpatialPose*) poseGroup_out->HierarchyPosePool + hierarchyPoseCount;
		poseGroup_out->channels = (a3_SpatialPoseChannel*)poseGroup_out->spatialPosePool + spatialPoseCount;

		for (a3ui32 i = 0; i < hierarchyPoseCount; i++)
		{
			for (a3ui32 j = 0; j < numNodes; j++)
			{
				poseGroup_out->HierarchyPosePool[i].spatialPose = poseGroup_out->spatialPosePool + j;
			}
		}


		// reset all data
		a3hierarchyPoseReset(poseGroup_out->HierarchyPosePool, spatialPoseCount);
		memset(poseGroup_out->channels, a3poseChannel_none, channelSize);
		poseGroup_out->poseCount = hierarchyPoseCount;
		poseGroup_out->spatialPoseCount = spatialPoseCount;

		// done
		return 1;
	}
	return -1;
}

// release pose set
a3i32 a3hierarchyPoseGroupRelease(a3_HierarchyPoseGroup *poseGroup)
{
	// validate param exists and is initialized
	if (poseGroup && poseGroup->hierarchy)
	{
		// release everything (one free)
		//free(???);

		// reset pointers
		poseGroup->hierarchy = 0;

		// done
		return 1;
	}
	return -1;
}


//-----------------------------------------------------------------------------

// initialize hierarchy state given an initialized hierarchy
a3i32 a3hierarchyStateCreate(a3_HierarchyState *state_out, const a3_Hierarchy *hierarchy)
{
	// validate params and initialization states
	//	(output is not yet initialized, hierarchy is initialized)
	if (state_out && hierarchy && !state_out->hierarchy && hierarchy->nodes)
	{
		// determine memory requirements
		a3ui32 numNodes = hierarchy->numNodes;
		a3ui32 hierarchyPoseCount = 4;
		a3ui32 spatialPoseCount = hierarchyPoseCount * numNodes;
		a3ui32 memSize = sizeof(a3_SpatialPose) * spatialPoseCount;
		// allocate everything (one malloc)
		state_out->localSpacePose->spatialPose = (a3_SpatialPose*)malloc(memSize);

		// set pointers
		state_out->hierarchy = hierarchy;
		state_out->objectSpacePose->spatialPose = state_out->objectSpacePose->spatialPose + numNodes;

		for (a3ui32 i = 0; i < hierarchyPoseCount; i++)
		{
			for (a3ui32 j = 0; j < numNodes; j++)
			{
				state_out->localSpacePose[i].spatialPose = state_out->localSpacePose->spatialPose + j;
			}
		}

		// reset all data
		a3hierarchyPoseReset(state_out->localSpacePose, spatialPoseCount);

		// done
		return 1;
	}
	return -1;
}

// release hierarchy state
a3i32 a3hierarchyStateRelease(a3_HierarchyState *state)
{
	// validate param exists and is initialized
	if (state && state->hierarchy)
	{
		// release everything (one free)
		//free(???);

		// reset pointers
		state->hierarchy = 0;

		// done
		return 1;
	}
	return -1;
}


//-----------------------------------------------------------------------------

// load HTR file, read and store complete pose group and hierarchy
a3i32 a3hierarchyPoseGroupLoadHTR(a3_HierarchyPoseGroup* poseGroup_out, a3_Hierarchy* hierarchy_out, const a3byte* resourceFilePath)
{
	if (poseGroup_out && !poseGroup_out->poseCount && hierarchy_out && !hierarchy_out->numNodes && resourceFilePath && *resourceFilePath)
	{

	}
	return -1;
}

// load BVH file, read and store complete pose group and hierarchy
a3i32 a3hierarchyPoseGroupLoadBVH(a3_HierarchyPoseGroup* poseGroup_out, a3_Hierarchy* hierarchy_out, const a3byte* resourceFilePath)
{
	if (poseGroup_out && !poseGroup_out->poseCount && hierarchy_out && !hierarchy_out->numNodes && resourceFilePath && *resourceFilePath)
	{

	}
	return -1;
}

// save HTR file, write and store complete pose group and hierarchy
a3i32 a3hierarchyPoseGroupSaveHTR(a3_HierarchyPoseGroup* const poseGroup_in, a3_Hierarchy* const hierarchy_in, const a3byte* resourceFilePath)
{
	if (poseGroup_in && !poseGroup_in->poseCount && hierarchy_in && !hierarchy_in->numNodes && resourceFilePath && *resourceFilePath)
	{

	}
	return -1;
}
// save BVH file, write and store complete pose group and hierarchy
a3i32 a3hierarchyPoseGroupSaveBVH(a3_HierarchyPoseGroup* const poseGroup_in, a3_Hierarchy* const hierarchy_in, const a3byte* resourceFilePath)
{
	if (poseGroup_in && !poseGroup_in->poseCount && hierarchy_in && !hierarchy_in->numNodes && resourceFilePath && *resourceFilePath)
	{

	}
	return -1;
}

//-----------------------------------------------------------------------------
