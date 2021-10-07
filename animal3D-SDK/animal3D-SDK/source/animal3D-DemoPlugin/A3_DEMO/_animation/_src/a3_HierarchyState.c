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
#include <stdio.h>
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
		a3ui32 spatialPoseCount = poseCount * hierarchy->numNodes;
		a3ui32 spatialPoseSize = sizeof(a3_SpatialPose) * spatialPoseCount;
		a3ui32 hPosesSize = sizeof(a3_HierarchyPose) * poseCount;
		a3ui32 channelSize = sizeof(a3_SpatialPoseChannel) * hierarchy->numNodes;

		// allocate everything (one malloc)
		poseGroup_out->spatialPosePool = (a3_SpatialPose*)malloc(spatialPoseSize);
		poseGroup_out->HPoses = (a3_HierarchyPose*)malloc(hPosesSize);

		// set pointers
		poseGroup_out->hierarchy = hierarchy;
		poseGroup_out->poseCount = poseCount;
		poseGroup_out->spatialPoseCount = spatialPoseCount;
		poseGroup_out->channels = (a3_SpatialPoseChannel*)malloc(channelSize);
		
		a3ui32 i = 0;
		a3ui32 j = 0;
		for (i = 0; i < poseCount; i++)
		{
			j = hierarchy->numNodes * i;
			poseGroup_out->HPoses[i].spatialPose = poseGroup_out->spatialPosePool + j;
		}

		// reset all data
		for (i = 0; i < poseCount; i++)
		{
			a3hierarchyPoseReset(poseGroup_out->HPoses + i, hierarchy->numNodes);
		}

		memset(poseGroup_out->channels, a3poseChannel_none, channelSize);

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
		a3ui32 poseCount = 3; // 3 because 3 hposes (Sample, Object, and Local)

		// determine memory requirements
		a3ui32 hMemSize = sizeof(a3_HierarchyPose) * 3;
		a3ui32 poseMemSize = sizeof(a3_SpatialPose) * hierarchy->numNodes * poseCount; // * 3 because 3 hposes (Sample, Object, and Local)
		// allocate everything (one malloc)
		state_out->localSpacePose = (a3_HierarchyPose*)malloc(hMemSize + poseMemSize);
		
		// set pointers
		state_out->hierarchy = hierarchy;
		state_out->objectSpacePose = state_out->localSpacePose + 1;
		state_out->samplePose = state_out->localSpacePose + 2;

		state_out->localSpacePose->spatialPose = (a3_SpatialPose*)state_out->localSpacePose + hMemSize;
		state_out->objectSpacePose->spatialPose = state_out->localSpacePose->spatialPose + 1;
		state_out->samplePose->spatialPose = state_out->localSpacePose->spatialPose + 2;

		// reset all data
		a3hierarchyPoseReset(state_out->localSpacePose, hierarchy->numNodes);
		a3hierarchyPoseReset(state_out->objectSpacePose, hierarchy->numNodes);
		a3hierarchyPoseReset(state_out->samplePose, hierarchy->numNodes);

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
		// open specified file
		a3i32 maxChars = 256;
		a3ui64 fileSize = 0;
		char* buffer;

		FILE* file = fopen(resourceFilePath, "r");
		if (file == NULL)
		{
			printf("Error opening file: ");
			printf(resourceFilePath);
			return -1;
		}

		// get size of file
		else
		{
			fseek(file, 0L, SEEK_END);
			fileSize = ftell(file);
			fseek(file, 0L, SEEK_SET);
		}

		// read file into string
		buffer = malloc(fileSize + 1);
		if (!buffer) fclose(file), printf("failed to allocate memory"), stderr, exit(1);
		fread(buffer, 1, fileSize, file);

		// null terminate buffer for safety
		buffer[fileSize] = '\0';

		// close file
		fclose(file);

		// break up text using delimiters
		char lineDelim[] = " \n\t";
		char* ptr = strtok(buffer, lineDelim);

		enum Tag
		{
			HEADER,
			SEGMENTNAMES,
			BASEPOSE,
			FRAMES,
			NONE,
		} currentTag = 0;
		a3byte* nodeName;
		while (ptr != NULL)
		{
			// skip comments
			if (*ptr == '#')
			{
				ptr = strtok(NULL, "\n");
				ptr = strtok(NULL, lineDelim);
			}
			if (strcmp(ptr, "[Header]") == 0)
			{
				currentTag = HEADER;
			}
			else if (strcmp(ptr, "[SegmentNames&Hierarchy]") == 0)
			{
				currentTag = SEGMENTNAMES;
				ptr = strtok(NULL, lineDelim);
				if (*ptr == '#')
				{
					ptr = strtok(NULL, "\n");
					ptr = strtok(NULL, lineDelim);
				}
			}
			else if (strcmp(ptr, "[BasePosition]") == 0)
			{
				currentTag = BASEPOSE;
				ptr = strtok(NULL, lineDelim);
				if (*ptr == '#')
				{
					ptr = strtok(NULL, "\n");
					ptr = strtok(NULL, lineDelim);
				}
			}
			else if (*ptr == '[')
			{
				currentTag = FRAMES;
				nodeName = ptr;
				ptr = strtok(NULL, lineDelim);
				if (!ptr)
					break;
				if (*ptr == '#')
				{
					ptr = strtok(NULL, "\n");
					ptr = strtok(NULL, lineDelim);
				}
			}
			
			switch (currentTag)
			{
			case HEADER:
				if (strcmp(ptr, "FileType") == 0)
				{
					ptr = strtok(NULL, lineDelim);
				}
				if (strcmp(ptr, "DataType") == 0)
				{
					ptr = strtok(NULL, lineDelim);
				}
				if (strcmp(ptr, "FileVersion") == 0)
				{
					ptr = strtok(NULL, lineDelim);
				}
				if (strcmp(ptr, "NumSegments") == 0)
				{
					ptr = strtok(NULL, lineDelim);
					sscanf(ptr, "%d", &hierarchy_out->numNodes);
				}
				if (strcmp(ptr, "NumFrames") == 0)
				{
					ptr = strtok(NULL, lineDelim);
					sscanf(ptr, "%d", &poseGroup_out->poseCount);
					poseGroup_out->poseCount += 1;
				}
				if (strcmp(ptr, "DataFrameRate") == 0)
				{
					ptr = strtok(NULL, lineDelim);
				}
				if (strcmp(ptr, "EulerRotationOrder") == 0)
				{
					ptr = strtok(NULL, lineDelim);
					sscanf(ptr, "%d", &poseGroup_out->eulerOrder);
				}
				if (strcmp(ptr, "CalibrationUnits") == 0)
				{
					ptr = strtok(NULL, lineDelim);
				}
				if (strcmp(ptr, "RotationUnits") == 0)
				{
					ptr = strtok(NULL, lineDelim);
				}
				if (strcmp(ptr, "GlobalAxisofGravity") == 0)
				{
					ptr = strtok(NULL, lineDelim);
				}
				if (strcmp(ptr, "BoneLengthAxis") == 0)
				{
					ptr = strtok(NULL, lineDelim);
				}
				if (strcmp(ptr, "ScaleFactor") == 0)
				{
					ptr = strtok(NULL, lineDelim);
				}
				break;
			case SEGMENTNAMES:
				
				nodeName = ptr;
				a3hierarchyCreate(hierarchy_out, hierarchy_out->numNodes, 0);
				a3hierarchySetNode(hierarchy_out, 0, -1, nodeName);
				ptr = strtok(NULL, lineDelim);
				for (a3ui32 i = 1; i < hierarchy_out->numNodes; i++)
				{
					if (*ptr != '#')
					{
						nodeName = ptr;
						ptr = strtok(NULL, lineDelim);
						nodeName = ptr;
						ptr = strtok(NULL, lineDelim);
						a3hierarchySetNode(hierarchy_out, i, a3hierarchyGetNodeIndex(hierarchy_out, ptr), nodeName);
					}
					else
					{
						// skip comment line
						ptr = strtok(NULL, "\n");
						i--;
					}
				}
				//currentTag = NONE;
				a3hierarchyPoseGroupCreate(poseGroup_out, hierarchy_out, poseGroup_out->poseCount);
				break;
			case BASEPOSE:

				for (a3ui32 i = 1; i < hierarchy_out->numNodes; i++)
				{
					if (*ptr != '#')
					{
						nodeName = ptr;
						ptr = strtok(NULL, lineDelim);
						sscanf(ptr, "%f", &poseGroup_out->HPoses[0].spatialPose[a3hierarchyGetNodeIndex(hierarchy_out, nodeName)].translation.x);
						ptr = strtok(NULL, lineDelim);
						sscanf(ptr, "%f", &poseGroup_out->HPoses[0].spatialPose[a3hierarchyGetNodeIndex(hierarchy_out, nodeName)].translation.y);
						ptr = strtok(NULL, lineDelim);
						sscanf(ptr, "%f", &poseGroup_out->HPoses[0].spatialPose[a3hierarchyGetNodeIndex(hierarchy_out, nodeName)].translation.z);
						poseGroup_out->HPoses[0].spatialPose[a3hierarchyGetNodeIndex(hierarchy_out, nodeName)].translation.w = 1;

						ptr = strtok(NULL, lineDelim);
						sscanf(ptr, "%f", &poseGroup_out->HPoses[0].spatialPose[a3hierarchyGetNodeIndex(hierarchy_out, nodeName)].rotate_euler.x);
						ptr = strtok(NULL, lineDelim);
						sscanf(ptr, "%f", &poseGroup_out->HPoses[0].spatialPose[a3hierarchyGetNodeIndex(hierarchy_out, nodeName)].rotate_euler.y);
						ptr = strtok(NULL, lineDelim);
						sscanf(ptr, "%f", &poseGroup_out->HPoses[0].spatialPose[a3hierarchyGetNodeIndex(hierarchy_out, nodeName)].rotate_euler.z);
						poseGroup_out->HPoses[0].spatialPose[a3hierarchyGetNodeIndex(hierarchy_out, nodeName)].rotate_euler.w = 1;
						
						// bone length
						ptr = strtok(NULL, lineDelim);

						//next line
						ptr = strtok(NULL, lineDelim);
					}
					else
					{
						// skip comment line
						ptr = strtok(NULL, "\n");
						i--;
					}
				}
				currentTag = NONE;
				break;
			case FRAMES:
				nodeName++[strlen(nodeName) - 1] = 0;
				a3ui32 i = a3hierarchyGetNodeIndex(hierarchy_out, nodeName);
				a3ui32 index = 0;
				for (a3ui32 j = 0; j < poseGroup_out->poseCount -1; j++)
				{
					sscanf(ptr, "%d", &index);
					index++;
					ptr = strtok(NULL, lineDelim);
					sscanf(ptr, "%f", &poseGroup_out->HPoses[index].spatialPose[i].translation.x);
					ptr = strtok(NULL, lineDelim);
					sscanf(ptr, "%f", &poseGroup_out->HPoses[index].spatialPose[i].translation.y);
					ptr = strtok(NULL, lineDelim);
					sscanf(ptr, "%f", &poseGroup_out->HPoses[index].spatialPose[i].translation.z);
					poseGroup_out->HPoses[0].spatialPose[a3hierarchyGetNodeIndex(hierarchy_out, nodeName)].translation.w = 1;

					ptr = strtok(NULL, lineDelim);
					sscanf(ptr, "%f", &poseGroup_out->HPoses[index].spatialPose[i].rotate_euler.x);
					ptr = strtok(NULL, lineDelim);
					sscanf(ptr, "%f", &poseGroup_out->HPoses[index].spatialPose[i].rotate_euler.y);
					ptr = strtok(NULL, lineDelim);
					sscanf(ptr, "%f", &poseGroup_out->HPoses[index].spatialPose[i].rotate_euler.z);
					poseGroup_out->HPoses[0].spatialPose[a3hierarchyGetNodeIndex(hierarchy_out, nodeName)].rotate_euler.w = 1;

					// bone length
					ptr = strtok(NULL, lineDelim);

					ptr = strtok(NULL, lineDelim);
				}
				//currentTag = NONE;
				break;
			case NONE:
				break;
			}
			if (currentTag != FRAMES)
			{
				ptr = strtok(NULL, lineDelim);
			}
			
		}
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
