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
	
	a3_KeyframeAnimation.c
	Implementation of keyframe animation interfaces.
*/

#include "../a3_KeyframeAnimation.h"
#include "../a3_HierarchyState.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// macros to help with names
#define A3_CLIP_DEFAULTNAME		("unnamed clip")
#define A3_CLIP_SEARCHNAME		((clipName && *clipName) ? clipName : A3_CLIP_DEFAULTNAME)


//-----------------------------------------------------------------------------

a3_Sample* a3sampleLerp(a3_Sample* sample_out, a3_Sample const* sample0, a3_Sample const* sample1, a3real u)
{
	a3hierarchyPoseLerp(sample_out->pose, sample0->pose, sample1->pose, u, sample_out->numNodes);
	return sample_out;
}

a3ui32 a3sampleInit(a3_Sample* sample_out, a3ui32 numNodes)
{
	// determine memory requirements
	a3ui32 const nodeCount = numNodes;
	a3ui32 const hposeCount = 4;
	a3ui32 const sposeCount = hposeCount * nodeCount;
	a3ui32 const memreq = sizeof(a3_SpatialPose) * sposeCount;

	// allocate everything (one malloc)
	sample_out->pose = (a3_HierarchyPose*)malloc(sizeof(a3_HierarchyPose));
	sample_out->pose->pose = (a3_SpatialPose*)malloc(memreq);

	// set pointers
	sample_out->numNodes = numNodes;

	// reset all data
	a3hierarchyPoseReset(sample_out->pose, sposeCount);

	// done
	return 1;
	return -1;
}

//-----------------------------------------------------------------------------

// allocate keyframe pool
a3i32 a3keyframePoolCreate(a3_KeyframePool* keyframePool_out, const a3ui32 count)
{
	// initialize array of keyframes
	keyframePool_out->keyframe = malloc(count * sizeof(a3_Keyframe));

	keyframePool_out->count = count;
	return -1;
}

// release keyframe pool
a3i32 a3keyframePoolRelease(a3_KeyframePool* keyframePool)
{
	// deallocate array of keyframes
	free(keyframePool->keyframe);
	return -1;
}

// initialize keyframe
a3i32 a3keyframeInit(a3_Keyframe* keyframe_out, const a3real duration, const a3ui32 value_x)
{
	// setting duration and its inverse
	keyframe_out->duration = duration;
	keyframe_out->durationInv = 1.0f / duration;

	// setting keyframe data
	keyframe_out->data = value_x;

	return -1;
}

// initialize keyframe
a3i32 a3keyframeInitHpose(a3_Keyframe* keyframe_out, const a3real duration, a3_HierarchyPose* hPose)
{
	// setting duration and its inverse
	keyframe_out->duration = duration;
	keyframe_out->durationInv = 1.0f / duration;

	// setting keyframe data
	keyframe_out->sample.pose = hPose;

	return -1;
}

//create clip pool from file
a3i32 a3clipPoolCreateFromFile(a3_ClipPool* clipPool_out, const char* filePath)
{
	// open specified file
	a3i32 maxChars = 256;
	a3ui64 fileSize = 0;
	char* buffer;

	FILE* file = fopen(filePath, "r");
	if (file == NULL)
	{
		printf("Error opening file: ");
		printf(filePath);
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
	char lineDelim[] = "\n";
	char* ptr = strtok(buffer, lineDelim);
	while (ptr != NULL)
	{
		printf("'%s'", ptr);
		ptr = strtok(NULL, lineDelim);
	}

	return -1;
}

// allocate clip pool
a3i32 a3clipPoolCreate(a3_ClipPool* clipPool_out, const a3ui32 count)
{
	// initialize array of clips
	clipPool_out->clip = malloc(count * sizeof(a3_Clip));
	clipPool_out->count = count;
	return 1;
}

// release clip pool
a3i32 a3clipPoolRelease(a3_ClipPool* clipPool)
{
	// deallocate array of clips
	free(clipPool->clip);
	return -1;
}

// initialize clip with first and last indices
a3i32 a3clipInit(a3_Clip* clip_out, const a3byte clipName[a3keyframeAnimation_nameLenMax], const a3_KeyframePool* keyframePool, const a3ui32 firstKeyframeIndex, const a3ui32 finalKeyframeIndex,
const a3_ClipTransition* forwardClipTransition, const a3_ClipTransition* reverseClipTransition)
{
	// set name
	for (a3ui32 i = 0; i < a3keyframeAnimation_nameLenMax; i++)
	{
		clip_out->name[i] = clipName[i];
	}

	// set referenced keyframe pool
	clip_out->framePool = keyframePool;
	clip_out->keyframeCount = finalKeyframeIndex - firstKeyframeIndex;

	// set first and last keyframe indices
	clip_out->first_keyframe = firstKeyframeIndex;
	clip_out->last_keyframe = finalKeyframeIndex;

	clip_out->forwardTransition = forwardClipTransition;
	clip_out->reverseTransition = reverseClipTransition;

	a3clipCalculateDuration(clip_out);
	return -1;
}

// get clip index from pool
a3i32 a3clipGetIndexInPool(const a3_ClipPool* clipPool, const a3byte clipName[a3keyframeAnimation_nameLenMax])
{
	for (a3ui32 i = 0; i < clipPool->count; i++)
	{
		if (clipPool->clip[i].name == clipName)
			return i;
	}
	return -1;
}

a3i32 a3clipTransitionInit(a3_ClipTransition* clipTransition_out, a3_ClipPool* pool, a3index index, a3f32 startTime, a3f32 clipPlaybackDirection)
{
	clipTransition_out->clipPool = pool;
	clipTransition_out->clipIndex = index;
	clipTransition_out->clipTime = startTime;
	clipTransition_out->playbackDirection = clipPlaybackDirection;

	return 1;
}


//-----------------------------------------------------------------------------
