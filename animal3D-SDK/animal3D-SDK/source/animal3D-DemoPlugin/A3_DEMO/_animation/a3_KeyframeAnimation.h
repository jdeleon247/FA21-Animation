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
	
	modified by Rory Beebout

	a3_KeyframeAnimation.h
	Data structures for fundamental keyframe animation.
*/

#ifndef __ANIMAL3D_KEYFRAMEANIMATION_H
#define __ANIMAL3D_KEYFRAMEANIMATION_H


#include "animal3D-A3DM/a3math/a3vector.h"
#include "animal3D-A3DM/a3math/a3interpolation.h"
#include "a3_HierarchyState.h"


//-----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#else	// !__cplusplus
typedef struct a3_Sample					a3_Sample;
typedef struct a3_Keyframe					a3_Keyframe;
typedef struct a3_KeyframePool				a3_KeyframePool;
typedef struct a3_Clip						a3_Clip;
typedef struct a3_ClipPool					a3_ClipPool;
typedef struct a3_ClipTransition			a3_ClipTransition;
#endif	// __cplusplus


//-----------------------------------------------------------------------------

// constant values
enum
{
	a3keyframeAnimation_nameLenMax = 32,
};


// single generic value at time
struct a3_Sample
{
	a3_HierarchyPose* pose;
};

// description of single keyframe
// metaphor: moment
struct a3_Keyframe
{
	// index in keyframe pool
	a3index index;

	// active time interval and its reciprocal
	a3real duration, durationInv;
	//a3f32 duration, durationInv;

	// sample value described by a keyframe
	a3i32 data;

	// the known sample at the start of the interval
	a3_Sample sample;

};

// pool of keyframe descriptors
struct a3_KeyframePool
{
	// array of keyframes
	a3_Keyframe *keyframe;

	// number of keyframes
	a3ui32 count;
};


// allocate keyframe pool
a3i32 a3keyframePoolCreate(a3_KeyframePool* keyframePool_out, const a3ui32 count);

// release keyframe pool
a3i32 a3keyframePoolRelease(a3_KeyframePool* keyframePool);

// initialize keyframe
a3i32 a3keyframeInit(a3_Keyframe* keyframe_out, const a3real duration, const a3ui32 value_x);

a3i32 a3keyframeInitHpose(a3_Keyframe* keyframe_out, const a3real duration, a3_HierarchyPose* hPose);


//-----------------------------------------------------------------------------

// description of single clip
// metaphor: timeline
struct a3_Clip
{
	// clip name
	a3byte name[a3keyframeAnimation_nameLenMax];

	// index in clip pool
	a3index index;

	// duration of clip and its reciprocal
	a3f32 duration, durationInv;

	// number of referenced keyframes
	a3ui32 keyframeCount;

	// index of first referenced keyframe in pool
	a3index first_keyframe;

	// index of final referenced keyframe in pool
	a3index last_keyframe;

	const a3_ClipTransition* forwardTransition;

	const a3_ClipTransition* reverseTransition;

	// array of keyframes
	const a3_KeyframePool* framePool;
};

// group of clips
struct a3_ClipPool
{
	// array of clips
	a3_Clip* clip;

	// number of clips
	a3ui32 count;
};

// clip transition
struct a3_ClipTransition
{
	// array of clips
	const a3_ClipPool* clipPool;

	// index of target clip
	a3index clipIndex;

	// clip time;
	a3f32 clipTime;

	// playbackDirection;
	a3f32 playbackDirection;
};

// create clip pool from file
a3i32 a3clipPoolCreateFromFile(a3_ClipPool* clipPool_out, const char* filePath);

// split input strings using a specific char
a3i32 a3ColumnCount(char* inputStr);

// allocate clip pool
a3i32 a3clipPoolCreate(a3_ClipPool* clipPool_out, const a3ui32 count);

// release clip pool
a3i32 a3clipPoolRelease(a3_ClipPool* clipPool);

// initialize clip with first and last indices
a3i32 a3clipInit(a3_Clip* clip_out, const a3byte clipName[a3keyframeAnimation_nameLenMax], const a3_KeyframePool* keyframePool, const a3ui32 firstKeyframeIndex, const a3ui32 finalKeyframeIndex,
const a3_ClipTransition* forwardClipTransition, const a3_ClipTransition* reverseClipTransition);

// get clip index from pool
a3i32 a3clipGetIndexInPool(const a3_ClipPool* clipPool, const a3byte clipName[a3keyframeAnimation_nameLenMax]);

// calculate clip duration as sum of keyframes' durations
a3i32 a3clipCalculateDuration(a3_Clip* clip);

// calculate keyframes' durations by distributing clip's duration
a3i32 a3clipDistributeDuration(a3_Clip* clip, const a3real newClipDuration);

// initialize clip transition with desired transition attributes
a3i32 a3clipTransitionInit(a3_ClipTransition* clipTransition_out, a3_ClipPool* pool, a3index index, a3f32 startTime, a3f32 clipPlaybackDirection);


//-----------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif	// __cplusplus


#include "_inl/a3_KeyframeAnimation.inl"


#endif	// !__ANIMAL3D_KEYFRAMEANIMATION_H