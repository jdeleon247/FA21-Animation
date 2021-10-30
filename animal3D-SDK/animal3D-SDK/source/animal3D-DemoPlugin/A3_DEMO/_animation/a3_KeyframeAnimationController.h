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
	
	a3_KeyframeAnimationController.h
	Keyframe animation clip controller. Basically a frame index manager. Very 
	limited in what one can do with this; could potentially be so much more.
*/

#ifndef __ANIMAL3D_KEYFRAMEANIMATIONCONTROLLER_H
#define __ANIMAL3D_KEYFRAMEANIMATIONCONTROLLER_H


#include "a3_KeyframeAnimation.h"


//-----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C"
{
#else	// !__cplusplus
typedef struct a3_ClipController			a3_ClipController;
#endif	// __cplusplus


//-----------------------------------------------------------------------------

// clip controller
// metaphor: playhead
struct a3_ClipController
{
	a3byte name[a3keyframeAnimation_nameLenMax];

	//index of clip to control in referenced clip pool
	a3index clipIndex;

	//current time relative to start of clip between 0 and current clip's duration
	a3f32 clipTime;

	//normalized clip time [0-1]
	a3f32 clipParam;

	//index of current keyframe in referenced keyframe pool
	//a3index keyframeIndex;
	a3index keyframeIndex0, keyframeIndex1;

	//current time relative to current keyframe | between 0 and current keyframe's duration
	a3f32 keyframeTime;

	//normalized keyframe time [0-1]
	a3f32 keyframeParam;

	//playback behavior: +1 for forward | 0 for pause | -1 for reverse
	a3f32 playbackDirection;

	//pointer to pool of clips to control
	const a3_ClipPool* clipPool;

	//pointer to current clip
	a3_Clip const* currentClip;

	//pointer to current keyframe
	a3_Keyframe const* keyframePtr0;
	//pointer to next keyframe
	a3_Keyframe const* keyframePtr1;
};


//-----------------------------------------------------------------------------

// initialize clip controller
a3i32 a3clipControllerInit(a3_ClipController* clipCtrl_out, const a3byte ctrlName[a3keyframeAnimation_nameLenMax], const a3_ClipPool* clipPool, const a3ui32 clipIndex_pool, a3f32 clipTime, a3f32 playbackDirection);

// update clip controller
a3i32 a3clipControllerUpdate(a3_ClipController* clipCtrl, const a3real dt);

// set clip to play
a3i32 a3clipControllerSetClip(a3_ClipController* clipCtrl, const a3_ClipPool* clipPool, const a3ui32 clipIndex_pool);

// evaluate the current value at time
a3i32 a3clipControllerEvaluate(a3_ClipController const* clipCtrl, a3_Sample* sample_out);

//-----------------------------------------------------------------------------


#ifdef __cplusplus
}
#endif	// __cplusplus


#include "_inl/a3_KeyframeAnimationController.inl"


#endif	// !__ANIMAL3D_KEYFRAMEANIMATIONCONTROLLER_H