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
	
	a3_KeyframeAnimationController.c
	Implementation of keyframe animation controller.
*/

#include "../a3_KeyframeAnimationController.h"

#include <string.h>


//-----------------------------------------------------------------------------

// initialize clip controller
a3i32 a3clipControllerInit(a3_ClipController* clipCtrl_out, const a3byte ctrlName[a3keyframeAnimation_nameLenMax], const a3_ClipPool* clipPool, const a3ui32 clipIndex_pool, a3f32 clipTime, a3f32 playbackDirection)
{
	for (a3ui32 i = 0; i < a3keyframeAnimation_nameLenMax; i++)
	{
		clipCtrl_out->name[i] = ctrlName[i];
	}
	clipCtrl_out->clipIndex = clipIndex_pool;
	clipCtrl_out->clipPool = clipPool;
	clipCtrl_out->playbackDirection = playbackDirection;
	clipCtrl_out->currentClip = &clipPool->clip[clipIndex_pool];

	// Forward/normal init
	if (playbackDirection >= 0)
	{
		clipCtrl_out->keyframeIndex0 = clipPool->clip[clipIndex_pool].first_keyframe;
		clipCtrl_out->keyframePtr0 = &clipPool->clip[clipIndex_pool].framePool->keyframe[clipPool->clip[clipIndex_pool].first_keyframe];
		clipCtrl_out->keyframeTime = 0;
		clipCtrl_out->clipTime = clipTime;
	}
	// Reverse init
	else
	{
		clipCtrl_out->keyframeIndex0 = clipPool->clip[clipIndex_pool].last_keyframe;
		clipCtrl_out->keyframePtr0 = &clipPool->clip[clipIndex_pool].framePool->keyframe[clipPool->clip[clipIndex_pool].last_keyframe];
		clipCtrl_out->keyframeTime = clipCtrl_out->keyframePtr0->duration;

		// interpret clipTime 0 as requesting the "end" of the clip
		if (clipTime == 0)
		{
			clipCtrl_out->clipTime = clipPool->clip[clipIndex_pool].duration;
		}
	}

	return 1;
}

//-----------------------------------------------------------------------------
