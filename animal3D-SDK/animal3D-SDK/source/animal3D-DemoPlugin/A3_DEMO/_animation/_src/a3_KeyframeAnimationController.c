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
a3i32 a3clipControllerInit(a3_ClipController* clipCtrl_out, const a3byte ctrlName[a3keyframeAnimation_nameLenMax], const a3_ClipPool* clipPool, const a3ui32 clipIndex_pool)
{
	return -1;
}

// update clip controller
a3i32 a3clipControllerUpdate(a3_ClipController* clipCtrl, const a3real dt)
{
	clipCtrl->currentClip = &clipCtrl->clipPool->clip[clipCtrl->clipIndex];
	clipCtrl->currentKeyframe = &clipCtrl->currentClip->framePool[clipCtrl->keyframeIndex];


	
	if (clipCtrl->playbackDirection == 0)
	{
		//break;
	}
	//forward
	if (clipCtrl->playbackDirection == 1)
	{
		clipCtrl->clipTime += dt;
		clipCtrl->keyframeTime += dt;

		if (clipCtrl->keyframeTime >= clipCtrl->currentKeyframe->duration)
		{
			//move to next keyframe
			clipCtrl->keyframeIndex++;
			clipCtrl->currentKeyframe = &clipCtrl->currentClip->framePool[clipCtrl->keyframeIndex];

			//move keyFrameTime to next keyframe
			clipCtrl->keyframeTime = clipCtrl->keyframeTime - clipCtrl->currentKeyframe->duration;
		}

		//t = (current time - key start * durationInv
		clipCtrl->keyframeParam = (clipCtrl->keyframeTime - clipCtrl->currentKeyframe->duration) * clipCtrl->currentKeyframe->durationInv;
	}

	//reverse
	if (clipCtrl->playbackDirection == -1)
	{
		clipCtrl->clipTime -= dt;
		clipCtrl->keyframeTime -= dt;

		if (clipCtrl->keyframeTime < clipCtrl->currentKeyframe->duration)
		{
			//move to next keyframe
			clipCtrl->keyframeIndex--;
			clipCtrl->currentKeyframe = &clipCtrl->currentClip->framePool[clipCtrl->keyframeIndex];

			//move keyFrameTime to next keyframe
			clipCtrl->keyframeTime = clipCtrl->keyframeTime - clipCtrl->currentKeyframe->duration;
		}

		//t = (current time - key start * durationInv
		clipCtrl->keyframeParam = (clipCtrl->keyframeTime - clipCtrl->currentKeyframe->duration) * clipCtrl->currentKeyframe->durationInv;
	}
	
}

// set clip to play
a3i32 a3clipControllerSetClip(a3_ClipController* clipCtrl, const a3_ClipPool* clipPool, const a3ui32 clipIndex_pool)
{

}


//-----------------------------------------------------------------------------
