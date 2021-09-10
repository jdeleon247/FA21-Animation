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
	
	a3_KeyframeAnimationController.h
	inline definitions for keyframe animation controller.
*/

#ifdef __ANIMAL3D_KEYFRAMEANIMATIONCONTROLLER_H
#ifndef __ANIMAL3D_KEYFRAMEANIMATIONCONTROLLER_INL
#define __ANIMAL3D_KEYFRAMEANIMATIONCONTROLLER_INL


//-----------------------------------------------------------------------------

// update clip controller
inline a3i32 a3clipControllerUpdate(a3_ClipController* clipCtrl, const a3real dt)
{
	clipCtrl->currentClip = &clipCtrl->clipPool->clip[clipCtrl->clipIndex];
	clipCtrl->currentKeyframe = &clipCtrl->currentClip->framePool->keyframe[clipCtrl->keyframeIndex];

	if (clipCtrl->playbackDirection == 0)
	{
		//break;
	}
	//forward
	if (clipCtrl->playbackDirection == 1)
	{
		//Pre-resolution | increment time.
		clipCtrl->clipTime += dt;
		clipCtrl->keyframeTime += dt;

		//Resolve time | Update keyframe details if necessary
		if (clipCtrl->keyframeTime >= clipCtrl->currentKeyframe->duration)
		{
			//move to next keyframe
			clipCtrl->keyframeIndex++;
			clipCtrl->currentKeyframe = &clipCtrl->currentClip->framePool->keyframe[clipCtrl->keyframeIndex];

			//move keyFrameTime to next keyframe
			clipCtrl->keyframeTime = clipCtrl->keyframeTime - clipCtrl->currentKeyframe->duration;
		}

		//Post-resolution | normalize params.
		//t = (current time - key start * durationInv
		clipCtrl->keyframeParam = (clipCtrl->keyframeTime - clipCtrl->currentKeyframe->duration) * clipCtrl->currentKeyframe->durationInv;
		clipCtrl->clipParam = (clipCtrl->clipTime - clipCtrl->currentClip->duration) * clipCtrl->currentClip->durationInv;

		//Loop after finish
		if (clipCtrl->clipParam == 1)
		{
			clipCtrl->keyframeIndex = clipCtrl->currentClip->first_keyframe;
			clipCtrl->currentKeyframe = &clipCtrl->currentClip->framePool->keyframe[clipCtrl->keyframeIndex];
		}
	}

	//reverse
	if (clipCtrl->playbackDirection == -1)
	{
		//Pre-resolution | increment time.
		clipCtrl->clipTime -= dt;
		clipCtrl->keyframeTime -= dt;

		//Resolve time | Update keyframe details if necessary
		if (clipCtrl->keyframeTime < clipCtrl->currentKeyframe->duration)
		{
			//move to next keyframe
			clipCtrl->keyframeIndex--;
			clipCtrl->currentKeyframe = &clipCtrl->currentClip->framePool->keyframe[clipCtrl->keyframeIndex];

			//move keyFrameTime to next keyframe
			clipCtrl->keyframeTime = clipCtrl->keyframeTime - clipCtrl->currentKeyframe->duration;
		}

		//Post-resolution | normalize params.
		//t = (current time - key start * durationInv
		clipCtrl->keyframeParam = (clipCtrl->keyframeTime - clipCtrl->currentKeyframe->duration) * clipCtrl->currentKeyframe->durationInv;
		clipCtrl->keyframeParam = (clipCtrl->keyframeTime - clipCtrl->currentKeyframe->duration) * clipCtrl->currentKeyframe->durationInv;

		//Loop after finish
		if (clipCtrl->clipParam == 0)
		{
			clipCtrl->keyframeIndex = clipCtrl->currentClip->last_keyframe;
			clipCtrl->currentKeyframe = &clipCtrl->currentClip->framePool->keyframe[clipCtrl->keyframeIndex];
		}
	}
	return 1;
}

// set clip to play
inline a3i32 a3clipControllerSetClip(a3_ClipController* clipCtrl, const a3_ClipPool* clipPool, const a3ui32 clipIndex_pool)
{
	clipCtrl->clipIndex = clipIndex_pool;
	clipCtrl->clipPool = clipPool;
	return -1;
}


//-----------------------------------------------------------------------------


#endif	// !__ANIMAL3D_KEYFRAMEANIMATIONCONTROLLER_INL
#endif	// __ANIMAL3D_KEYFRAMEANIMATIONCONTROLLER_H