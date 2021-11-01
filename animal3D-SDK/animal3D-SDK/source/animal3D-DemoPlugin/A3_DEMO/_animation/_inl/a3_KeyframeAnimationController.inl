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
	inline definitions for keyframe animation controller.
*/

#ifdef __ANIMAL3D_KEYFRAMEANIMATIONCONTROLLER_H
#ifndef __ANIMAL3D_KEYFRAMEANIMATIONCONTROLLER_INL
#define __ANIMAL3D_KEYFRAMEANIMATIONCONTROLLER_INL


//-----------------------------------------------------------------------------

// update clip controller
inline a3i32 a3clipControllerUpdate(a3_ClipController* clipCtrl, const a3real dt)
{
	// step/nearest:
	// nothing special, same as lab1

	// lerp:
	// if end, k0 <- k1, calc new k1
	// if begin, k1 <- k0, calc new k0

	// Catmull:
	// if over: kp <- k0 <- k1 <- kn, calc new kn
	// if begin: kn <- k1 <- k0 <- kp, calc new kp


	clipCtrl->currentClip = &clipCtrl->clipPool->clip[clipCtrl->clipIndex];
	clipCtrl->keyframePtr0 = &clipCtrl->currentClip->framePool->keyframe[clipCtrl->keyframeIndex0];
	clipCtrl->keyframePtr1 = &clipCtrl->currentClip->framePool->keyframe[clipCtrl->keyframeIndex0];

	if (clipCtrl->clipIndex != clipCtrl->currentClip->index);
	{
		clipCtrl->currentClip = &clipCtrl->clipPool->clip[clipCtrl->clipIndex];
		//a3clipControllerInit(clipCtrl, clipCtrl->name, clipCtrl->currentClip->forwardTransition->clipPool,
			//clipCtrl->currentClip->forwardTransition->clipIndex, clipCtrl->currentClip->forwardTransition->clipTime,
			//clipCtrl->currentClip->forwardTransition->playbackDirection);
	}

	if (clipCtrl->playbackDirection == 0)
	{
		//break;
	}
	//forward
	if (clipCtrl->playbackDirection == 1)
	{
		clipCtrl->keyframePtr1 = &clipCtrl->currentClip->framePool->keyframe[clipCtrl->keyframeIndex0 + 1];
		if (!clipCtrl->keyframePtr1)
		{
			clipCtrl->keyframePtr1 = &clipCtrl->currentClip->framePool->keyframe[clipCtrl->keyframeIndex0];
		}

		//Pre-resolution | increment time.
		clipCtrl->clipTime += dt;
		clipCtrl->keyframeTime += dt;

		//Resolve time
		// Forward terminus
		if (clipCtrl->clipParam >= 1)
		{
			a3clipControllerInit(clipCtrl, clipCtrl->name, clipCtrl->currentClip->forwardTransition->clipPool,
				clipCtrl->currentClip->forwardTransition->clipIndex, clipCtrl->currentClip->forwardTransition->clipTime,
				clipCtrl->currentClip->forwardTransition->playbackDirection);
		}
		// Update keyframe details if necessary
		if (clipCtrl->keyframeTime >= clipCtrl->keyframePtr0->duration)
		{
			//move keyFrameTime to next keyframe
			clipCtrl->keyframeTime = clipCtrl->keyframeTime - clipCtrl->keyframePtr0->duration;

			//move to next keyframe
			if (clipCtrl->currentClip->keyframeCount + clipCtrl->currentClip->first_keyframe > clipCtrl->keyframeIndex0)
			{
				clipCtrl->keyframeIndex0++;
				clipCtrl->keyframePtr0 = &clipCtrl->currentClip->framePool->keyframe[clipCtrl->keyframeIndex0];
			}
		}

		//Post-resolution | normalize params.
		//t = (current time - key start * durationInv
		clipCtrl->keyframeParam = clipCtrl->keyframeTime * clipCtrl->keyframePtr0->durationInv;
		clipCtrl->clipParam = clipCtrl->clipTime * clipCtrl->currentClip->durationInv;

	}

	//reverse
	if (clipCtrl->playbackDirection == -1)
	{
		clipCtrl->keyframePtr1 = &clipCtrl->currentClip->framePool->keyframe[clipCtrl->keyframeIndex0 - 1];
		if (!clipCtrl->keyframePtr1)
		{
			clipCtrl->keyframePtr1 = &clipCtrl->currentClip->framePool->keyframe[clipCtrl->keyframeIndex0];
		}

		//Pre-resolution | increment time.
		clipCtrl->clipTime -= dt;
		clipCtrl->keyframeTime -= dt;

		//Resolve time
		// Reverse terminus
		if (clipCtrl->clipTime <= 0)
		{
			// apply reverse transition
			a3clipControllerInit(clipCtrl, clipCtrl->name, clipCtrl->currentClip->reverseTransition->clipPool,
				clipCtrl->currentClip->reverseTransition->clipIndex,
				clipCtrl->currentClip->reverseTransition->clipTime,
				clipCtrl->currentClip->reverseTransition->playbackDirection);
		}
		// Update keyframe details if necessary
		if (clipCtrl->keyframeTime < 0.0)
		{
			//move keyFrameTime to next keyframe
			clipCtrl->keyframeTime = clipCtrl->keyframePtr0->duration + clipCtrl->keyframeTime;

			//move to next keyframe
			if (clipCtrl->currentClip->first_keyframe < clipCtrl->keyframeIndex0)
			{
				clipCtrl->keyframeIndex0--;
				clipCtrl->keyframePtr0 = &clipCtrl->currentClip->framePool->keyframe[clipCtrl->keyframeIndex0];
			}
		}

		//Post-resolution | normalize params.
		//t = (current time - key start * durationInv
		clipCtrl->keyframeParam = clipCtrl->keyframeTime * clipCtrl->keyframePtr0->durationInv;
		clipCtrl->clipParam = clipCtrl->clipTime * clipCtrl->currentClip->durationInv;

		
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

// evaluate the current value at time
inline a3i32 a3clipControllerEvaluate(a3_ClipController const* clipCtrl, a3_Sample* sample_out)
{
	if (clipCtrl && clipCtrl->currentClip && sample_out)
	{
		// 0: no interpolation
		//*sample_out = clipCtrl->keyframePtr0->sample;

		// 1:
		// if (u <0.5) then k0, else k1

		// 2: lerp
		a3sampleLerp(sample_out, &clipCtrl->keyframePtr0->sample, &clipCtrl->keyframePtr1->sample, clipCtrl->keyframeParam);
		
		// 3: Catmull-Rom/cubic Hermite


		return clipCtrl->keyframeIndex0;
	}
	return -1;
}


//-----------------------------------------------------------------------------


#endif	// !__ANIMAL3D_KEYFRAMEANIMATIONCONTROLLER_INL
#endif	// __ANIMAL3D_KEYFRAMEANIMATIONCONTROLLER_H