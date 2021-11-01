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
	
	a3_DemoMode1_Animation-idle-update.c
	Demo mode implementations: animation scene.

	********************************************
	*** UPDATE FOR ANIMATION SCENE MODE      ***
	********************************************
*/

//-----------------------------------------------------------------------------

#include "../a3_DemoMode1_Animation.h"

//typedef struct a3_DemoState a3_DemoState;
#include "../a3_DemoState.h"

#include "../_a3_demo_utilities/a3_DemoMacros.h"


//-----------------------------------------------------------------------------
// UTILS

inline a3real4r a3demo_mat2quat_safe(a3real4 q, a3real4x4 const m)
{
	// ****TO-DO: 
	//	-> convert rotation part of matrix to quaternion
	//	-> NOTE: this is for testing dual quaternion skinning only; 
	//		quaternion data would normally be computed with poses

	return q;
}


//-----------------------------------------------------------------------------
// UPDATE

void a3demo_update_objects(a3_DemoState* demoState, a3f64 const dt,
	a3_DemoSceneObject* sceneObjectBase, a3ui32 count, a3boolean useZYX, a3boolean applyScale);
void a3demo_update_defaultAnimation(a3_DemoState* demoState, a3f64 const dt,
	a3_DemoSceneObject* sceneObjectBase, a3ui32 count, a3ui32 axis);
void a3demo_update_bindSkybox(a3_DemoSceneObject* obj_camera, a3_DemoSceneObject* obj_skybox);
void a3demo_update_pointLight(a3_DemoSceneObject* obj_camera, a3_DemoPointLight* pointLightBase, a3ui32 count);

void a3demo_applyScale_internal(a3_DemoSceneObject* sceneObject, a3real4x4p s);

void a3animation_update(a3_DemoState* demoState, a3_DemoMode1_Animation* demoMode, a3f64 const dt)
{
	a3ui32 i;
	a3_DemoModelMatrixStack matrixStack[animationMaxCount_sceneObject];

	a3_HierarchyState* controlHS1 = demoMode->hierarchyState_skel + 3, * controlHS0 = demoMode->hierarchyState_skel + 2, * activeHS = demoMode->hierarchyState_skel + 1, * baseHS = demoMode->hierarchyState_skel;

	// active camera
	a3_DemoProjector const* activeCamera = demoMode->projector + demoMode->activeCamera;
	a3_DemoSceneObject const* activeCameraObject = activeCamera->sceneObject;

	// temp scale mat
	a3mat4 scaleMat = a3mat4_identity;

	a3demo_update_objects(demoState, dt,
		demoMode->object_scene, animationMaxCount_sceneObject, 0, 0);
	a3demo_update_objects(demoState, dt,
		demoMode->object_camera, animationMaxCount_cameraObject, 1, 0);

	a3demo_updateProjectorViewProjectionMat(demoMode->proj_camera_main);

	// apply scales to objects
	for (i = 0; i < animationMaxCount_sceneObject; ++i)
		a3demo_applyScale_internal(demoMode->object_scene + i, scaleMat.m);

	// update skybox
	a3demo_update_bindSkybox(demoMode->obj_camera_main, demoMode->obj_skybox);

	// update matrix stack data
	for (i = 0; i < animationMaxCount_sceneObject; ++i)
	{
		a3demo_updateModelMatrixStack(matrixStack + i,
			activeCamera->projectionMat.m, activeCameraObject->modelMat.m, activeCameraObject->modelMatInv.m,
			demoMode->object_scene[i].modelMat.m, a3mat4_identity.m);
	}

	// skeletal
	if (demoState->updateAnimation && !demoMode->playingAnim)
	{
		i = (a3ui32)(demoState->timer_display->totalTime);
		demoMode->hierarchyKeyPose_display[0] = (i + 0) % (demoMode->hierarchyPoseGroup_skel->hposeCount - 1);
		demoMode->hierarchyKeyPose_display[1] = (i + 1) % (demoMode->hierarchyPoseGroup_skel->hposeCount - 1);
		demoMode->hierarchyKeyPose_param = (a3real)(demoState->timer_display->totalTime - (a3f64)i);
	}

	for (i = 0; i < 2; i++)
	{
		a3clipControllerUpdate(demoMode->clipController + i, (a3real)dt);
	}

	//a3hierarchyPoseCopy(activeHS->objectSpace,
	//	demoMode->hierarchyPoseGroup_skel->hpose + demoMode->hierarchyKeyPose_display[0] + 1,
	//	demoMode->hierarchy_skel->numNodes);
	//a3hierarchyPoseLerp(activeHS->objectSpace,	// use as temp storage
	//	demoMode->hierarchyPoseGroup_skel->hpose + demoMode->hierarchyKeyPose_display[0] + 1,
	//	demoMode->hierarchyPoseGroup_skel->hpose + demoMode->hierarchyKeyPose_display[1] + 1,
	//	demoMode->hierarchyKeyPose_param,
	//	demoMode->hierarchy_skel->numNodes);
	//a3hierarchyPoseConcat(activeHS->localSpace,	// goal to calculate
	//	baseHS->localSpace, // holds base pose
	//	activeHS->objectSpace, // temp storage
	//	demoMode->hierarchy_skel->numNodes);

	a3_HierarchyPose const* blendControls[16];
	a3real const* inputParams[8];

	inputParams[0] = &demoMode->clipController[0].keyframeParam;
	inputParams[1] = &demoMode->clipController[1].keyframeParam;
	inputParams[2] = &demoMode->clipController[0].keyframeParam;
	inputParams[3] = &demoMode->clipController[1].keyframeParam;
	inputParams[4] = &demoMode->clipController[0].keyframeParam;
	inputParams[5] = &demoMode->clipController[1].keyframeParam;
	inputParams[6] = &demoMode->clipController[0].keyframeParam;
	inputParams[7] = &demoMode->clipController[1].keyframeParam;


	//blendControls[0] = demoMode->clipController[0].keyframePtr0->sample.pose
	blendControls[0] = demoMode->clipController[0].keyframePtr0->sample.pose;
	blendControls[1] = demoMode->clipController[0].keyframePtr1->sample.pose;
	blendControls[2] = demoMode->clipController[1].keyframePtr0->sample.pose;
	blendControls[3] = demoMode->clipController[1].keyframePtr1->sample.pose;
	blendControls[4] = demoMode->clipController[1].keyframePtr0->sample.pose;
	blendControls[5] = demoMode->clipController[1].keyframePtr0->sample.pose;
	blendControls[6] = demoMode->clipController[1].keyframePtr0->sample.pose;
	blendControls[7] = demoMode->clipController[1].keyframePtr0->sample.pose;
	blendControls[8] = demoMode->clipController[1].keyframePtr0->sample.pose;
	blendControls[9] = demoMode->clipController[1].keyframePtr0->sample.pose;
	blendControls[10] = demoMode->clipController[1].keyframePtr0->sample.pose;
	blendControls[11] = demoMode->clipController[1].keyframePtr0->sample.pose;
	blendControls[12] = demoMode->clipController[1].keyframePtr0->sample.pose;
	blendControls[13] = demoMode->clipController[1].keyframePtr0->sample.pose;
	blendControls[14] = demoMode->clipController[1].keyframePtr0->sample.pose;
	blendControls[15] = demoMode->clipController[1].keyframePtr0->sample.pose;

	//blendControls[2] = demoMode->hierarchyPoseGroup_skel->hpose + demoMode->hierarchyKeyPose_display[0] + 1;
	//blendControls[3] = demoMode->hierarchyPoseGroup_skel->hpose + demoMode->hierarchyKeyPose_display[1] + 1;

	a3hierarchyPoseOpLERP(controlHS0->objectSpace, blendControls, inputParams, demoMode->hierarchy_skel->numNodes);
	a3hierarchyPoseOpLERP(controlHS1->objectSpace, &blendControls[2], &inputParams[1], demoMode->hierarchy_skel->numNodes);

	blendControls[0] = controlHS0->objectSpace;
	blendControls[1] = controlHS1->objectSpace;



	a3_HierarchyPoseBlendNode lerp;
	lerp.op = a3hierarchyPoseOpLERP;
	for (a3ui32 i = 0; i < 8; ++i) {
		lerp.param[i] = inputParams[i];
		lerp.pose_ctrl[i] = blendControls[i];}
	lerp.pose_out = activeHS->localSpace;
	lerp.numNodes = demoMode->hierarchy_skel->numNodes;
	
	a3hierarchyPoseBlendNodeCall(&lerp);

	

	switch (demoMode->blendOpIndex)
	{
	case 0: a3hierarchyPoseOpIdentity(activeHS->localSpace, blendControls, inputParams, demoMode->hierarchy_skel->numNodes);
		break;
	case 1: a3hierarchyPoseOpConstruct(activeHS->objectSpace, 
		demoMode->hierarchyPoseGroup_skel->hpose->pose->angles, 
		demoMode->hierarchyPoseGroup_skel->hpose->pose->scale,
		demoMode->hierarchyPoseGroup_skel->hpose->pose->translation,
		demoMode->hierarchy_skel->numNodes);
		break;
	case 2: a3hierarchyPoseOpCopy(activeHS->objectSpace, blendControls, inputParams, demoMode->hierarchy_skel->numNodes);
		break;
	case 3: a3hierarchyPoseOpInvert(activeHS->objectSpace, blendControls, inputParams, demoMode->hierarchy_skel->numNodes);
		break;
	case 4: a3hierarchyPoseOpConcat(activeHS->objectSpace, blendControls, inputParams, demoMode->hierarchy_skel->numNodes);
		break;
	case 5: a3hierarchyPoseOpNearest(activeHS->objectSpace, blendControls, inputParams, demoMode->hierarchy_skel->numNodes);
		break;
	case 6: 
		inputParams[0] = &demoMode->clipController[0].clipParam;
		a3hierarchyPoseClipCtrlOpLerp(activeHS->objectSpace, demoMode->clipController, demoMode->clipController + 1, inputParams, demoMode->hierarchy_skel->numNodes);
		break;
	case 7: a3hierarchyPoseOpCubic(activeHS->objectSpace, blendControls, inputParams, demoMode->hierarchy_skel->numNodes);
		break;
	case 8: a3hierarchyPoseOpSplit(activeHS->objectSpace, blendControls, inputParams, demoMode->hierarchy_skel->numNodes);
		break;
	case 9: a3hierarchyPoseOpScale(activeHS->objectSpace, blendControls, inputParams, demoMode->hierarchy_skel->numNodes);
		break;
	case 10: a3hierarchyPoseOpTriangular(activeHS->objectSpace, blendControls, inputParams, demoMode->hierarchy_skel->numNodes);
		break;
	case 11: a3hierarchyPoseOpBiNearest(activeHS->objectSpace, blendControls, inputParams, demoMode->hierarchy_skel->numNodes);
		break;
	case 12: a3hierarchyPoseOpBiLerp(activeHS->objectSpace, blendControls, inputParams, demoMode->hierarchy_skel->numNodes);
		break;
	case 13: a3hierarchyPoseOpBiCubic(activeHS->objectSpace, blendControls, inputParams, demoMode->hierarchy_skel->numNodes);
		break;
	default:											  
		break;
	}
	blendControls[0] = baseHS->localSpace;
	blendControls[1] = activeHS->objectSpace;
	a3hierarchyPoseOpConcat(activeHS->localSpace, blendControls, inputParams, demoMode->hierarchy_skel->numNodes);
	a3hierarchyPoseConvert(activeHS->localSpace,
		demoMode->hierarchy_skel->numNodes,
		demoMode->hierarchyPoseGroup_skel->channel,
		demoMode->hierarchyPoseGroup_skel->order);

	a3kinematicsSolveForward(activeHS);
	a3hierarchyStateUpdateObjectInverse(activeHS);
	a3hierarchyStateUpdateObjectBindToCurrent(activeHS, baseHS);

	blendControls[1] = controlHS0->objectSpace;
	a3hierarchyPoseOpConcat(controlHS0->localSpace, blendControls, inputParams, demoMode->hierarchy_skel->numNodes);
	a3hierarchyPoseConvert(controlHS0->localSpace,
		demoMode->hierarchy_skel->numNodes,
		demoMode->hierarchyPoseGroup_skel->channel,
		demoMode->hierarchyPoseGroup_skel->order);

	a3kinematicsSolveForward(controlHS0);
	a3hierarchyStateUpdateObjectInverse(controlHS0);
	a3hierarchyStateUpdateObjectBindToCurrent(controlHS0, baseHS);

	blendControls[1] = controlHS1->objectSpace;
	a3hierarchyPoseOpConcat(controlHS1->localSpace, blendControls, inputParams, demoMode->hierarchy_skel->numNodes);
	a3hierarchyPoseConvert(controlHS1->localSpace,
		demoMode->hierarchy_skel->numNodes,
		demoMode->hierarchyPoseGroup_skel->channel,
		demoMode->hierarchyPoseGroup_skel->order);

	a3kinematicsSolveForward(controlHS1);
	a3hierarchyStateUpdateObjectInverse(controlHS1);
	a3hierarchyStateUpdateObjectBindToCurrent(controlHS1, baseHS);


	// prepare and upload graphics data
	for (a3i32 loop = 1; loop < 4; loop++)
	{
		activeHS = demoMode->hierarchyState_skel + loop;
		a3addressdiff const skeletonIndex = demoMode->obj_skeleton - demoMode->object_scene;
		a3ui32 const mvp_size = demoMode->hierarchy_skel->numNodes * sizeof(a3mat4);
		a3ui32 const t_skin_size = sizeof(demoMode->t_skin);
		a3ui32 const dq_skin_size = sizeof(demoMode->dq_skin);
		a3mat4 const mvp_obj = matrixStack[skeletonIndex].modelViewProjectionMat;
		a3mat4* mvp_joint, * mvp_bone, * t_skin;
		a3dualquat* dq_skin;
		a3index i;
		a3i32 p;
		
		// update joint and bone transforms
		for (i = 0; i < demoMode->hierarchy_skel->numNodes; ++i)
		{
			mvp_joint = demoMode->mvp_joint + i;
			mvp_bone = demoMode->mvp_bone + i;
			t_skin = demoMode->t_skin + i;
			dq_skin = demoMode->dq_skin + i;
		
			// joint transform
			a3real4x4SetScale(scaleMat.m, a3real_quarter);
			a3real4x4Concat(activeHS->objectSpace->pose[i].transform.m, scaleMat.m);
			a3real4x4Product(mvp_joint->m, mvp_obj.m, scaleMat.m);
			
			// bone transform
			p = demoMode->hierarchy_skel->nodes[i].parentIndex;
			if (p >= 0)
			{
				// position is parent joint's position
				scaleMat.v3 = activeHS->objectSpace->pose[p].transform.v3;

				// direction basis is from parent to current
				a3real3Diff(scaleMat.v2.v,
					activeHS->objectSpace->pose[i].transform.v3.v, scaleMat.v3.v);

				// right basis is cross of some upward vector and direction
				// select 'z' for up if either of the other dimensions is set
				a3real3MulS(a3real3CrossUnit(scaleMat.v0.v,
					a3real2LengthSquared(scaleMat.v2.v) > a3real_zero
					? a3vec3_z.v : a3vec3_y.v, scaleMat.v2.v), a3real_quarter);
			
				// up basis is cross of direction and right
				a3real3MulS(a3real3CrossUnit(scaleMat.v1.v,
					scaleMat.v2.v, scaleMat.v0.v), a3real_quarter);
			}
			else
			{
				// if we are a root joint, make bone invisible
				a3real4x4SetScale(scaleMat.m, a3real_zero);
			}
			a3real4x4Product(mvp_bone->m, mvp_obj.m, scaleMat.m);

			// get base to current object-space
			*t_skin = activeHS->objectSpaceBindToCurrent->pose[i].transform;
		
			// calculate DQ
			{
				a3real4 d = { a3real_zero };
				a3demo_mat2quat_safe(dq_skin->r.q, t_skin->m);
				a3real3ProductS(d, t_skin->v3.v, a3real_half);
				a3quatProduct(dq_skin->d.q, d, dq_skin->r.q);
			}


			a3bufferRefill(demoState->ubo_transformMVP + loop - 1, 0, mvp_size, demoMode->mvp_joint);
			a3bufferRefill(demoState->ubo_transformMVPB + loop - 1, 0, mvp_size, demoMode->mvp_bone);
		}
		
		// upload
		//a3bufferRefill(demoState->ubo_transformMVP, 0, mvp_size, demoMode->mvp_joint);
		//a3bufferRefill(demoState->ubo_transformMVPB, 0, mvp_size, demoMode->mvp_bone);
		a3bufferRefill(demoState->ubo_transformBlend, 0, t_skin_size, demoMode->t_skin);
		a3bufferRefillOffset(demoState->ubo_transformBlend, 0, t_skin_size, dq_skin_size, demoMode->dq_skin);


		//a3bufferRefillOffset(demoState->ubo_transformMVP, 0, mvp_size, mvp_size, demoMode->mvp_joint);
		//a3bufferRefillOffset(demoState->ubo_transformMVPB, 0, mvp_size, mvp_size, demoMode->mvp_bone);

		//a3bufferRefillOffset(demoState->ubo_transformMVP, 0, mvp_size*2, mvp_size, demoMode->mvp_joint);
		//a3bufferRefillOffset(demoState->ubo_transformMVPB, 0, mvp_size*2, mvp_size, demoMode->mvp_bone);
	}
}


//-----------------------------------------------------------------------------
