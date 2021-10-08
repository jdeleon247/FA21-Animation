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
	
	a3_Kinematics.c
	Implementation of kinematics solvers.
*/

#include "../a3_Kinematics.h"


//-----------------------------------------------------------------------------

// partial FK solver
a3i32 a3kinematicsSolveForwardPartial(const a3_HierarchyState *hierarchyState, const a3ui32 firstIndex, const a3ui32 nodeCount)
{
	if (hierarchyState && hierarchyState->hierarchy && 
		firstIndex < hierarchyState->hierarchy->numNodes && nodeCount)
	{
		// ****TO-DO: implement forward kinematics algorithm
		//	- for all nodes starting at first index
		//		- if node is not root (has parent node)
		//			- object matrix = parent object matrix * local matrix
		//		- else
		//			- copy local matrix to object matrix


		//Done?
		a3ui32 i = firstIndex;
		a3ui32 p = 0;

		for (i; i < nodeCount; i++)
		{
			//get parent index of current
			p = hierarchyState->hierarchy->nodes[i].parentIndex;

			// not root
			//if (p < i)
			//{
			//	// object matrix = parent object matrix * local matrix
			//	a3real4x4Product(hierarchyState->objectSpacePose->spatialPose[i].transform.m, hierarchyState->objectSpacePose->spatialPose[p].transform.m, hierarchyState->localSpacePose->spatialPose[i].transform.m);
			//}
			//// root
			//else
			//{	// copy local matrix to object matrix
			//	hierarchyState->objectSpacePose->spatialPose[i].transform = hierarchyState->localSpacePose->spatialPose[i].transform;
			//}

			// not root
			if (p == -1)
			{
				// copy local matrix to object matrix
				hierarchyState->objectSpacePose->spatialPose[i].transform = hierarchyState->localSpacePose->spatialPose[i].transform;
			}
			// root
			else
			{	
				// object matrix = parent object matrix * local matrix
				a3real4x4Product(hierarchyState->objectSpacePose->spatialPose[i].transform.m, hierarchyState->objectSpacePose->spatialPose[p].transform.m, hierarchyState->localSpacePose->spatialPose[i].transform.m);
			}
		}
		return 1;
	}
	return -1;
}


//-----------------------------------------------------------------------------

// partial IK solver
a3i32 a3kinematicsSolveInversePartial(const a3_HierarchyState *hierarchyState, const a3ui32 firstIndex, const a3ui32 nodeCount)
{
	if (hierarchyState && hierarchyState->hierarchy &&
		firstIndex < hierarchyState->hierarchy->numNodes && nodeCount)
	{
		// ****TO-DO: implement inverse kinematics algorithm
		//	- for all nodes starting at first index
		//		- if node is not root (has parent node)
		//			- local matrix = inverse parent object matrix * object matrix
		//		- else
		//			- copy object matrix to local matrix
	}
	return -1;
}


//-----------------------------------------------------------------------------
