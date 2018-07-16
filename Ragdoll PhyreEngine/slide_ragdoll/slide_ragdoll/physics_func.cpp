/* SIE CONFIDENTIAL
 * PlayStation(R)4 Programmer Tool Runtime Library Release 05.008.001
 *                Copyright (C) 2017 Sony Interactive Entertainment Inc.
 *                                                All Rights Reserved.
 */

///////////////////////////////////////////////////////////////////////////////
// Include Headers

#include "physics_func.h"
#include "../../api_physics_effects/common/render_func.h"
#include "../../api_physics_effects/common/create_render_mesh.h"
#include "../ragdoll_common/pfx_ragdoll.h"
#include "../ragdoll_common/ragdoll_control.h"
#include <new>
#include "input.h"
using namespace sce::SampleUtil::Input;

///////////////////////////////////////////////////////////////////////////////
// Simulation Data

static int pickJointId = -1;
static int sSceneId;
static float timeStep = 0.016f;

//x,y,z for ragdoll force
int x = 1.0f;
int y = 2.0f;
int z = 3.0f;

//J プールメモリ
//E Pool memory
static PfxUInt32 poolBytes;
static void *poolBuff;

//E World
PfxRigidBodyWorld *world;

//J ラージメッシュ
//E Large mesh
#include "environments.h"
static PfxArray<PfxLargeTriMesh*>	*largeMeshes;
static PfxArray<PfxConvexMesh*>		*convexMeshes;

//J 描画メッシュ
//E Render meshes
static PfxMap<void*,int> *renderMeshes;

///////////////////////////////////////////////////////////////////////////////
// Ragdoll controll and animation

PfxBool gPickingRagdoll;

///////////////////////////////////////////////////////////////////////////////
// Create Scene

int createBrick(const PfxVector3 &pos, const PfxQuat &rot, const PfxVector3 &boxSize, PfxFloat mass)
{
	PfxRigidState state;
	PfxRigidBody body;
	PfxCollidable collidable;

	pfxCalcMassCylinder;

	PfxBox box(boxSize);
	PfxShape shape;
	shape.reset();
	shape.setBox(box);
	collidable.reset();
	collidable.addShape(shape);
	collidable.finish();
	body.reset();
	body.setRestitution(0.0f);
	body.setMass(mass);
	body.setInertia(pfxCalcInertiaBox(boxSize, mass));
	state.reset();
	state.setPosition(pos);
	state.setOrientation(rot);
	state.setMotionType(kPfxMotionTypeActive);
	state.setUseSleep(1);

	return world->addRigidBody(state, body, collidable);
}

void createPyramid(const PfxVector3 &offsetPosition, int stackSize, const PfxVector3 &boxSize)
{
//	PfxFloat space = 0.0001f;
//	PfxVector3 pos(1.0f, 1.0f /*boxSize[1]*/, 1.0f);
//
//	PfxFloat diffX = boxSize[0] * 1.02f;
//	PfxFloat diffY = boxSize[1] * 1.02f;
//	PfxFloat diffZ = boxSize[2] * 1.02f;
//
//	PfxFloat offsetX = -stackSize * (diffX * 2.0f + space) * 0.5f;
//	PfxFloat offsetZ = -stackSize * (diffZ * 2.0f + space) * 0.5f;
//	while (stackSize) {
//		for (int j = 0; j<stackSize; j++) {
//			pos[2] = offsetZ + (PfxFloat)j * (diffZ * 2.0f + space);
//			for (int i = 0; i<stackSize; i++) {
//				pos[0] = offsetX + (PfxFloat)i * (diffX * 2.0f + space);
//				createBrick(offsetPosition + pos, PfxQuat::identity(), boxSize, 5.0f);
//			}
//		}
//		offsetX += diffX;
//		offsetZ += diffZ;
//		pos[1] += (diffY * 2.0f + space);
//		stackSize--;
//	}
////	physicsThrowCubes();
//
}
#include <camera.h>
void createTowerCircle(const PfxVector3 &offsetPosition, int stackSize, int rotSize, const PfxVector3 &boxSize)
{
	//PfxFloat radius = 1.3f * rotSize * boxSize[0] / SCE_PFX_PI;

	//// create active boxes
	//PfxQuat rotY = PfxQuat::identity();
	//PfxFloat posY = boxSize[1];

	//for (int i = 0;i<stackSize;i++) {
	//	for (int j = 0;j<rotSize;j++) {
	//		createBrick(offsetPosition + rotate(rotY, PfxVector3(0.0f, posY, radius)), rotY, boxSize, 5.0f);

	//		rotY *= PfxQuat::rotationY(SCE_PFX_PI / (rotSize*0.5f));
	//	}

	//	posY += boxSize[1] * 2.0f;
	//	rotY *= PfxQuat::rotationY(SCE_PFX_PI / (PfxFloat)rotSize);
	//}
	////physicsThrowCubes();


	// create active boxes
	PfxQuat rotY = PfxQuat::identity();
	PfxFloat posY = boxSize[1];
	PfxFloat posX = 1.0f;

	for (int i = 0;i<101;i++) {
		for (int j = 0;j<rotSize;j++) {
			createBrick((rotY, PfxVector3(0.0f, posY, posX)), rotY, boxSize, 1.0f);	//10
			//world->getRigidBody.m_active=0;
		}

		posY += boxSize[1] * 5.0f;
		rotY *= PfxQuat::rotationY(SCE_PFX_PI / (PfxFloat)rotSize);
	}
	
	//int sceCameraGetFrameData()

	//physicsThrowCubes();
}

void createSceneBoxGround()
{
	PfxRigidState state;
	PfxRigidBody body;
	PfxCollidable collidable;
	
	PfxBox box(250.0f,5.5f,250.0f);
	PfxShape shape;
	shape.reset();
	shape.setBox(box);
	collidable.reset();
	collidable.addShape(shape);
	collidable.finish();
	body.reset();
	state.reset();
	state.setPosition(PfxVector3(0.0f,-3.0f,0.0f));
	state.setMotionType(kPfxMotionTypeFixed);

	world->addRigidBody(state,body,collidable);
}

void createSceneLandscape(const PfxVector3 &pos)
{
	PfxLargeTriMesh* pLargeMesh = (PfxLargeTriMesh*)SCE_PFX_UTIL_ALLOC(16,sizeof(PfxLargeTriMesh));
	SCE_PFX_ALWAYS_ASSERT(pLargeMesh);
	memset(pLargeMesh,0,sizeof(PfxLargeTriMesh));
	
	PfxCreateLargeTriMeshParam param;
	
	param.verts = Env2Vtx;
	param.numVerts = Env2VtxCount;
	param.vertexStrideBytes = sizeof(float)*8;
	
	param.triangles = Env2Idx;
	param.numTriangles = Env2IdxCount/3;
	param.triangleStrideBytes = sizeof(unsigned int)*3;
	
	param.userData = Env2Col;
	
	param.flag = SCE_PFX_MESH_FLAG_32BIT_INDEX | SCE_PFX_MESH_FLAG_USE_QUANTIZED | SCE_PFX_MESH_FLAG_USE_BVH | SCE_PFX_MESH_FLAG_OUTPUT_INFO;
	
	PfxInt32 ret = pfxCreateLargeTriMesh(*pLargeMesh,param);
	if(ret != SCE_PFX_OK) {
		SCE_PFX_PRINTF("Can't create large mesh. err = 0x%x\n",ret);
		SCE_PFX_ALWAYS_ASSERT(ret == SCE_PFX_OK);
	}
	
	PfxRigidState state;
	PfxRigidBody body;
	PfxCollidable collidable;
	
	PfxShape shape;
	shape.reset();
	shape.setLargeTriMesh(pLargeMesh);
	collidable.reset();
	collidable.addShape(shape);
	collidable.finish();
	body.reset();
	body.setFriction(0.2f);
	body.setRestitution(0.0f);
	state.reset();
	state.setPosition(pos);
	state.setOrientation(PfxQuat::rotationY(SCE_PFX_PI));
	state.setMotionType(kPfxMotionTypeFixed);
	world->addRigidBody(state,body,collidable);
	
	largeMeshes->push(pLargeMesh);
	
	int renderMeshId = renderInitMesh(
		Env2Vtx, sizeof(float) * 8,
		Env2Vtx + 3, sizeof(float) * 8,
		Env2Idx, sizeof(unsigned int) * 3,
		Env2VtxCount, Env2IdxCount/3);

	renderMeshes->insert(pLargeMesh,renderMeshId);
}

///////////////////////////////////////////////////////////////////////////////
// Ragdoll behavior

#define RAGDOLL_BEHAVIOR_RESULT_NONE	0
#define RAGDOLL_BEHAVIOR_RESULT_PUSH	1
#define RAGDOLL_BEHAVIOR_RESULT_ROLL	2
#define RAGDOLL_BEHAVIOR_RESULT_SLIDE	3

int execRagdollBehavior()
{
	int pushCount = 0;
	int slideCount = 0;
	PfxVector3 averageNormalForPush(0.0f);
	PfxVector3 averageNormalForSlide(0.0f);

	// Check if the trigger hits facets with special behavior flags
	// Note that it uses the previous result of contacts
	PfxUInt32 triggerId = Ragdoll::getTriggerId();
	const PfxQueryContactInfo *q = world->queryFirstContact(triggerId);
	for (; q != NULL; q = world->queryNextContact(q, triggerId)) {
		for (int i = 0; i < q->numContactPoints; i++) {
			const PfxQueryContactPoint &cp = q->contactPoints[i];
			if (q->rigidbodyIdA == triggerId) {
				PfxSubData subData = cp.subDataB;
				if (subData.getType() == PfxSubData::MESH_INFO) {
					PfxUInt32 userData = subData.getUserData();
					SCE_PFX_PRINTF("hit RB %u island %u facet %u data 0x%x\n", q->rigidbodyIdB, subData.getIslandId(), subData.getFacetId(), userData);
					if (userData == 0x0000FF) {
						averageNormalForPush += cp.normal;
						pushCount++;
					}
					else if (userData == 0xFF0000) {
						averageNormalForSlide += cp.normal;
						slideCount++;
					}
				}
			}
			else {
				PfxSubData subData = cp.subDataA;
				if (subData.getType() == PfxSubData::MESH_INFO) {
					PfxUInt32 userData = subData.getUserData();
					SCE_PFX_PRINTF("hit RB %u island %u facet %u data 0x%x\n", q->rigidbodyIdA, subData.getIslandId(), subData.getFacetId(), userData);
					if (userData == 0x0000FF) {
						averageNormalForPush -= cp.normal;
						pushCount++;
					}
					else if (userData == 0xFF0000) {
						averageNormalForSlide -= cp.normal;
						slideCount++;
					}
				}
			}
		}
	}

	int result = RAGDOLL_BEHAVIOR_RESULT_NONE;

	if (pushCount > 0) {
		Ragdoll::pushRagdoll(averageNormalForPush / (float)pushCount, PfxVector3(0.0f, 0.0f, 0.0f), PfxVector3(0.0f), 1.0f, 0.0f);
		result = RAGDOLL_BEHAVIOR_RESULT_PUSH;
	}

	if (slideCount > 0) {
		Ragdoll::slideRagdoll(averageNormalForSlide / (float)slideCount, PfxVector3(0.0f), PfxVector3(0.0f), 3.0f, 3.0f, 10.0f);
		result = RAGDOLL_BEHAVIOR_RESULT_SLIDE;
	}

	if (pushCount == 0 && slideCount == 0) {
		const PfxRigidState &rootState = Ragdoll::getRootRigidStateInRagdoll();
		PfxFloat energy = lengthSqr(rootState.getLinearVelocity()) + lengthSqr(rootState.getAngularVelocity());
		if (energy > 0.1f && energy < 5.0f) {
			Ragdoll::rollRagdollFaceUpOrFaceDown(10.0f, 1.0f);
			result = RAGDOLL_BEHAVIOR_RESULT_ROLL;
		}
	}

	return result;
}

///////////////////////////////////////////////////////////////////////////////
// Ragdoll state control

void changeRagdollState(Ragdoll::eRagdollState ragdollState)
{
	Ragdoll::RagdollState &currRagdollState = Ragdoll::getCurrentRagdollState();

	currRagdollState.passedTime = 0.0f;
	currRagdollState.animationTime = 0.0f;
	currRagdollState.ragdollState = ragdollState;
	
	Ragdoll::restoreRootWeight();
	
	switch (ragdollState) {
		case Ragdoll::RAGDOLL_STATE_IDLE:
		SCE_PFX_PRINTF("change Idle\n");
		currRagdollState.currentAnimation = Ragdoll::getAnimation(Ragdoll::ANIM_IDLE);
		currRagdollState.animationDuration = currRagdollState.currentAnimation->duration;
		currRagdollState.motorRatio = 1.0f;
		Ragdoll::increaseRootWeight();
		Ragdoll::resetRagdollOrigin();
		Ragdoll::correctAnimationOffset();
		break;
		
		case Ragdoll::RAGDOLL_STATE_FLYBY:
		SCE_PFX_PRINTF("change Flyby\n");
		currRagdollState.currentAnimation = Ragdoll::getAnimation(Ragdoll::ANIM_CLING);
		currRagdollState.animationDuration = currRagdollState.currentAnimation->duration;
		currRagdollState.motorRatio = 1.0f;
		break;
		
		case Ragdoll::RAGDOLL_STATE_TRANSITION:
		SCE_PFX_PRINTF("change Transition\n");
		if (currRagdollState.nextState == Ragdoll::RAGDOLL_STATE_GETUP_UP) {
			currRagdollState.currentAnimation = Ragdoll::getAnimation(Ragdoll::ANIM_GETUP_UP);
		}
		else {
			currRagdollState.currentAnimation = Ragdoll::getAnimation(Ragdoll::ANIM_GETUP_DOWN);
		}
		currRagdollState.animationDuration = currRagdollState.currentAnimation->duration;
		currRagdollState.motorRatio = 0.0f;
		Ragdoll::increaseRootWeight();
		Ragdoll::resetRagdollOrigin();
		Ragdoll::correctAnimationOffset();
		break;
		
		case Ragdoll::RAGDOLL_STATE_GETUP_UP:
		SCE_PFX_PRINTF("change GetUpUp\n");
		currRagdollState.currentAnimation = Ragdoll::getAnimation(Ragdoll::ANIM_GETUP_UP);
		currRagdollState.animationDuration = currRagdollState.currentAnimation->duration;
		currRagdollState.motorRatio = 0.0f;
		break;
		
		case Ragdoll::RAGDOLL_STATE_GETUP_DOWN:
		SCE_PFX_PRINTF("change GetUpDown\n");
		currRagdollState.currentAnimation = Ragdoll::getAnimation(Ragdoll::ANIM_GETUP_DOWN);
		currRagdollState.animationDuration = currRagdollState.currentAnimation->duration;
		currRagdollState.motorRatio = 0.0f;
		break;
	}
	
	Ragdoll::changeMotorRatio(currRagdollState.motorRatio);
}

void processRagdollState()
{
	Ragdoll::RagdollState &currRagdollState = Ragdoll::getCurrentRagdollState();

	switch (currRagdollState.ragdollState) {
		case Ragdoll::RAGDOLL_STATE_IDLE:
		Ragdoll::moveRagdollRootByAnimation(timeStep);
		currRagdollState.animationTime += timeStep;
		if (currRagdollState.animationTime > currRagdollState.animationDuration) currRagdollState.animationTime = 0.0f;
		break;
		
		case Ragdoll::RAGDOLL_STATE_FLYBY:
		{
			//SCE_PFX_PRINTF("process Flyby : motor %f anim %f / %f\n",
			//	currRagdollState.motorRatio, currRagdollState.animationTime, currRagdollState.animationDuration);
			const PfxFloat duration = 0.1f;
			Ragdoll::changeMotorRatio(currRagdollState.motorRatio);
			if(!gPickingRagdoll) {
				currRagdollState.motorRatio -= 0.05;
				if (currRagdollState.motorRatio < 0.2f) currRagdollState.motorRatio = 0.2f;
			}
			else {
				currRagdollState.motorRatio = 1.0f;
			}

			if (!gPickingRagdoll) {
				int result = execRagdollBehavior();
				if (result == RAGDOLL_BEHAVIOR_RESULT_PUSH || result == RAGDOLL_BEHAVIOR_RESULT_SLIDE) {
					Ragdoll::increaseRootWeight();
					currRagdollState.motorRatio = 1.0f;
					currRagdollState.passedTime = 0.0f;
				}
			}

			// check if the root remains still.
			const PfxRigidState &rootState = Ragdoll::getRootRigidStateInRagdoll();
			PfxFloat energy = lengthSqr(rootState.getLinearVelocity()) + lengthSqr(rootState.getAngularVelocity());
			if (energy < 0.05f && currRagdollState.passedTime > duration && !gPickingRagdoll) {
				PfxMatrix3 rootOri(rootState.getOrientation());
				if (dot(PfxVector3(0.0f, 1.0f, 0.0f), rootOri.getCol2()) > 0.0f) {
					currRagdollState.nextState = Ragdoll::RAGDOLL_STATE_GETUP_DOWN;
					changeRagdollState(Ragdoll::RAGDOLL_STATE_TRANSITION);
				}
				else {
					currRagdollState.nextState = Ragdoll::RAGDOLL_STATE_GETUP_UP;
					changeRagdollState(Ragdoll::RAGDOLL_STATE_TRANSITION);
				}
				return;
			}
			currRagdollState.animationTime += timeStep;
			if (currRagdollState.animationTime > currRagdollState.animationDuration) currRagdollState.animationTime = 0.0f;
		}
		break;
		
		case Ragdoll::RAGDOLL_STATE_TRANSITION:
		{
			const PfxFloat duration = 0.5f; // sec
			//SCE_PFX_PRINTF("process Transition : motor %f anim %f / %f\n",
			//	currRagdollState.motorRatio, currRagdollState.animationTime, currRagdollState.animationDuration);
			
			if (currRagdollState.passedTime > 0.0f) {
				Ragdoll::moveRagdollRootByAnimation(timeStep * duration / currRagdollState.passedTime);
			}
			Ragdoll::changeMotorRatio(currRagdollState.motorRatio);
			currRagdollState.motorRatio += 0.05f;
			if(currRagdollState.motorRatio > 1.0f) currRagdollState.motorRatio = 1.0f;
			if(currRagdollState.passedTime > duration) {
				changeRagdollState(currRagdollState.nextState);
			}
		}
		break;
		
		case Ragdoll::RAGDOLL_STATE_GETUP_UP:
		case Ragdoll::RAGDOLL_STATE_GETUP_DOWN:
		{
			//SCE_PFX_PRINTF("process %s : motor %f anim %f / %f\n",
			//	(currRagdollState.ragdollState == Ragdoll::RAGDOLL_STATE_GETUP_UP ? "GetUpUp" : "GetUpDown"),
			//	currRagdollState.motorRatio, currRagdollState.animationTime, currRagdollState.animationDuration);

			Ragdoll::moveRagdollRootByAnimation(timeStep);
			Ragdoll::changeMotorRatio(currRagdollState.motorRatio);
			currRagdollState.motorRatio += 0.05f;
			if(currRagdollState.motorRatio > 1.0f) currRagdollState.motorRatio = 1.0f;
			if(currRagdollState.animationTime > currRagdollState.animationDuration) {
				changeRagdollState(Ragdoll::RAGDOLL_STATE_IDLE);
				return;
			}
			currRagdollState.animationTime += timeStep;
		}
		break;
	}
	
	currRagdollState.passedTime += timeStep;
}

///////////////////////////////////////////////////////////////////////////////
// Create scene

void physicsCreateScene(int sceneId)
{
	const int numScenes = 1;
	sSceneId = sceneId % numScenes;
	
	physicsResetWorld();
	
	gPickingRagdoll = false;
	
	createSceneLandscape(PfxVector3(0.0f, 0.0f, 0.0f));
	//createPyramid(PfxVector3(2.5f, 0.0f, -2.5f), 5, PfxVector3(0.25f, 0.25f, 0.25f));
	//createPyramid(PfxVector3(-2.5f, 0.0f, 2.5f), 5, PfxVector3(0.25f, 0.25f, 0.25f));
	createTowerCircle(PfxVector3(-1.5f, 0.0f, 1.5f), 10, 2, PfxVector3(0.25f, 0.25f, 0.25f));

	Ragdoll::createRagdollFromSkeleton();
	changeRagdollState(Ragdoll::RAGDOLL_STATE_IDLE); 
	
	int n = world->getRigidBodyCount();
	int j = world->getJointCount();

	SCE_PFX_PRINTF("PfxRigidBodyWorld %5lu bytes\n",sizeof(PfxRigidBodyWorld));

	SCE_PFX_PRINTF("----- Size of rigid body buffer ------\n");
	SCE_PFX_PRINTF("                  size * num = total\n");
	SCE_PFX_PRINTF("PfxRigidState      %5lu * %5d = %5lu bytes\n",sizeof(PfxRigidState),n,sizeof(PfxRigidState)*n);
	SCE_PFX_PRINTF("PfxRigidBody       %5lu * %5d = %5lu bytes\n",sizeof(PfxRigidBody),n,sizeof(PfxRigidBody)*n);
	SCE_PFX_PRINTF("PfxCollidable      %5lu * %5d = %5lu bytes\n",sizeof(PfxCollidable),n,sizeof(PfxCollidable)*n);
	SCE_PFX_PRINTF("PfxJoint           %5lu * %5d = %5lu bytes\n",sizeof(PfxJoint),j,sizeof(PfxJoint)*j);
	SCE_PFX_PRINTF("PfxSolverBody      %5lu * %5d = %5lu bytes\n",sizeof(PfxSolverBody),n,sizeof(PfxSolverBody)*n);
	SCE_PFX_PRINTF("PfxContactManifold %5lu * %5d = %5lu bytes\n",sizeof(PfxContactManifold),world->getContactCapacity(),sizeof(PfxContactManifold)*world->getContactCapacity());
	SCE_PFX_PRINTF("PfxBroadphasePair  %5lu * %5d = %5lu bytes\n",sizeof(PfxBroadphasePair),world->getContactCapacity(),sizeof(PfxBroadphasePair)*world->getContactCapacity());

	int totalBytes = 
		(sizeof(PfxRigidState) + sizeof(PfxRigidBody) + sizeof(PfxCollidable) + sizeof(PfxSolverBody)) * n +
		(sizeof(PfxContactManifold) + sizeof(PfxBroadphasePair)) * world->getContactCapacity();
	SCE_PFX_PRINTF("----------------------------------------------------------\n");
	SCE_PFX_PRINTF("Total %5d bytes\n",totalBytes);
}

///////////////////////////////////////////////////////////////////////////////
// Initialize / Finalize Engine

static void *largeMeshesBuff = NULL;
static void *convexMeshesBuff = NULL;
static void *renderMeshesBuff = NULL;
static void *largeCompoundsBuff = NULL;
static void *worldBuff = NULL;

bool physicsInit()
{
	largeMeshesBuff = SCE_PFX_UTIL_ALLOC(16,sizeof(PfxArray<PfxLargeTriMesh*>));
	SCE_PFX_ALWAYS_ASSERT(largeMeshesBuff);
	convexMeshesBuff = SCE_PFX_UTIL_ALLOC(16,sizeof(PfxArray<PfxConvexMesh*>));
	SCE_PFX_ALWAYS_ASSERT(convexMeshesBuff);
	largeCompoundsBuff = SCE_PFX_UTIL_ALLOC(16,sizeof(PfxArray<PfxLargeCompound*>));
	SCE_PFX_ALWAYS_ASSERT(largeCompoundsBuff);
	renderMeshesBuff = SCE_PFX_UTIL_ALLOC(16,sizeof(PfxMap<void*,int>));
	SCE_PFX_ALWAYS_ASSERT(renderMeshesBuff);
	
	largeMeshes	= new(largeMeshesBuff) PfxArray<PfxLargeTriMesh*>;
	convexMeshes = new(convexMeshesBuff) PfxArray<PfxConvexMesh*>;
	renderMeshes = new(renderMeshesBuff) PfxMap<void*,int>(largeMeshes->capacity()+convexMeshes->capacity());

	//J 物理シーンを管理するPfxRigidBodyWorldインスタンスを作成します。シーンへのアクセスは全てPfxRigidBodyWorldを通して行います。
	//E Create the PfxRigidBodyWorld instance that manages a physics scene. Accessing a physics scene is processed via PfxRigidBodyWorld.

	PfxRigidBodyWorldParam worldParam;

	//J マルチスレッドで演算するステージをmultiThreadFlagに、スレッド数をnumWorkerThreadsにセットします。
	//E Set multi-threaded stages to multiThreadFlag and the number of threads to numWorkerThreads.
	worldParam.multiThreadFlag = SCE_PFX_ENABLE_MULTITHREAD_ALL;
	worldParam.numWorkerThreads = 3;

	//J マルチスレッドを使用するときは多めにコンテキスト用バッファを指定します。
	//E Multi-threaded calculation requires more context buffer than single-threaded.
	worldParam.contextWorkBytes = 4 * 1024 * 1024;
	worldParam.ccdWorkBytes = 1 * 1024 * 1024;

	//J スリープ機能を有効化するためにはsimulationFlagにSCE_PFX_ENABLE_SLEEPをセットしてください。
	//E To enable sleep (deactivation) function set SCE_PFX_ENABLE_SLEEP to simulationFlag.
	worldParam.simulationFlag = SCE_PFX_ENABLE_SLEEP | SCE_PFX_ENABLE_CONTACT_CACHE;

	worldParam.worldMin = PfxLargePosition(PfxVector3(-250.0f));
	worldParam.worldMax = PfxLargePosition(PfxVector3(250.0f));

	worldParam.maxNonContactPairs = 300;

	//J ワールドパラメータに記述されたシーンの規模からバイトサイズを計算し、メモリを確保します。
	//E Calculate byte size from a scene scale described in a world parameter and allocate memory for pool buffer.
	poolBytes = PfxRigidBodyWorld::getRigidBodyWorldBytes(worldParam);
	poolBuff = SCE_PFX_UTIL_ALLOC(16,poolBytes);
	SCE_PFX_ALWAYS_ASSERT(poolBuff);
	
	SCE_PFX_PRINTF("pool %d bytes\n",poolBytes);

	worldParam.poolBytes = poolBytes;
	worldParam.poolBuff = poolBuff;
	
	worldParam.velocityIteration[0] = 3;
	worldParam.velocityIteration[1] = 6;
	worldParam.velocityIteration[2] = 10;

	worldParam.timeStep = timeStep;
	//worldParam.gravity = PfxVector3(0.0f);

	//J PfxRigidBodyWorldクラスのインスタンスを作成し、ワールドを初期化します。
	//E Create an instance of PfxRigidBodyWorld and initialize a world
	worldBuff = SCE_PFX_UTIL_ALLOC(16,sizeof(PfxRigidBodyWorld));
	world = new(worldBuff) PfxRigidBodyWorld(worldParam);
	SCE_PFX_ALWAYS_ASSERT(world);

	world->initialize();
	
	Ragdoll::initializeRagdollAndAnimation(world);
	
	return true;
}

void physicsReleaseMeshes()
{
	for (PfxUInt32 c=0; c<largeMeshes->size(); ++c)
	{
		pfxReleaseLargeTriMesh(*(*largeMeshes)[c]);
		SCE_PFX_UTIL_FREE((*largeMeshes)[c]);
	}
	largeMeshes->clear();

	for (PfxUInt32 c=0; c<convexMeshes->size(); ++c)
	{
		pfxReleaseConvexMesh(*(*convexMeshes)[c]);
		SCE_PFX_UTIL_FREE((*convexMeshes)[c]);
	}
	convexMeshes->clear();
	
	renderMeshes->clear();
}

void physicsResetWorld()
{
	physicsReleaseMeshes();

	if(world) {
		world->reset();
	}

	pickJointId = -1;
}

void physicsRelease()
{
	world->finalize();

	physicsResetWorld();

	world->~PfxRigidBodyWorld();
	SCE_PFX_UTIL_FREE(worldBuff);
	SCE_PFX_UTIL_FREE(poolBuff);

	renderMeshes->~PfxMap<void*,int>();
	SCE_PFX_UTIL_FREE(renderMeshesBuff);
	largeMeshes->~PfxArray<PfxLargeTriMesh*>();
	SCE_PFX_UTIL_FREE(largeMeshesBuff);
	convexMeshes->~PfxArray<PfxConvexMesh*>();
	SCE_PFX_UTIL_FREE(convexMeshesBuff);

	Ragdoll::finalizeRagdollAndAnimation();
}

void preSimulation()
{
	// Evaluate animation
	Ragdoll::evaluateAnimation();

	// Update the trigger body's position and orientation by the current pose
	Ragdoll::updateTriggerPosition();

	// Process state of the ragdoll
	processRagdollState();

	// Setup ragdoll motor
	Ragdoll::updateMotorRatio(timeStep);

	// Drive ragdoll
	Ragdoll::driveRagdoll();

	// If needed, force to move the ragdoll to match the current pose
	switch (Ragdoll::getCurrentRagdollState().ragdollState) {
		case Ragdoll::RAGDOLL_STATE_GETUP_UP:
		case Ragdoll::RAGDOLL_STATE_GETUP_DOWN:
		Ragdoll::poseProjection(0.4f, true);
		break;

		default:
		break;
	}
}

void postSimulation()
{
	// Calc the skeleton pose from the ragdoll
	Ragdoll::applyRagdollPoseToSkeleton();
}

void physicsSimulate()
{
	preSimulation();
	
	world->simulate();
	
	postSimulation();
}

///////////////////////////////////////////////////////////////////////////////
// Picking

inline PfxVector3 calcLocalCoord(const PfxRigidState &state,const PfxVector3 &coord)
{
	return rotate(conj(state.getOrientation()),(coord - state.getPosition()));
}

PfxVector3 physicsPickStart(const PfxVector3 &p1,const PfxVector3 &p2)
{
	PfxRayInput pickRay;
	PfxRayOutput pickOut;
	
	pickRay.m_contactFilterSelf = 0xffffffff;
	pickRay.m_contactFilterTarget = 0x7fffffff; // this ray doesn't hit triggers
	pickRay.m_startPosition = p1;
	pickRay.m_direction = p2-p1;
	pickRay.m_facetMode = SCE_PFX_RAY_FACET_MODE_FRONT_ONLY;
	
	world->castSingleRay(pickRay,pickOut);
	
	if(pickOut.m_contactFlag) {
		PfxBallJointInitParam jparam;
		jparam.anchorPoint = pickOut.m_contactPoint;

		PfxRigidState &stateA = world->getRigidState(0);
		PfxRigidState &stateB = world->getRigidState(pickOut.m_objectId);

		if(pickJointId < 0) {
			PfxJoint joint;
			pfxInitializeBallJoint(joint,stateA,stateB,jparam);
			pickJointId = world->addJoint(joint);
		}

		PfxJoint &pickJoint = world->getJoint(pickJointId);

		pfxInitializeBallJoint(pickJoint,stateA,stateB,jparam);
		world->updateJoint(pickJointId);

		PfxRigidBody &bodyB = world->getRigidBody(pickOut.m_objectId);

		pickJoint.m_constraints[0].m_maxImpulse = bodyB.getMass() * 2.0f;
		pickJoint.m_constraints[1].m_maxImpulse = bodyB.getMass() * 2.0f;
		pickJoint.m_constraints[2].m_maxImpulse = bodyB.getMass() * 2.0f;

		SCE_PFX_PRINTF("pick objId %d ",pickOut.m_objectId);
		if(pickOut.m_subData.getType() == PfxSubData::MESH_INFO) {
			SCE_PFX_PRINTF("mesh islandId %d facetId %d",pickOut.m_subData.getIslandId(),pickOut.m_subData.getFacetId());
		}
		SCE_PFX_PRINTF("\n");
		
		if (Ragdoll::isInRagdoll(pickOut.m_objectId)) {
			changeRagdollState(Ragdoll::RAGDOLL_STATE_FLYBY);
			gPickingRagdoll = true;
		}
		
		return pickOut.m_contactPoint.offset;
	}

	return PfxVector3(0.0f);
}

void physicsPickUpdate(const PfxVector3 &p)
{
	if(pickJointId < 0) return;

	PfxJoint &pickJoint = world->getJoint(pickJointId);

	if(pickJoint.m_active>0) {
		PfxRigidState &stateA = world->getRigidState(pickJoint.m_rigidBodyIdA);
		PfxRigidState &stateB = world->getRigidState(pickJoint.m_rigidBodyIdB);
		pickJoint.m_anchorA = calcLocalCoord(stateA,p);
		if(stateB.isAsleep()) {
			stateB.wakeup();
		}
	}
}

void physicsPickEnd()
{
	if(pickJointId < 0) return;

	PfxJoint &pickJoint = world->getJoint(pickJointId);
	
	if(pickJoint.m_active>0) {
		pickJoint.m_active = 0;
	}
	
	gPickingRagdoll = false;
}



void physicsThrowRagdoll()
{
	//PfxVector3 targetLinearVelocity(0.0f , 5.0f, -15.0f);
	//PfxVector3 targetAngularVelocity(5.0f, 0.0f, 0.0f);
	
	if (x < 200.0f) {
		x = x * 1.5;
		y = y * 1.5;
		z = z * 1.5;
		SCE_PFX_PRINTF("x, y, z: %i, %i, %i \n", x, y, z);
		PfxVector3 targetLinearVelocity(x, y, -z);
		PfxVector3 targetAngularVelocity(y, 0.0f, 0.0f);

		targetLinearVelocity = rotate(PfxQuat::rotationY(SCE_PFX_PI*2.0f* 0.01f * (rand() % 100)), targetLinearVelocity);

		if (rand() % 2 == 0) {
			targetAngularVelocity = -targetAngularVelocity;
		}

		Ragdoll::applyTargetVelocityToRagdoll(targetLinearVelocity, targetAngularVelocity, 1.0f, 1.0f);

		changeRagdollState(Ragdoll::RAGDOLL_STATE_FLYBY);
	}
	
}

void checkController() {
	PadData padD;
	padD.touchPadData;
	padD.buttons;
	padD.lx;
	padD.motionData;
	padD.rx;
	padD.touchNumber;
	padD.r2;
	padD.l2;

	SCE_PFX_PRINTF("padD.r2 number: %i \n", padD.r2);
	SCE_PFX_PRINTF("padD.l2 number: %i \n", padD.l2);
	SCE_PFX_PRINTF("padD.buttons number: %i \n", padD.buttons);
	SCE_PFX_PRINTF("padD.lx number: %i \n", padD.lx);
	SCE_PFX_PRINTF("padD.motionData number: %i \n", padD.motionData);
	SCE_PFX_PRINTF("padD.rx number: %i \n", padD.rx);
	SCE_PFX_PRINTF("padD.touchNumber number: %i \n", padD.touchNumber);
}

int lxAnalogValue;
int lyAnalogValue;


void getAnalogueInfo() {
	PadData padD;
	padD.r2;
	padD.l2;
	//int rtriggerValue = padD.r2;	//0
	int ltriggerValue = padD.l2;	//7

	padD.lx;
	padD.ly;
	int lxAnalogValue = padD.lx;
	int lyAnalogValue = padD.ly;
	
}

void physicsThrowCubes()
{
	//for (int i = 0;i < (int)world->getRigidBodyCount();i++) {
	//world->applyForce(); //world->getRigidBodyCount()
	//}

	//PfxQuat::identity();	
	//const sce::Vectormath::Simd::Aos::Vector2 &getRightStick();

	PadData padD;
	padD.r2;
	padD.l2;
	//int rtriggerValue = padD.r2;	//0
	int ltriggerValue = padD.l2;	//7

	padD.rx;
	//padD.ly;
	padD.ry;
	//padD.lx;
	//int lxAnalogValue = padD.rx;
	//int lyAnalogValue = padD.ry;

	getAnalogueInfo();
	for (unsigned int i = 0;i < world->getRigidBodyCount();i++) {
		if (!world->isRemovedRigidBody(i)) {
			pfxApplyExternalForce(
				world->getRigidState(i), world->getRigidBody(i),
				world->getRigidBody(i).getMass() * PfxVector3(padD.rx, padD.ry, 1000),		//moves if all values numerical for Vec3
				PfxVector3(0.0f),
				world->getTimeStep());
		}
	}

	SCE_PFX_PRINTF("Rx: %i \n", padD.rx);
	SCE_PFX_PRINTF("Ry: %i \n", padD.ry);


}


///////////////////////////////////////////////////////////////////////////////
// Get Information

int physicsGetNumRigidbodies()
{
	return world->getRigidBodyCount();
}

const PfxRigidState& physicsGetState(int id)
{
	return world->getRigidState(id);
}

const PfxRigidBody& physicsGetBody(int id)
{
	return world->getRigidBody(id);
}

const PfxCollidable& physicsGetCollidable(int id)
{
	return world->getCollidable(id);
}

int physicsGetNumContacts()
{
	return world->getContactCount();
}

const PfxContactManifold &physicsGetContact(int id,PfxUInt32 &rigidbodyIdA,PfxUInt32 &rigidbodyIdB)
{
	return world->getContactManifold(id,rigidbodyIdA,rigidbodyIdB);
}

int physicsGetRenderMeshId(void *collisionMesh)
{
	int meshId = -1;
	renderMeshes->find(collisionMesh,meshId);
	return meshId;
}

///////////////////////////////////////////////////////////////////////////////
// Serialize

namespace sce {
namespace PhysicsEffects {

void serializeInitWrite(PfxSerializeCapacity *capacity,PfxSerializeBuffer *buffer)
{
	SCE_PFX_ASSERT(capacity);
	SCE_PFX_ASSERT(buffer);

	capacity->maxRigidBodies		= world->getRigidBodyCapacity();
	capacity->maxShapes				= world->getShapeCapacity();
	capacity->maxJoints				= world->getJointCapacity();
	capacity->maxContacts			= world->getContactCapacity();
	capacity->maxConvexMeshes		= convexMeshes->capacity();
	capacity->maxLargeMeshes		= largeMeshes->capacity();
	capacity->maxLargeCompounds = 0;
	capacity->maxNonContactPairs	= world->m_maxNonContactPairs;
	capacity->numRigidBodies		= world->getRigidBodyCount();
	capacity->numShapes				= world->getShapeCount();
	capacity->numJoints				= world->getJointCount();
	capacity->numConvexMeshes		= convexMeshes->size();
	capacity->numLargeMeshes		= largeMeshes->size();
	capacity->numNonContactPairs	= world->m_numNonContactPairs;
	
	buffer->worldMin		= &world->m_worldMin;
	buffer->worldMax		= &world->m_worldMax;
	buffer->gravity			= &world->m_gravity;
	buffer->states			= world->m_states.ptr();
	buffer->bodies			= world->m_bodies.ptr();
	buffer->collidables		= world->m_collidables.ptr();
	buffer->shapes			= world->m_shapes.ptr();
	buffer->joints			= world->m_joints.ptr();
	buffer->nonContactPairs	= world->m_nonContactPairs;
	buffer->timeStep		= &world->m_timeStep;
	buffer->iteration		= &world->m_velocityIteration[0];
	buffer->separateBias	= &world->m_separateWeight;
	buffer->convexMeshes	= convexMeshes->ptr();
	buffer->largeMeshes		= largeMeshes->ptr();
	buffer->largeCompounds = NULL;
}

void serializeTermWrite(PfxSerializeCapacity *capacity,PfxSerializeBuffer *buffer)
{
}

void serializeError(PfxInt32 err,const char **tags,int nTags)
{
	SCE_PFX_PRINTF("error occurred err=0x%x ",err);
	for(int i=0;i<nTags;i++) {
		SCE_PFX_PRINTF("|%s",tags[i]);
	}
	SCE_PFX_PRINTF("\n");

	world->reset();
}

} //namespace PhysicsEffects
} //namespace sce

///////////////////////////////////////////////////////////////////////////////
// Serialize

bool physicsOutbound(const char *filename)
{
	FILE *fp = NULL;
#ifdef _WIN32
	errno_t err = fopen_s(&fp,filename,"w");
	if(err != 0) {
		SCE_PFX_PRINTF("can't save %s\n",filename);
		return false;
	}
#else
	fp = fopen(filename,"w");
	if(!fp) {
		SCE_PFX_PRINTF("can't save %s\n",filename);
		return false;
	}
#endif
	
	pfxSerializeWrite(
		fp,
		serializeInitWrite,
		serializeTermWrite,
		serializeError
		);
	
	fclose(fp);
	
	return true;
}
