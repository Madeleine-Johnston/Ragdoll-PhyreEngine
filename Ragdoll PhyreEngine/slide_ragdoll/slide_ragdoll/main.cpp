/* SIE CONFIDENTIAL
 * PlayStation(R)4 Programmer Tool Runtime Library Release 05.008.001
 *                Copyright (C) 2017 Sony Interactive Entertainment Inc.
 *                                                All Rights Reserved.
 */

#include "../../api_physics_effects/common/common.h"
#include "../../api_physics_effects/common/ctrl_func.h"
#include "../../api_physics_effects/common/render_func.h"
#include "../../api_physics_effects/common/perf_func.h"

#include "physics_func.h"
#include "../../tutorial_physics_effects_high_level/high_level/pfx_high_level_include.h"
#include "../ragdoll_common/ragdoll_control.h"
#include "input.h"
#include <chrono>
//include text header
//#include <Text/PhyreText.h>


#define	SAMPLE_NAME "tutorial_ragdoll"

//#define ENABLE_DEBUG_DRAW

#ifdef ENABLE_DEBUG_DRAW
PfxUInt32 debugRenderWorkBytes;
void *debugRenderWorkBuff;
PfxDebugRender s_debugRender;
#endif

static bool g_isRunning = true;

int sceneId = 0;
bool simulating = true;

PfxVector3 ray_p1,ray_p2;

static PfxVector3 pickPos(0.0f);

// Rigid body world
extern PfxRigidBodyWorld *world;

// Animation
namespace Ragdoll {
	extern EdgeAnimSkeleton *gSkeleton;
	extern PfxTransform3 *gWorldMatrices;
	extern PfxTransform3 *gWorldMatricesFromRagdoll;
}

// Serialize
#if defined(__psp2__)
unsigned int	sceLibcHeapSize	= 16*1024*1024;
#elif defined(__ORBIS__)
size_t	sceLibcHeapSize	= 32*1024*1024;
#endif

#if defined(_WIN32)
const char* serialize_fname = "snapshot.txt";
#elif defined(__psp2__)
const char* serialize_fname = "app0:snapshot.txt";
#elif defined(__ORBIS__)
const char* serialize_fname = "/hostapp/snapshot.txt";
#endif

static int frames = 0;
static double starttime = 0;
static bool first;
static float fps = 0.0f;
double timepassed;

//void checkFrameRate() {
//	if (first)
//	{
//		frames = 0;
//		starttime = timepassed;
//		first = false;
//		return;
//	}
//
//	if (timepassed - starttime > 0.25 && frames > 10)
//	{
//		fps = (double)frames / (timepassed - starttime);
//		starttime = timepassed;
//		frames = 0;
//	}
//}

static void render(void)
{
	//trying to only get timevalues for first 3 renders - doesnt work right now. Tried bool check too.
	int pass = 0;
	pass++;
	if (pass < 3) {
		auto start = std::chrono::high_resolution_clock::now();

		renderBegin();

		auto finish = std::chrono::high_resolution_clock::now();
		auto difference = finish - start;
		printf("time diff0:	%i \n", difference);

		auto start1 = std::chrono::high_resolution_clock::now();

		const PfxVector3 colorWhite(1.0f);
		const PfxVector3 colorGray(0.0f);

		//text rendering
		//PHYRE_TRY(m_text[i]->renderText(m_renderer));

		for (int i = 0;i < (int)world->getRigidBodyCount();i++) {
			if (world->isRemovedRigidBody(i)) continue;

			const PfxRigidState &state = world->getRigidState(i);
			const PfxCollidable &coll = world->getCollidable(i);

			if (state.getMotionType() == kPfxMotionTypeTrigger) continue;

			PfxVector3 color = state.isAsleep() ? colorGray : colorWhite;

			PfxTransform3 rbT(state.getOrientation(), state.getPosition());

			PfxShapeIterator itrShape(coll);
			for (int j = 0;j < (int)coll.getNumShapes();j++, ++itrShape) {
				const PfxShape &shape = *itrShape;
				PfxTransform3 offsetT = shape.getOffsetTransform() * PfxTransform3::scale(shape.getScaleXyz());
				PfxTransform3 worldT = rbT * offsetT;

				switch (shape.getType()) {
				case kPfxShapeSphere:
					renderSphere(
						worldT,
						color,
						PfxFloatInVec(shape.getSphere().m_radius));
					break;

				case kPfxShapeBox:
					renderBox(
						worldT,
						color,
						shape.getBox().m_half);
					break;

				case kPfxShapeCapsule:
					renderCapsule(
						worldT,
						color,
						PfxFloatInVec(shape.getCapsule().m_radius),
						PfxFloatInVec(shape.getCapsule().m_halfLen));
					break;

				case kPfxShapeCylinder:
					renderCylinder(
						worldT,
						color,
						PfxFloatInVec(shape.getCylinder().m_radius),
						PfxFloatInVec(shape.getCylinder().m_halfLen));
					break;

				case kPfxShapeConvexMesh:
				case kPfxShapeLargeTriMesh:
				{
					int meshId = -1;
					if (shape.getType() == kPfxShapeConvexMesh) {
						meshId = physicsGetRenderMeshId((void*)shape.getConvexMesh());
					}
					else {
						meshId = physicsGetRenderMeshId((void*)shape.getLargeTriMesh());
					}

					if (meshId >= 0) {
						renderMesh(worldT, color, meshId);
					}
				}
				break;

				default:
					break;
				}
			}
		}

		auto finish1 = std::chrono::high_resolution_clock::now();
		auto difference1 = finish1 - start1;
		printf("time diff1:	%i \n", difference1);

		auto start2 = std::chrono::high_resolution_clock::now();

		renderDebugBegin();

		auto finish2 = std::chrono::high_resolution_clock::now();
		auto difference2 = finish2 - start2;
		printf("time diff2:	%i \n", difference2);

		auto start3 = std::chrono::high_resolution_clock::now();

#ifdef ENABLE_DEBUG_DRAW
		pfxDebugRenderRenderLocalAxis(s_debugRender, world->getRigidStatePtr(), world->getRigidBodyCount());
		pfxDebugRenderRenderAabb(s_debugRender, world->getActiveBroadphaseProxyContainer());
		pfxDebugRenderRenderContact(s_debugRender, world->getContactContainer(), world->getRigidStatePtr(), world->getContactPairPtr(), world->getContactCount());
		pfxDebugRenderRenderJoint(s_debugRender, world->getRigidStatePtr(), world->getJointPtr(), world->getJointCount());
#endif

		auto finish3 = std::chrono::high_resolution_clock::now();
		auto difference3 = finish3 - start3;
		printf("time diff3:	%i \n", difference3);

		auto start4 = std::chrono::high_resolution_clock::now();
		// Render a skeleton
#if 0
		PfxRigidState &rootRb = Ragdoll::getRootRigidStateInRagdoll();

		PfxVector3 offsetPos = rootRb.getPosition();
		PfxMatrix3 offsetOri(rootRb.getOrientation());

		const uint16_t hipId = edgeAnimSkeletonGetJointIndexByName(Ragdoll::gSkeleton, "Hips");
		const uint16_t torsoId = edgeAnimSkeletonGetJointIndexByName(Ragdoll::gSkeleton, "Spine1");

		PfxTransform3 &hipT = Ragdoll::gWorldMatrices[hipId];
		PfxTransform3 &torsoT = Ragdoll::gWorldMatrices[torsoId];
		PfxVector3 rootPos = 0.5f * (hipT.getTranslation() + torsoT.getTranslation());

		offsetOri = offsetOri * transpose(hipT.getUpper3x3());


		// render animation skeleton
		for (uint16_t j = 0; j < Ragdoll::gSkeleton->numJoints; j++) {
			int16_t parent = edgeAnimSkeletonGetJointParent(Ragdoll::gSkeleton, j);
			PfxTransform3 &trnsB = Ragdoll::gWorldMatrices[j];
			PfxVector3 posB = trnsB.getTranslation() + PfxVector3(1.0f, 0.0f, 0.0f);
			PfxVector3 posB_ = offsetPos + offsetOri * (trnsB.getTranslation() - rootPos);

			if (parent >= 0) {
				PfxTransform3 &trnsA = Ragdoll::gWorldMatrices[parent];
				PfxVector3 posA = trnsA.getTranslation() + PfxVector3(1.0f, 0.0f, 0.0f);
				PfxVector3 posA_ = offsetPos + offsetOri * (trnsA.getTranslation() - rootPos);
				renderDebugLine(posA, posB, PfxVector3(0.0f, 0.0f, 1.0f));
				renderDebugLine(posA_, posB_, PfxVector3(1.0f, 0.0f, 0.0f));
			}
		}

		renderDebugLine(PfxVector3(1.0f, 0.0f, 0.0f), PfxVector3(1.0f, 0.0f, 0.0f) + PfxVector3(0.1f, 0.0f, 0.0f), PfxVector3(0.8f, 0.0f, 0.0f));
		renderDebugLine(PfxVector3(1.0f, 0.0f, 0.0f), PfxVector3(1.0f, 0.0f, 0.0f) + PfxVector3(0.0f, 1.0f, 0.0f), PfxVector3(0.0f, 0.8f, 0.0f));
		renderDebugLine(PfxVector3(1.0f, 0.0f, 0.0f), PfxVector3(1.0f, 0.0f, 0.0f) + PfxVector3(0.0f, 0.0f, 0.1f), PfxVector3(0.0f, 0.0f, 0.8f));


		// render skeleton from the ragdoll
		for (uint16_t j = 0; j < Ragdoll::gSkeleton->numJoints; j++) {
			int16_t parent = edgeAnimSkeletonGetJointParent(Ragdoll::gSkeleton, j);
			PfxTransform3 &trnsB = Ragdoll::gWorldMatricesFromRagdoll[j];
			PfxVector3 posB = trnsB.getTranslation() + PfxVector3(-1.0f, 0.0f, 0.0f);

			if (parent >= 0) {
				PfxTransform3 &trnsA = Ragdoll::gWorldMatricesFromRagdoll[parent];
				PfxVector3 posA = trnsA.getTranslation() + PfxVector3(-1.0f, 0.0f, 0.0f);
				renderDebugLine(posA, posB, PfxVector3(0.0f, 1.0f, 0.0f));
			}
		}


		// render root position
		{
			PfxVector3 originPos = Ragdoll::getRagdollOriginPosition();
			PfxMatrix3 originOri = Ragdoll::getRagdollOriginOrientation();

			PfxVector3 rootPos = Ragdoll::getRagdollTargetRootPosition();
			PfxMatrix3 rootOri = Ragdoll::getRagdollTargetRootOrientation();

			renderDebugLine(originPos, originPos + 0.1f * originOri.getCol0(), PfxVector3(0.8f, 0.0f, 0.0f));
			renderDebugLine(originPos, originPos + 0.1f * originOri.getCol1(), PfxVector3(0.0f, 0.8f, 0.0f));
			renderDebugLine(originPos, originPos + 0.1f * originOri.getCol2(), PfxVector3(0.0f, 0.0f, 0.8f));

			renderDebugLine(rootPos, rootPos + 0.1f * rootOri.getCol0(), PfxVector3(0.8f, 0.0f, 0.0f));
			renderDebugLine(rootPos, rootPos + 0.1f * rootOri.getCol1(), PfxVector3(0.0f, 0.8f, 0.0f));
			renderDebugLine(rootPos, rootPos + 0.1f * rootOri.getCol2(), PfxVector3(0.0f, 0.0f, 0.8f));

			renderDebugLine(rootPos, originPos, PfxVector3(1.0f, 0.0f, 0.0f));
		}
#endif
		auto finish4 = std::chrono::high_resolution_clock::now();
		auto difference4 = finish4 - start4;
		printf("time diff4:	%i \n", difference4);

		auto start5 = std::chrono::high_resolution_clock::now();

		renderDebugEnd();

		auto finish5 = std::chrono::high_resolution_clock::now();
		auto difference5 = finish5 - start5;
		printf("time diff5:	%i \n", difference5);

		auto start6 = std::chrono::high_resolution_clock::now();

		renderEnd(true);

		auto finish6 = std::chrono::high_resolution_clock::now();
		auto difference6 = finish6 - start6;
		printf("time diff6:	%i \n", difference6);
	}
}

static int init(void)
{
	perfInit();
	ctrlInit();
	renderInit(SAMPLE_NAME);
	physicsInit();

	float angX = -0.51f;
	float angY = 0.55f;
	float r = 15.0f;
	renderSetViewAngle(angX,angY,r);

#ifdef ENABLE_DEBUG_DRAW
	debugRenderWorkBytes = pfxDebugRenderQueryMem();
	debugRenderWorkBuff = SCE_PFX_UTIL_ALLOC(16,debugRenderWorkBytes);
	
	PfxDebugRenderInitParam param;
	param.pointFunc = renderDebugPoint;
	param.lineFunc = renderDebugLine;
	param.arcFunc = renderDebugArc;
	param.aabbFunc = renderDebugAabb;
	param.boxFunc = renderDebugBox;
	
	pfxDebugRenderInit(s_debugRender,param,debugRenderWorkBuff,debugRenderWorkBytes);

	pfxDebugRenderSetScale(s_debugRender,0.1f);
#endif

	return 0;
}

static int shutdown(void)
{
	ctrlRelease();
	renderRelease();
	physicsRelease();
	perfRelease();

#ifdef ENABLE_DEBUG_DRAW
	SCE_PFX_UTIL_FREE(debugRenderWorkBuff);
#endif

	//delete text 
	/*for (PUInt32 i = 0; i<c_totalTextString;i++)
	{
		delete m_textMaterial[i];
		m_textMaterial[i] = NULL;
		delete m_text[i];
		m_text[i] = NULL;
	}*/


	return 0;
}

void releaseAllMeshes()
{
	//	meshes to display
	renderReleaseMeshAll();
}

static void update(void)
{
	float angX,angY,r;
	renderGetViewAngle(angX,angY,r);

	ctrlUpdate();
	//checkController();
	getAnalogueInfo();
	
	if(ctrlButtonPressed(BTN_UP)) {
		angX -= 0.05f;
		if(angX < -1.4f) angX = -1.4f;
		if(angX > -0.01f) angX = -0.01f;
	}

	if(ctrlButtonPressed(BTN_DOWN)) {
		angX += 0.05f;
		if(angX < -1.4f) angX = -1.4f;
		if(angX > -0.01f) angX = -0.01f;
	}

	if(ctrlButtonPressed(BTN_LEFT)) {
		angY -= 0.05f;
	}

	if(ctrlButtonPressed(BTN_RIGHT)) {
		angY += 0.05f;
	}

	if(ctrlButtonPressed(BTN_ZOOM_OUT)) {
		r *= 1.1f;
		if(r > 500.0f) r = 500.0f;
	}

	if(ctrlButtonPressed(BTN_ZOOM_IN)) {
		r *= 0.9f;
		if(r < 1.0f) r = 1.0f;
	}

	if(ctrlButtonPressed(BTN_SCENE_RESET) == BTN_STAT_DOWN) {
		//getAnalogueInfo();
		physicsThrowCubes();
		printf("throw cubes");
	}

	if(ctrlButtonPressed(BTN_SCENE_NEXT) == BTN_STAT_DOWN) {
		renderWait();
		renderReleaseMeshAll();
		physicsCreateScene(++sceneId);
	}

	if(ctrlButtonPressed(BTN_SIMULATION) == BTN_STAT_DOWN) {
		simulating = !simulating;
	}

	if(ctrlButtonPressed(BTN_STEP) == BTN_STAT_DOWN) {
		simulating = true;
	}
	else if(ctrlButtonPressed(BTN_STEP) == BTN_STAT_UP || ctrlButtonPressed(BTN_STEP) == BTN_STAT_KEEP) {
		simulating = false;
	}

	if (ctrlButtonPressed(BTN_JUMP) == BTN_STAT_DOWN) {
		physicsThrowRagdoll();
		printf("throw ragdoll");
		//physicsThrowCubes();
	}

	if (ctrlButtonPressed(BTN_SERIALIZE_OUT)==BTN_STAT_DOWN) {
		physicsOutbound(serialize_fname);
	}

	int w,h;
	renderGetScreenSize(w,h);
	ctrlSetScreenSize(w,h);
	
	if(ctrlButtonPressed(BTN_PICK) == BTN_STAT_DOWN) {
		int sx,sy;
		ctrlGetCursorPosition(sx,sy);
		PfxVector3 wp1((float)sx,(float)sy,0.0f);
		PfxVector3 wp2((float)sx,(float)sy,1.0f);
		wp1 = renderGetWorldPosition(wp1);
		wp2 = renderGetWorldPosition(wp2);
		pickPos = physicsPickStart(wp1,wp2);

		ray_p1 = wp1;
		ray_p2 = wp2;
	}
	else if(ctrlButtonPressed(BTN_PICK) == BTN_STAT_KEEP) {
		int sx,sy;
		ctrlGetCursorPosition(sx,sy);
		PfxVector3 sp = renderGetScreenPosition(pickPos);
		PfxVector3 wp((float)sx,(float)sy,sp[2]);
		wp = renderGetWorldPosition(wp);
		physicsPickUpdate(wp);
	}
	else if(ctrlButtonPressed(BTN_PICK) == BTN_STAT_UP) {
		physicsPickEnd();
	}

	renderSetViewAngle(angX,angY,r);
	}

#ifndef _WIN32

///////////////////////////////////////////////////////////////////////////////
// Main

int main(void)
{
	init();

	physicsCreateScene(sceneId);

	printf("## %s: INIT SUCCEEDED ##\n", SAMPLE_NAME);

	while (g_isRunning) {
		update();
		if(simulating) physicsSimulate();
		render();
		perfSync();
		void getAnalogueInfo();
	}

	shutdown();

	printf("## %s: FINISHED ##\n", SAMPLE_NAME);

	return 0;
}

#else

///////////////////////////////////////////////////////////////////////////////
// WinMain

int WINAPI WinMain(HINSTANCE hInstance,HINSTANCE hPrevInstance,LPSTR lpCmdLine,int nCmdShow)
{
#ifdef _WIN64
	SCE_PFX_ASSERT( SCE_PFX_IS_RUNNING_ON_64BIT_ENV()  );
#endif

	init();
	
	physicsCreateScene(sceneId);
	
	SCE_PFX_PRINTF("## %s: INIT SUCCEEDED ##\n", SAMPLE_NAME);
	
	MSG msg;
	while(g_isRunning) {
		if(PeekMessage(&msg,NULL,0,0,PM_REMOVE)) {
			if(msg.message==WM_QUIT) {
				g_isRunning = false;
			}
			else {
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
		}
		else {
			update();
			if(simulating) physicsSimulate();
			render();
			perfSync();
		}
	}

	shutdown();

	SCE_PFX_PRINTF("## %s: FINISHED ##\n", SAMPLE_NAME);

	return (msg.wParam);
}

#endif
