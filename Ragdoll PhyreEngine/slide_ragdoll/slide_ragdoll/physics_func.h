/* SIE CONFIDENTIAL
 * PlayStation(R)4 Programmer Tool Runtime Library Release 05.008.001
 *                Copyright (C) 2017 Sony Interactive Entertainment Inc.
 *                                                All Rights Reserved.
 */

#ifndef __PHYSICS_FUNC_H__
#define __PHYSICS_FUNC_H__

#include "../../tutorial_physics_effects_high_level/high_level/pfx_high_level_include.h"


using namespace sce::PhysicsEffects;

//// The number of text strings used by this game.
//static const Phyre::PUInt32 c_totalTextStrings = 10;
//// The material used to render the text with.
//Phyre::PText::PBitmapTextMaterial *m_textMaterial[c_totalTextStrings];
//// The text object to be rendered.
//Phyre::PText::PBitmapFontText *m_text[c_totalTextStrings];
//
//// Create the text and text materials
//PText::PBitmapFont &bitmapFont = *PCluster::PObjectIteratorOfType
//<PText::PBitmapFont>(*m_loadedClusters[g_fontIndex]);
//for (PUInt32 i = 0; i < c_totalTextStrings; i++)
//{
//	PHYRE_TRY(PText::PUtilityText::CreateText(bitmapFont,
//		*m_loadedClusters[g_textShaderIndex], m_text[i], m_textMaterial[i],
//		PText::PUtilityText::PE_TEXT_RENDER_TECHNIQUE_ALPHA_BLEND));
//}


//J シミュレーション
//E Simulation
bool physicsInit();
void physicsRelease();
void physicsCreateScene(int sceneId);
void physicsSimulate();
void physicsResetWorld();

//J ピッキング
//E picking
PfxVector3 physicsPickStart(const PfxVector3 &p1,const PfxVector3 &p2);
void physicsPickUpdate(const PfxVector3 &p);
void physicsPickEnd();

// Throw a ragdoll
void physicsThrowRagdoll();
void physicsThrowCubes();
void checkController();
void getAnalogueInfo();

// Get render mesh index
int physicsGetRenderMeshId(void *collisionMesh);

// Serialize
bool physicsInbound(const char *filename);
bool physicsOutbound(const char *filename);

#endif /* __RENDER_FUNC_H__ */
