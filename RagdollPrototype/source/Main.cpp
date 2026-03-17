#include <cmath>

#include "plugin.h"
#include "common.h"
#include "Patch.h"

#include "CEntity.h"
#include "CPed.h"
#include "CFont.h"
#include "CSprite.h"
#include "CRGBA.h"
#include "CVector.h"
#include "eEntityType.h"
#include "ePedBones.h"
#include "RenderWare.h"

using namespace plugin;

namespace ragdoll_debug {

    struct BoneLabel {
        unsigned int id;
        const char* name;
    };

    static bool gDrawBones = true;
    static bool gForceTPose = true;

    static BoneLabel gBoneLabels[] = {
        { BONE_PELVIS,         "PELVIS" },
        { BONE_SPINE1,         "SPINE1" },
        { BONE_UPPERTORSO,     "TORSO"  },
        { BONE_NECK,           "NECK"   },
        { BONE_HEAD,           "HEAD"   },

        { BONE_LEFTSHOULDER,   "L SHLDR" },
        { BONE_LEFTELBOW,      "L ELBOW" },
        { BONE_LEFTWRIST,      "L WRIST" },
        { BONE_LEFTHAND,       "L HAND"  },

        { BONE_RIGHTSHOULDER,  "R SHLDR" },
        { BONE_RIGHTELBOW,     "R ELBOW" },
        { BONE_RIGHTWRIST,     "R WRIST" },
        { BONE_RIGHTHAND,      "R HAND"  },

        { BONE_LEFTHIP,        "L HIP"   },
        { BONE_LEFTKNEE,       "L KNEE"  },
        { BONE_LEFTANKLE,      "L ANKLE" },
        { BONE_LEFTFOOT,       "L FOOT"  },

        { BONE_RIGHTHIP,       "R HIP"   },
        { BONE_RIGHTKNEE,      "R KNEE"  },
        { BONE_RIGHTANKLE,     "R ANKLE" },
        { BONE_RIGHTFOOT,      "R FOOT"  }
    };

    static float VecDot(const CVector& a, const CVector& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    static CVector VecCross(const CVector& a, const CVector& b) {
        return CVector(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        );
    }

    static CVector VecAdd(const CVector& a, const CVector& b) {
        return CVector(a.x + b.x, a.y + b.y, a.z + b.z);
    }

    static CVector VecSub(const CVector& a, const CVector& b) {
        return CVector(a.x - b.x, a.y - b.y, a.z - b.z);
    }

    static CVector VecScale(const CVector& v, float s) {
        return CVector(v.x * s, v.y * s, v.z * s);
    }

    static float VecLenSq(const CVector& v) {
        return v.x * v.x + v.y * v.y + v.z * v.z;
    }

    static float VecLen(const CVector& v) {
        return std::sqrt(VecLenSq(v));
    }

    static CVector SafeNormalise(const CVector& v, const CVector& fallback) {
        const float lenSq = VecLenSq(v);
        if (lenSq <= 0.000001f)
            return fallback;

        const float invLen = 1.0f / std::sqrt(lenSq);
        return CVector(v.x * invLen, v.y * invLen, v.z * invLen);
    }

    static float ClampUnit(float x) {
        if (x < -1.0f) return -1.0f;
        if (x > 1.0f) return  1.0f;
        return x;
    }

    static CVector GetMatrixRight(const RwMatrix* m) {
        return CVector(m->right.x, m->right.y, m->right.z);
    }

    static CVector GetMatrixUp(const RwMatrix* m) {
        return CVector(m->up.x, m->up.y, m->up.z);
    }

    static CVector GetMatrixAt(const RwMatrix* m) {
        return CVector(m->at.x, m->at.y, m->at.z);
    }

    static CVector GetMatrixPos(const RwMatrix* m) {
        return CVector(m->pos.x, m->pos.y, m->pos.z);
    }

    static void SetMatrixPos(RwMatrix* m, const CVector& p) {
        m->pos.x = p.x;
        m->pos.y = p.y;
        m->pos.z = p.z;
    }

    static CVector ProjectOntoPlane(const CVector& v, const CVector& planeNormalRaw) {
        const CVector n = SafeNormalise(planeNormalRaw, CVector(0.0f, 0.0f, 1.0f));
        return VecSub(v, VecScale(n, VecDot(v, n)));
    }

    static CVector RotateVectorAroundAxisRad(const CVector& v, const CVector& axisRaw, float angleRad) {
        const CVector axis = SafeNormalise(axisRaw, CVector(0.0f, 0.0f, 1.0f));
        const float c = std::cos(angleRad);
        const float s = std::sin(angleRad);

        return VecAdd(
            VecAdd(
                VecScale(v, c),
                VecScale(VecCross(axis, v), s)
            ),
            VecScale(axis, VecDot(axis, v) * (1.0f - c))
        );
    }

    static void RotateMatrixBasisAroundAxisRad(RwMatrix* m, const CVector& axis, float angleRad) {
        CVector right = GetMatrixRight(m);
        CVector up = GetMatrixUp(m);
        CVector at = GetMatrixAt(m);

        right = RotateVectorAroundAxisRad(right, axis, angleRad);
        up = RotateVectorAroundAxisRad(up, axis, angleRad);
        at = RotateVectorAroundAxisRad(at, axis, angleRad);

        m->right.x = right.x;
        m->right.y = right.y;
        m->right.z = right.z;

        m->up.x = up.x;
        m->up.y = up.y;
        m->up.z = up.z;

        m->at.x = at.x;
        m->at.y = at.y;
        m->at.z = at.z;

        RwMatrixUpdate(m);
    }

    static RpHAnimHierarchy* GetSkinHierarchy(RpClump* clump) {
        if (!clump)
            return nullptr;

        return GetAnimHierarchyFromSkinClump(clump);
    }

    static RwMatrix* GetBoneMatrixById(RpHAnimHierarchy* hierarchy, int boneId) {
        if (!hierarchy)
            return nullptr;

        const int index = RpHAnimIDGetIndex(hierarchy, boneId);
        if (index < 0)
            return nullptr;

        RwMatrix* matrixArray = RpHAnimHierarchyGetMatrixArray(hierarchy);
        if (!matrixArray)
            return nullptr;

        return &matrixArray[index];
    }

    static void RotateBoneSoChildPointsTo(
        RwMatrix* bone,
        const CVector& currentChildDirRaw,
        const CVector& desiredChildDirRaw,
        const CVector& fallbackAxisRaw)
    {
        if (!bone)
            return;

        const CVector currentDir = SafeNormalise(currentChildDirRaw, CVector(0.0f, 1.0f, 0.0f));
        const CVector desiredDir = SafeNormalise(desiredChildDirRaw, currentDir);

        float dot = ClampUnit(VecDot(currentDir, desiredDir));
        float angleRad = std::acos(dot);

        CVector axis = VecCross(currentDir, desiredDir);

        if (VecLenSq(axis) <= 0.000001f) {
            axis = fallbackAxisRaw;

            if (dot > 0.9999f)
                angleRad = 0.0f;
            else
                angleRad = 3.14159265358979323846f;
        }

        axis = SafeNormalise(axis, fallbackAxisRaw);
        RotateMatrixBasisAroundAxisRad(bone, axis, angleRad);
    }

    static void SolveLinearChain5(
        RwMatrix* a,
        RwMatrix* b,
        RwMatrix* c,
        RwMatrix* d,
        RwMatrix* e,
        const CVector& desiredDirRaw,
        const CVector& fallbackAxisRaw)
    {
        if (!a || !b || !c || !d || !e)
            return;

        const CVector aPos = GetMatrixPos(a);
        const CVector bPos = GetMatrixPos(b);
        const CVector cPos = GetMatrixPos(c);
        const CVector dPos = GetMatrixPos(d);
        const CVector ePos = GetMatrixPos(e);

        const float abLen = VecLen(VecSub(bPos, aPos));
        const float bcLen = VecLen(VecSub(cPos, bPos));
        const float cdLen = VecLen(VecSub(dPos, cPos));
        const float deLen = VecLen(VecSub(ePos, dPos));

        if (abLen <= 0.0001f || bcLen <= 0.0001f || cdLen <= 0.0001f || deLen <= 0.0001f)
            return;

        const CVector desiredDir = SafeNormalise(desiredDirRaw, CVector(0.0f, 0.0f, 1.0f));

        RotateBoneSoChildPointsTo(a, VecSub(bPos, aPos), desiredDir, fallbackAxisRaw);
        RotateBoneSoChildPointsTo(b, VecSub(cPos, bPos), desiredDir, fallbackAxisRaw);
        RotateBoneSoChildPointsTo(c, VecSub(dPos, cPos), desiredDir, fallbackAxisRaw);
        RotateBoneSoChildPointsTo(d, VecSub(ePos, dPos), desiredDir, fallbackAxisRaw);
        RotateBoneSoChildPointsTo(e, VecSub(ePos, dPos), desiredDir, fallbackAxisRaw);

        const CVector newB = VecAdd(aPos, VecScale(desiredDir, abLen));
        const CVector newC = VecAdd(newB, VecScale(desiredDir, bcLen));
        const CVector newD = VecAdd(newC, VecScale(desiredDir, cdLen));
        const CVector newE = VecAdd(newD, VecScale(desiredDir, deLen));

        SetMatrixPos(a, aPos);
        SetMatrixPos(b, newB);
        SetMatrixPos(c, newC);
        SetMatrixPos(d, newD);
        SetMatrixPos(e, newE);

        RwMatrixUpdate(a);
        RwMatrixUpdate(b);
        RwMatrixUpdate(c);
        RwMatrixUpdate(d);
        RwMatrixUpdate(e);
    }

    static void SolveLinearChain4(
        RwMatrix* a,
        RwMatrix* b,
        RwMatrix* c,
        RwMatrix* d,
        const CVector& desiredDirRaw,
        const CVector& fallbackAxisRaw)
    {
        if (!a || !b || !c || !d)
            return;

        const CVector aPos = GetMatrixPos(a);
        const CVector bPos = GetMatrixPos(b);
        const CVector cPos = GetMatrixPos(c);
        const CVector dPos = GetMatrixPos(d);

        const float abLen = VecLen(VecSub(bPos, aPos));
        const float bcLen = VecLen(VecSub(cPos, bPos));
        const float cdLen = VecLen(VecSub(dPos, cPos));

        if (abLen <= 0.0001f || bcLen <= 0.0001f || cdLen <= 0.0001f)
            return;

        const CVector desiredDir = SafeNormalise(desiredDirRaw, CVector(1.0f, 0.0f, 0.0f));

        RotateBoneSoChildPointsTo(a, VecSub(bPos, aPos), desiredDir, fallbackAxisRaw);
        RotateBoneSoChildPointsTo(b, VecSub(cPos, bPos), desiredDir, fallbackAxisRaw);
        RotateBoneSoChildPointsTo(c, VecSub(dPos, cPos), desiredDir, fallbackAxisRaw);
        RotateBoneSoChildPointsTo(d, VecSub(dPos, cPos), desiredDir, fallbackAxisRaw);

        const CVector newB = VecAdd(aPos, VecScale(desiredDir, abLen));
        const CVector newC = VecAdd(newB, VecScale(desiredDir, bcLen));
        const CVector newD = VecAdd(newC, VecScale(desiredDir, cdLen));

        SetMatrixPos(a, aPos);
        SetMatrixPos(b, newB);
        SetMatrixPos(c, newC);
        SetMatrixPos(d, newD);

        RwMatrixUpdate(a);
        RwMatrixUpdate(b);
        RwMatrixUpdate(c);
        RwMatrixUpdate(d);
    }

    static CVector BuildArmSideTarget(
        const CVector& torsoPos,
        const CVector& shoulderPos,
        const CVector& torsoUp,
        const CVector& fallbackSide)
    {
        CVector raw = VecSub(shoulderPos, torsoPos);
        raw = ProjectOntoPlane(raw, torsoUp);
        return SafeNormalise(raw, fallbackSide);
    }

    static CVector BuildStableLegDown(
        const CVector& lHipPos,
        const CVector& lKneePos,
        const CVector& rHipPos,
        const CVector& rKneePos,
        const CVector& fallbackDown)
    {
        const CVector lDown = SafeNormalise(VecSub(lKneePos, lHipPos), fallbackDown);
        const CVector rDown = SafeNormalise(VecSub(rKneePos, rHipPos), fallbackDown);
        const CVector avg = VecAdd(lDown, rDown);

        if (VecLenSq(avg) <= 0.000001f)
            return fallbackDown;

        return SafeNormalise(avg, fallbackDown);
    }

    static CVector BuildLegSideTarget(
        const CVector& pelvisPos,
        const CVector& hipPos,
        const CVector& legDown,
        const CVector& fallbackSide)
    {
        CVector raw = VecSub(hipPos, pelvisPos);
        raw = ProjectOntoPlane(raw, legDown);
        return SafeNormalise(raw, fallbackSide);
    }

    static CVector BuildLegTargetDir(
        const CVector& legDown,
        const CVector& legSide,
        float sideAmount)
    {
        CVector target = VecAdd(
            SafeNormalise(legDown, CVector(0.0f, 0.0f, -1.0f)),
            VecScale(SafeNormalise(legSide, CVector(1.0f, 0.0f, 0.0f)), sideAmount)
        );

        return SafeNormalise(target, legDown);
    }

    static void ApplyPlayerTPose(CPed* ped, RpHAnimHierarchy* hierarchy) {
        if (!ped || !hierarchy)
            return;

        if (ped != FindPlayerPed())
            return;

        // Center chain
        RwMatrix* pelvis = GetBoneMatrixById(hierarchy, BONE_PELVIS);
        RwMatrix* spine1 = GetBoneMatrixById(hierarchy, BONE_SPINE1);
        RwMatrix* torso = GetBoneMatrixById(hierarchy, BONE_UPPERTORSO);
        RwMatrix* neck = GetBoneMatrixById(hierarchy, BONE_NECK);
        RwMatrix* head = GetBoneMatrixById(hierarchy, BONE_HEAD);

        // Arms
        RwMatrix* lShoulder = GetBoneMatrixById(hierarchy, BONE_LEFTSHOULDER);
        RwMatrix* lElbow = GetBoneMatrixById(hierarchy, BONE_LEFTELBOW);
        RwMatrix* lWrist = GetBoneMatrixById(hierarchy, BONE_LEFTWRIST);
        RwMatrix* lHand = GetBoneMatrixById(hierarchy, BONE_LEFTHAND);

        RwMatrix* rShoulder = GetBoneMatrixById(hierarchy, BONE_RIGHTSHOULDER);
        RwMatrix* rElbow = GetBoneMatrixById(hierarchy, BONE_RIGHTELBOW);
        RwMatrix* rWrist = GetBoneMatrixById(hierarchy, BONE_RIGHTWRIST);
        RwMatrix* rHand = GetBoneMatrixById(hierarchy, BONE_RIGHTHAND);

        // Legs
        RwMatrix* lHip = GetBoneMatrixById(hierarchy, BONE_LEFTHIP);
        RwMatrix* lKnee = GetBoneMatrixById(hierarchy, BONE_LEFTKNEE);
        RwMatrix* lAnkle = GetBoneMatrixById(hierarchy, BONE_LEFTANKLE);
        RwMatrix* lFoot = GetBoneMatrixById(hierarchy, BONE_LEFTFOOT);

        RwMatrix* rHip = GetBoneMatrixById(hierarchy, BONE_RIGHTHIP);
        RwMatrix* rKnee = GetBoneMatrixById(hierarchy, BONE_RIGHTKNEE);
        RwMatrix* rAnkle = GetBoneMatrixById(hierarchy, BONE_RIGHTANKLE);
        RwMatrix* rFoot = GetBoneMatrixById(hierarchy, BONE_RIGHTFOOT);

        if (!pelvis || !spine1 || !torso || !neck || !head)
            return;

        if (!lShoulder || !lElbow || !lWrist || !lHand)
            return;
        if (!rShoulder || !rElbow || !rWrist || !rHand)
            return;

        if (!lHip || !lKnee || !lAnkle || !lFoot)
            return;
        if (!rHip || !rKnee || !rAnkle || !rFoot)
            return;

        const CVector pelvisPos = GetMatrixPos(pelvis);
        const CVector spinePos = GetMatrixPos(spine1);
        const CVector torsoPos = GetMatrixPos(torso);
        const CVector neckPos = GetMatrixPos(neck);
        const CVector headPos = GetMatrixPos(head);

        // Build a stable "up" from the current center chain itself.
        const CVector chainUp = SafeNormalise(
            VecAdd(
                VecAdd(
                    SafeNormalise(VecSub(spinePos, pelvisPos), CVector(0.0f, 0.0f, 1.0f)),
                    SafeNormalise(VecSub(torsoPos, spinePos), CVector(0.0f, 0.0f, 1.0f))
                ),
                VecAdd(
                    SafeNormalise(VecSub(neckPos, torsoPos), CVector(0.0f, 0.0f, 1.0f)),
                    SafeNormalise(VecSub(headPos, neckPos), CVector(0.0f, 0.0f, 1.0f))
                )
            ),
            CVector(0.0f, 0.0f, 1.0f)
        );

        CVector forwardHint = SafeNormalise(GetMatrixAt(torso), CVector(0.0f, 1.0f, 0.0f));
        forwardHint = SafeNormalise(ProjectOntoPlane(forwardHint, chainUp), CVector(0.0f, 1.0f, 0.0f));
        const CVector rightHint = SafeNormalise(VecCross(forwardHint, chainUp), CVector(1.0f, 0.0f, 0.0f));

        // Center chain solved the same way.
        SolveLinearChain5(
            pelvis, spine1, torso, neck, head,
            chainUp,
            forwardHint
        );

        const CVector torsoPos2 = GetMatrixPos(torso);
        const CVector pelvisPos2 = GetMatrixPos(pelvis);

        // Arms solved the same way.
        const CVector lArmSide = BuildArmSideTarget(
            torsoPos2,
            GetMatrixPos(lShoulder),
            chainUp,
            VecScale(rightHint, -1.0f)
        );

        const CVector rArmSide = BuildArmSideTarget(
            torsoPos2,
            GetMatrixPos(rShoulder),
            chainUp,
            rightHint
        );

        SolveLinearChain4(
            lShoulder, lElbow, lWrist, lHand,
            lArmSide,
            forwardHint
        );

        SolveLinearChain4(
            rShoulder, rElbow, rWrist, rHand,
            rArmSide,
            forwardHint
        );

        // Legs solved the same way.
        const CVector lHipPos = GetMatrixPos(lHip);
        const CVector lKneePos = GetMatrixPos(lKnee);
        const CVector rHipPos = GetMatrixPos(rHip);
        const CVector rKneePos = GetMatrixPos(rKnee);

        const CVector stableLegDown = BuildStableLegDown(
            lHipPos, lKneePos,
            rHipPos, rKneePos,
            VecScale(chainUp, -1.0f)
        );

        const CVector lLegSide = BuildLegSideTarget(
            pelvisPos2,
            lHipPos,
            stableLegDown,
            VecScale(rightHint, -1.0f)
        );

        const CVector rLegSide = BuildLegSideTarget(
            pelvisPos2,
            rHipPos,
            stableLegDown,
            rightHint
        );

        const CVector lLegDir = BuildLegTargetDir(stableLegDown, lLegSide, 0.04f);
        const CVector rLegDir = BuildLegTargetDir(stableLegDown, rLegSide, 0.04f);

        SolveLinearChain4(
            lHip, lKnee, lAnkle, lFoot,
            lLegDir,
            forwardHint
        );

        SolveLinearChain4(
            rHip, rKnee, rAnkle, rFoot,
            rLegDir,
            forwardHint
        );
    }

    static bool NeedGTAUpdate(CEntity* entity) {
        if (!entity)
            return true;

        if (entity->m_nType != ENTITY_TYPE_PED)
            return true;

        if (!entity->m_pRwClump)
            return true;

        if (!gForceTPose)
            return true;

        CPed* ped = reinterpret_cast<CPed*>(entity);
        if (ped != FindPlayerPed())
            return true;

        return false;
    }

    static void RunGameUpdateRpHAnim(CEntity* entity) {
        if (!entity)
            return;

        RpClump* clump = entity->m_pRwClump;
        if (!clump)
            return;

        RpAtomic* atomic = GetFirstAtomic(clump);
        if (!atomic)
            return;

        RpGeometry* geometry = RpAtomicGetGeometry(atomic);
        if (!geometry)
            return;

        if (!RpSkinGeometryGetSkin(geometry))
            return;

        if (entity->bDontUpdateHierarchy)
            return;

        RpHAnimHierarchy* hierarchy = GetSkinHierarchy(clump);
        if (!hierarchy)
            return;

        RpHAnimHierarchyUpdateMatrices(hierarchy);
    }

    static void RunCustomUpdateRpHAnim(CEntity* entity) {
        if (!entity)
            return;

        if (entity->m_nType != ENTITY_TYPE_PED)
            return;

        RpClump* clump = entity->m_pRwClump;
        if (!clump)
            return;

        RpAtomic* atomic = GetFirstAtomic(clump);
        if (!atomic)
            return;

        RpGeometry* geometry = RpAtomicGetGeometry(atomic);
        if (!geometry)
            return;

        if (!RpSkinGeometryGetSkin(geometry))
            return;

        if (entity->bDontUpdateHierarchy)
            return;

        RpHAnimHierarchy* hierarchy = GetSkinHierarchy(clump);
        if (!hierarchy)
            return;

        RpHAnimHierarchyUpdateMatrices(hierarchy);

        CPed* ped = reinterpret_cast<CPed*>(entity);
        ApplyPlayerTPose(ped, hierarchy);
    }

    static void __fastcall Hooked_UpdateRpHAnim(CEntity* self, int) {
        if (!self)
            return;

        if (NeedGTAUpdate(self))
            RunGameUpdateRpHAnim(self);
        else
            RunCustomUpdateRpHAnim(self);
    }

    static void DrawPedBoneLabels(CPed* ped) {
        if (!ped || !ped->m_pRwClump)
            return;

        for (unsigned int i = 0; i < sizeof(gBoneLabels) / sizeof(gBoneLabels[0]); ++i) {
            RwV3d boneWorld{};
            ped->GetBonePosition(boneWorld, gBoneLabels[i].id, true);

            RwV3d screen{};
            float w = 0.0f;
            float h = 0.0f;

            if (!CSprite::CalcScreenCoors(boneWorld, &screen, &w, &h, true, true))
                continue;

            CFont::SetBackground(false, false);
            CFont::SetScale(0.35f, 0.8f);
            CFont::SetJustify(false);
            CFont::SetOrientation(ALIGN_LEFT);
            CFont::SetProportional(true);
            CFont::SetFontStyle(FONT_MENU);
            CFont::SetColor(CRGBA(255, 80, 80, 255));
            CFont::SetDropColor(CRGBA(0, 0, 0, 255));
            CFont::SetDropShadowPosition(1);
            CFont::PrintString(screen.x, screen.y, gBoneLabels[i].name);
        }
    }

    class RagdollPrototype {
    public:
        RagdollPrototype() {
            patch::RedirectJump(0x532B20, Hooked_UpdateRpHAnim);

            Events::gameProcessEvent += [] {
                if (KeyPressed(VK_F5))
                    gDrawBones = !gDrawBones;

                if (KeyPressed(VK_F6))
                    gForceTPose = !gForceTPose;
                };

            Events::drawingEvent += [] {
                if (!gDrawBones)
                    return;

                CPed* player = FindPlayerPed();
                if (!player)
                    return;

                DrawPedBoneLabels(player);
                };
        }
    };

    static RagdollPrototype gRagdollPrototype;

} // namespace ragdoll_debug
