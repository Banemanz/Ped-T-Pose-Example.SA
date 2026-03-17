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

    static void RotatePointAroundPivotRad(CVector& point, const CVector& pivot, const CVector& axis, float angleRad) {
        const CVector rel = VecSub(point, pivot);
        const CVector rotated = RotateVectorAroundAxisRad(rel, axis, angleRad);
        point = VecAdd(pivot, rotated);
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

    static void RotateLimbChainRigid(
        RwMatrix* joint,
        RwMatrix* child,
        RwMatrix* next,
        RwMatrix* end,
        const CVector& axis,
        float angleRad)
    {
        if (!joint || !child || !next || !end)
            return;

        CVector jointPos = GetMatrixPos(joint);
        CVector childPos = GetMatrixPos(child);
        CVector nextPos = GetMatrixPos(next);
        CVector endPos = GetMatrixPos(end);

        RotateMatrixBasisAroundAxisRad(joint, axis, angleRad);
        RotateMatrixBasisAroundAxisRad(child, axis, angleRad);
        RotateMatrixBasisAroundAxisRad(next, axis, angleRad);
        RotateMatrixBasisAroundAxisRad(end, axis, angleRad);

        RotatePointAroundPivotRad(childPos, jointPos, axis, angleRad);
        RotatePointAroundPivotRad(nextPos, jointPos, axis, angleRad);
        RotatePointAroundPivotRad(endPos, jointPos, axis, angleRad);

        SetMatrixPos(joint, jointPos);
        SetMatrixPos(child, childPos);
        SetMatrixPos(next, nextPos);
        SetMatrixPos(end, endPos);

        RwMatrixUpdate(joint);
        RwMatrixUpdate(child);
        RwMatrixUpdate(next);
        RwMatrixUpdate(end);
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

    static void ApplyLimbTPoseDirect(
        RwMatrix* joint,
        RwMatrix* child,
        RwMatrix* next,
        RwMatrix* end,
        const CVector& desiredDirRaw,
        const CVector& fallbackAxisRaw)
    {
        if (!joint || !child || !next || !end)
            return;

        const CVector jointPos = GetMatrixPos(joint);
        const CVector childPos = GetMatrixPos(child);
        const CVector upperVec = VecSub(childPos, jointPos);

        if (VecLenSq(upperVec) <= 0.000001f)
            return;

        const CVector currentDir = SafeNormalise(upperVec, CVector(0.0f, -1.0f, 0.0f));
        const CVector desiredDir = SafeNormalise(desiredDirRaw, CVector(0.0f, -1.0f, 0.0f));

        float dot = ClampUnit(VecDot(currentDir, desiredDir));
        float angleRad = std::acos(dot); // radians

        CVector axis = VecCross(currentDir, desiredDir);

        if (VecLenSq(axis) <= 0.000001f) {
            axis = fallbackAxisRaw;

            if (dot > 0.9999f)
                angleRad = 0.0f;
            else
                angleRad = 3.14159265358979323846f;
        }

        axis = SafeNormalise(axis, fallbackAxisRaw);

        RotateLimbChainRigid(joint, child, next, end, axis, angleRad);
    }

    static void ApplyPlayerTPose(CPed* ped, RpHAnimHierarchy* hierarchy) {
        if (!ped || !hierarchy)
            return;

        if (ped != FindPlayerPed())
            return;

        RwMatrix* torso = GetBoneMatrixById(hierarchy, BONE_UPPERTORSO);
        RwMatrix* pelvis = GetBoneMatrixById(hierarchy, BONE_PELVIS);

        RwMatrix* lShoulder = GetBoneMatrixById(hierarchy, BONE_LEFTSHOULDER);
        RwMatrix* lElbow = GetBoneMatrixById(hierarchy, BONE_LEFTELBOW);
        RwMatrix* lWrist = GetBoneMatrixById(hierarchy, BONE_LEFTWRIST);
        RwMatrix* lHand = GetBoneMatrixById(hierarchy, BONE_LEFTHAND);

        RwMatrix* rShoulder = GetBoneMatrixById(hierarchy, BONE_RIGHTSHOULDER);
        RwMatrix* rElbow = GetBoneMatrixById(hierarchy, BONE_RIGHTELBOW);
        RwMatrix* rWrist = GetBoneMatrixById(hierarchy, BONE_RIGHTWRIST);
        RwMatrix* rHand = GetBoneMatrixById(hierarchy, BONE_RIGHTHAND);

        RwMatrix* lHip = GetBoneMatrixById(hierarchy, BONE_LEFTHIP);
        RwMatrix* lKnee = GetBoneMatrixById(hierarchy, BONE_LEFTKNEE);
        RwMatrix* lAnkle = GetBoneMatrixById(hierarchy, BONE_LEFTANKLE);
        RwMatrix* lFoot = GetBoneMatrixById(hierarchy, BONE_LEFTFOOT);

        RwMatrix* rHip = GetBoneMatrixById(hierarchy, BONE_RIGHTHIP);
        RwMatrix* rKnee = GetBoneMatrixById(hierarchy, BONE_RIGHTKNEE);
        RwMatrix* rAnkle = GetBoneMatrixById(hierarchy, BONE_RIGHTANKLE);
        RwMatrix* rFoot = GetBoneMatrixById(hierarchy, BONE_RIGHTFOOT);

        if (!torso || !pelvis)
            return;

        if (!lShoulder || !lElbow || !lWrist || !lHand)
            return;
        if (!rShoulder || !rElbow || !rWrist || !rHand)
            return;

        if (!lHip || !lKnee || !lAnkle || !lFoot)
            return;
        if (!rHip || !rKnee || !rAnkle || !rFoot)
            return;

        const CVector torsoPos = GetMatrixPos(torso);
        const CVector pelvisPos = GetMatrixPos(pelvis);

        const CVector torsoUp = SafeNormalise(GetMatrixUp(torso), CVector(0.0f, 0.0f, 1.0f));
        const CVector torsoAt = SafeNormalise(GetMatrixAt(torso), CVector(0.0f, 1.0f, 0.0f));

        // Arms: same working solve as before.
        const CVector lArmSide = BuildArmSideTarget(
            torsoPos,
            GetMatrixPos(lShoulder),
            torsoUp,
            CVector(-1.0f, 0.0f, 0.0f)
        );

        const CVector rArmSide = BuildArmSideTarget(
            torsoPos,
            GetMatrixPos(rShoulder),
            torsoUp,
            CVector(1.0f, 0.0f, 0.0f)
        );

        ApplyLimbTPoseDirect(
            lShoulder, lElbow, lWrist, lHand,
            lArmSide,
            torsoAt
        );

        ApplyLimbTPoseDirect(
            rShoulder, rElbow, rWrist, rHand,
            rArmSide,
            torsoAt
        );

        // Legs: derive "down" from the current upper-leg directions instead of torso axes.
        const CVector lHipPos = GetMatrixPos(lHip);
        const CVector lKneePos = GetMatrixPos(lKnee);
        const CVector rHipPos = GetMatrixPos(rHip);
        const CVector rKneePos = GetMatrixPos(rKnee);

        const CVector stableLegDown = BuildStableLegDown(
            lHipPos, lKneePos,
            rHipPos, rKneePos,
            VecScale(torsoUp, -1.0f)
        );

        const CVector lLegSide = BuildLegSideTarget(
            pelvisPos,
            lHipPos,
            stableLegDown,
            CVector(-1.0f, 0.0f, 0.0f)
        );

        const CVector rLegSide = BuildLegSideTarget(
            pelvisPos,
            rHipPos,
            stableLegDown,
            CVector(1.0f, 0.0f, 0.0f)
        );

        // Small spread only. Bigger values start turning the legs into a split.
        const CVector lLegDir = BuildLegTargetDir(stableLegDown, lLegSide, 0.08f);
        const CVector rLegDir = BuildLegTargetDir(stableLegDown, rLegSide, 0.08f);

        ApplyLimbTPoseDirect(
            lHip, lKnee, lAnkle, lFoot,
            lLegDir,
            torsoAt
        );

        ApplyLimbTPoseDirect(
            rHip, rKnee, rAnkle, rFoot,
            rLegDir,
            torsoAt
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
