/*
    Plugin-SDK (Grand Theft Auto) source file
    Authors: GTA Community. See more here
    https://github.com/DK22Pac/plugin-sdk
    Do not delete this comment block. Respect others' work!
*/

#include <plugin.h>
#include <CPed.h>
#include <CPools.h>
#include <CTimer.h>
#include <CWorld.h>
#include <ePedState.h>
#include <ePedBones.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

using namespace plugin;

namespace {
    constexpr float kGravity = 0.020f;
    constexpr float kVelocityDamping = 0.990f;
    constexpr float kGroundBounce = 0.15f;
    constexpr float kGroundFriction = 0.80f;
    constexpr int   kSolverIterations = 6;
    constexpr float kNodeCollisionRadius = 0.045f;

    enum RagNodeId {
        NODE_PELVIS,
        NODE_SPINE,
        NODE_NECK,
        NODE_HEAD,
        NODE_L_SHOULDER,
        NODE_L_ELBOW,
        NODE_L_HAND,
        NODE_R_SHOULDER,
        NODE_R_ELBOW,
        NODE_R_HAND,
        NODE_L_HIP,
        NODE_L_KNEE,
        NODE_L_FOOT,
        NODE_R_HIP,
        NODE_R_KNEE,
        NODE_R_FOOT,
        NODE_COUNT
    };

    struct Node {
        CVector pos{};
        CVector prevPos{};
        float invMass{};
    };

    struct Edge {
        unsigned short a{};
        unsigned short b{};
        float restLen{};
        float stiffness{};
    };

    struct BoneSource {
        unsigned short ragNode{};
        unsigned short pedBone{};
    };

    struct PedRagdoll {
        CPed* ped{};
        int pedRef{ -1 };
        bool active{};
        std::array<Node, NODE_COUNT> nodes{};
        std::vector<Edge> edges;
        unsigned int lastTouchedFrame{};
    };

    constexpr std::array<BoneSource, NODE_COUNT> kBoneMap = { {
        { NODE_PELVIS, BONE_PELVIS },
        { NODE_SPINE, BONE_SPINE1 },
        { NODE_NECK, BONE_NECK },
        { NODE_HEAD, BONE_HEAD },
        { NODE_L_SHOULDER, BONE_LEFTSHOULDER },
        { NODE_L_ELBOW, BONE_LEFTELBOW },
        { NODE_L_HAND, BONE_LEFTHAND },
        { NODE_R_SHOULDER, BONE_RIGHTSHOULDER },
        { NODE_R_ELBOW, BONE_RIGHTELBOW },
        { NODE_R_HAND, BONE_RIGHTHAND },
        { NODE_L_HIP, BONE_LEFTHIP },
        { NODE_L_KNEE, BONE_LEFTKNEE },
        { NODE_L_FOOT, BONE_LEFTFOOT },
        { NODE_R_HIP, BONE_RIGHTHIP },
        { NODE_R_KNEE, BONE_RIGHTKNEE },
        { NODE_R_FOOT, BONE_RIGHTFOOT }
    } };

    static std::vector<PedRagdoll> g_ragdolls;

    void AddEdge(std::vector<Edge>& edges, RagNodeId a, RagNodeId b, float restLen, float stiffness) {
        edges.push_back({ static_cast<unsigned short>(a), static_cast<unsigned short>(b), restLen, stiffness });
    }

    float Distance(const CVector& a, const CVector& b) {
        const CVector d = b - a;
        return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    }

    bool IsDeadEnough(const CPed* ped) {
        return ped && (ped->m_fHealth <= 0.0f || ped->m_ePedState == PEDSTATE_DIE || ped->m_ePedState == PEDSTATE_DEAD || ped->m_ePedState == PEDSTATE_DIE_BY_STEALTH);
    }

    void BuildEdgeRig(PedRagdoll& ragdoll) {
        auto& n = ragdoll.nodes;
        auto& e = ragdoll.edges;
        e.clear();

        AddEdge(e, NODE_PELVIS, NODE_SPINE, Distance(n[NODE_PELVIS].pos, n[NODE_SPINE].pos), 0.90f);
        AddEdge(e, NODE_SPINE, NODE_NECK, Distance(n[NODE_SPINE].pos, n[NODE_NECK].pos), 0.90f);
        AddEdge(e, NODE_NECK, NODE_HEAD, Distance(n[NODE_NECK].pos, n[NODE_HEAD].pos), 0.85f);

        AddEdge(e, NODE_NECK, NODE_L_SHOULDER, Distance(n[NODE_NECK].pos, n[NODE_L_SHOULDER].pos), 0.85f);
        AddEdge(e, NODE_L_SHOULDER, NODE_L_ELBOW, Distance(n[NODE_L_SHOULDER].pos, n[NODE_L_ELBOW].pos), 0.85f);
        AddEdge(e, NODE_L_ELBOW, NODE_L_HAND, Distance(n[NODE_L_ELBOW].pos, n[NODE_L_HAND].pos), 0.85f);

        AddEdge(e, NODE_NECK, NODE_R_SHOULDER, Distance(n[NODE_NECK].pos, n[NODE_R_SHOULDER].pos), 0.85f);
        AddEdge(e, NODE_R_SHOULDER, NODE_R_ELBOW, Distance(n[NODE_R_SHOULDER].pos, n[NODE_R_ELBOW].pos), 0.85f);
        AddEdge(e, NODE_R_ELBOW, NODE_R_HAND, Distance(n[NODE_R_ELBOW].pos, n[NODE_R_HAND].pos), 0.85f);

        AddEdge(e, NODE_PELVIS, NODE_L_HIP, Distance(n[NODE_PELVIS].pos, n[NODE_L_HIP].pos), 0.92f);
        AddEdge(e, NODE_L_HIP, NODE_L_KNEE, Distance(n[NODE_L_HIP].pos, n[NODE_L_KNEE].pos), 0.90f);
        AddEdge(e, NODE_L_KNEE, NODE_L_FOOT, Distance(n[NODE_L_KNEE].pos, n[NODE_L_FOOT].pos), 0.90f);

        AddEdge(e, NODE_PELVIS, NODE_R_HIP, Distance(n[NODE_PELVIS].pos, n[NODE_R_HIP].pos), 0.92f);
        AddEdge(e, NODE_R_HIP, NODE_R_KNEE, Distance(n[NODE_R_HIP].pos, n[NODE_R_KNEE].pos), 0.90f);
        AddEdge(e, NODE_R_KNEE, NODE_R_FOOT, Distance(n[NODE_R_KNEE].pos, n[NODE_R_FOOT].pos), 0.90f);

        AddEdge(e, NODE_L_SHOULDER, NODE_R_SHOULDER, Distance(n[NODE_L_SHOULDER].pos, n[NODE_R_SHOULDER].pos), 0.80f);
        AddEdge(e, NODE_L_HIP, NODE_R_HIP, Distance(n[NODE_L_HIP].pos, n[NODE_R_HIP].pos), 0.80f);
        AddEdge(e, NODE_L_HAND, NODE_R_HAND, Distance(n[NODE_L_HAND].pos, n[NODE_R_HAND].pos), 0.25f);
    }

    bool TryCaptureInitialPose(CPed* ped, PedRagdoll& ragdoll) {
        if (!ped || !ped->m_pRwClump)
            return false;

        for (const BoneSource& bone : kBoneMap) {
            RwV3d p{};
            ped->GetBonePosition(p, bone.pedBone, true);
            Node& n = ragdoll.nodes[bone.ragNode];
            n.pos = { p.x, p.y, p.z };
            n.prevPos = n.pos;
            n.invMass = 1.0f;
        }

        ragdoll.nodes[NODE_HEAD].invMass = 0.8f;
        ragdoll.nodes[NODE_PELVIS].invMass = 0.6f;
        ragdoll.nodes[NODE_L_FOOT].invMass = 0.7f;
        ragdoll.nodes[NODE_R_FOOT].invMass = 0.7f;

        BuildEdgeRig(ragdoll);
        return true;
    }

    PedRagdoll* FindRagdoll(CPed* ped) {
        auto it = std::find_if(g_ragdolls.begin(), g_ragdolls.end(), [ped](const PedRagdoll& r) {
            return r.ped == ped;
            });
        return it != g_ragdolls.end() ? &*it : nullptr;
    }

    bool SolveGround(Node& n) {
        bool hit = false;
        bool gotGround = false;
        CEntity* ignored{};
        const float groundZ = CWorld::FindGroundZFor3DCoord(n.pos.x, n.pos.y, n.pos.z + 2.0f, &gotGround, &ignored);

        if (gotGround) {
            const float floor = groundZ + kNodeCollisionRadius;
            if (n.pos.z < floor) {
                n.pos.z = floor;

                CVector vel = (n.pos - n.prevPos);
                vel.z = -vel.z * kGroundBounce;
                vel.x *= kGroundFriction;
                vel.y *= kGroundFriction;
                n.prevPos = n.pos - vel;
                hit = true;
            }
        }

        return hit;
    }

    void Simulate(PedRagdoll& ragdoll, float dt) {
        const float timeScale = std::max(0.5f, std::min(dt, 2.0f));

        for (Node& n : ragdoll.nodes) {
            CVector vel = (n.pos - n.prevPos) * kVelocityDamping;
            n.prevPos = n.pos;
            n.pos += vel;
            n.pos.z -= kGravity * timeScale;
        }

        for (int i = 0; i < kSolverIterations; ++i) {
            for (const Edge& edge : ragdoll.edges) {
                Node& a = ragdoll.nodes[edge.a];
                Node& b = ragdoll.nodes[edge.b];

                CVector delta = b.pos - a.pos;
                const float lenSq = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
                if (lenSq < 0.000001f)
                    continue;

                const float len = std::sqrt(lenSq);
                const float error = (len - edge.restLen) / len;
                const CVector corr = delta * (0.5f * edge.stiffness * error);

                a.pos += corr;
                b.pos -= corr;
            }

            for (Node& n : ragdoll.nodes)
                SolveGround(n);
        }
    }

    void RenderRagdollLines(const PedRagdoll& ragdoll) {
        if (ragdoll.edges.empty())
            return;

        std::vector<RwIm3DVertex> verts(ragdoll.edges.size() * 2);
        unsigned int idx = 0;

        for (const Edge& edge : ragdoll.edges) {
            const CVector& a = ragdoll.nodes[edge.a].pos;
            const CVector& b = ragdoll.nodes[edge.b].pos;

            RwIm3DVertexSetPos(&verts[idx], a.x, a.y, a.z);
            RwIm3DVertexSetRGBA(&verts[idx], 220, 220, 255, 255);
            ++idx;

            RwIm3DVertexSetPos(&verts[idx], b.x, b.y, b.z);
            RwIm3DVertexSetRGBA(&verts[idx], 220, 220, 255, 255);
            ++idx;
        }

        if (RwIm3DTransform(verts.data(), static_cast<unsigned int>(verts.size()), nullptr, rwIM3D_ALLOPAQUE)) {
            for (unsigned int i = 0; i < verts.size(); i += 2)
                RwIm3DRenderLine(i, i + 1);
            RwIm3DEnd();
        }
    }

    void TickRagdolls() {
        CPool<CPed, CCopPed>* pedPool = CPools::ms_pPedPool;
        if (!pedPool)
            return;

        for (int i = 0; i < pedPool->m_nSize; ++i) {
            CPed* ped = pedPool->GetAt(i);
            if (!ped)
                continue;

            if (!IsDeadEnough(ped))
                continue;

            PedRagdoll* found = FindRagdoll(ped);
            if (!found) {
                PedRagdoll created{};
                created.ped = ped;
                created.pedRef = CPools::GetPedRef(ped);
                created.active = TryCaptureInitialPose(ped, created);
                created.lastTouchedFrame = CTimer::m_FrameCounter;

                if (created.active)
                    g_ragdolls.push_back(created);
            }
            else {
                found->lastTouchedFrame = CTimer::m_FrameCounter;
            }
        }

        g_ragdolls.erase(std::remove_if(g_ragdolls.begin(), g_ragdolls.end(), [](PedRagdoll& ragdoll) {
            CPed* ped = CPools::GetPed(ragdoll.pedRef);
            if (!ped)
                return true;

            if (CTimer::m_FrameCounter - ragdoll.lastTouchedFrame > 240) {
                ped->bDontRender = false;
                return true;
            }

            const float dt = CTimer::ms_fTimeStep;
            Simulate(ragdoll, dt);

            ped->bDontRender = true;
            return false;
            }), g_ragdolls.end());
    }

    void DrawRagdolls() {
        RwRenderStateSet(rwRENDERSTATEZWRITEENABLE, reinterpret_cast<void*>(TRUE));
        RwRenderStateSet(rwRENDERSTATEZTESTENABLE, reinterpret_cast<void*>(TRUE));

        for (const PedRagdoll& ragdoll : g_ragdolls)
            RenderRagdollLines(ragdoll);
    }
}

class InhouseRagdollPlugin {
public:
    InhouseRagdollPlugin() {
        Events::gameProcessEvent += [] {
            TickRagdolls();
            };

        Events::drawingEvent += [] {
            DrawRagdolls();
            };

        Events::pedRenderEvent.after += [](CPed* ped) {
            if (!ped)
                return;

            if (PedRagdoll* ragdoll = FindRagdoll(ped)) {
                if (ragdoll->active)
                    ped->bDontRender = true;
            }
            };
    }
} g_inhouseRagdollPlugin;
