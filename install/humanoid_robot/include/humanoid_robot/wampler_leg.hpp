#pragma once
// ============================================================================
//  wampler_leg.hpp
//  Wampler 1986 DLS-IK for one 5-joint humanoid leg. Zero ROS dependency.
//
//  Reference:
//    C.W. Wampler II, "Manipulator Inverse Kinematic Solutions Based on
//    Vector Formulations and Damped Least-Squares Methods,"
//    IEEE Trans. Syst. Man. Cybern., SMC-16(1):93-101, 1986.
//
//  Three practical improvements from the paper implemented here:
//
//  ① RECURSIVE POSITION VECTORS  (§"Computing the Elements", Eq. 21)
//       p^(R/Q_{i-1}) built backward from foot using DCM chain.
//       In our world-frame formulation this reduces to p_foot − Q[i],
//       which is the world-frame equivalent of Eq. 16 corollary.
//
//  ② P = J̃J̃ᵀ via Eq. 47, upper-triangle-only computation
//       Pᵢⱼ = vᵢᴿ · vⱼᴿ  (position-only task, ωᵢᵀ terms absent)
//       Symmetric → only 6 of 9 elements computed.
//
//  ③ CHOLESKY SOLVE of (P + α²I)q̇ = J̃ᵀê  (Eq. 48)
//       P + α²I is symmetric positive-definite by construction.
//       More stable than cofactor inversion; cost = O(n²) per solve.
//
//  Leg joint layout (all revolute, n=5, task-space m=3 position):
//    q[0]  hip_roll    axis X
//    q[1]  hip_pitch   axis Y
//    q[2]  knee_pitch  axis Y
//    q[3]  ankle_pitch axis Y   ← overridden by foot-flat constraint in node
//    q[4]  ankle_roll  axis X
// ============================================================================

#include <array>
#include <cmath>
#include <algorithm>

// ─────────────────────────────────────────────────────────────────────────────
//  Fixed-size types
// ─────────────────────────────────────────────────────────────────────────────
using Vec3  = std::array<double, 3>;
using Vec5  = std::array<double, 5>;
using Mat33 = std::array<std::array<double,3>, 3>;
using DCM   = std::array<std::array<double,3>, 3>;   // 3×3 rotation matrix

// ─────────────────────────────────────────────────────────────────────────────
//  Vector math
// ─────────────────────────────────────────────────────────────────────────────
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return { a[1]*b[2]-a[2]*b[1],
             a[2]*b[0]-a[0]*b[2],
             a[0]*b[1]-a[1]*b[0] };
}
inline double dot3(const Vec3& a, const Vec3& b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
inline Vec3 vadd(const Vec3& a, const Vec3& b) {
    return { a[0]+b[0], a[1]+b[1], a[2]+b[2] };
}
inline Vec3 vsub(const Vec3& a, const Vec3& b) {
    return { a[0]-b[0], a[1]-b[1], a[2]-b[2] };
}
inline Vec3 vscale(double s, const Vec3& v) {
    return { s*v[0], s*v[1], s*v[2] };
}
inline double vnorm(const Vec3& v) { return std::sqrt(dot3(v,v)); }

// ─────────────────────────────────────────────────────────────────────────────
//  Direction Cosine Matrices  (Wampler Eq. 22)
//  ⁱC^(i+1) relates consecutive link frames.
//  For our leg, joints are either X-rotation (roll) or Y-rotation (pitch).
// ─────────────────────────────────────────────────────────────────────────────
inline DCM rotX_dcm(double a) {
    double c=std::cos(a), s=std::sin(a);
    return {{ {1,0,0}, {0,c,-s}, {0,s,c} }};
}
inline DCM rotY_dcm(double a) {
    double c=std::cos(a), s=std::sin(a);
    return {{ {c,0,s}, {0,1,0}, {-s,0,c} }};
}

// Apply local rotation R_local to accumulated R:  R_new = R_local * R
inline DCM apply_local_rot(const DCM& R_local, const DCM& R) {
    DCM out{};
    for (int i=0;i<3;++i)
        for (int j=0;j<3;++j)
            for (int k=0;k<3;++k)
                out[i][j] += R_local[i][k] * R[k][j];
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Cholesky solve for 3×3 SPD system  A x = b  (Wampler Eq. 48)
//
//  Decomposes A = L Lᵀ then solves by forward/back substitution.
//  Returns false only if A is not positive-definite (shouldn't happen with
//  the α²I bump, but we guard anyway).
// ─────────────────────────────────────────────────────────────────────────────
inline bool cholesky_solve3(const Mat33& A, const Vec3& b, Vec3& x)
{
    // Cholesky factorisation  A = L Lᵀ
    double v0 = A[0][0];
    if (v0 <= 0.0) return false;
    double L00 = std::sqrt(v0);

    double L10 = A[1][0] / L00;
    double v1  = A[1][1] - L10*L10;
    if (v1 <= 0.0) return false;
    double L11 = std::sqrt(v1);

    double L20 = A[2][0] / L00;
    double L21 = (A[2][1] - L20*L10) / L11;
    double v2  = A[2][2] - L20*L20 - L21*L21;
    if (v2 <= 0.0) return false;
    double L22 = std::sqrt(v2);

    // Forward substitution  L y = b
    double y0 =  b[0] / L00;
    double y1 = (b[1] - L10*y0) / L11;
    double y2 = (b[2] - L20*y0 - L21*y1) / L22;

    // Back substitution  Lᵀ x = y
    x[2] =  y2 / L22;
    x[1] = (y1 - L21*x[2]) / L11;
    x[0] = (y0 - L10*x[1] - L20*x[2]) / L00;

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  WamplerLeg
// ─────────────────────────────────────────────────────────────────────────────
struct WamplerLeg
{
    // ── Robot geometry ────────────────────────────────────────────────────────
    double L1{0.1225};   // thigh length [m]
    double L2{0.1230};   // shank length [m]

    // ── DLS parameters ────────────────────────────────────────────────────────
    // alpha: base damping factor α (Wampler Eq. 34, recommendation p.99: α≈0.03
    // for unit-scaled arms; scaled to our ~0.12m links → 0.01 is equivalent)
    double alpha    {0.01};
    // alpha_max: extra damping added at singularity (variable damping extension)
    double alpha_max{0.15};
    // w_thresh: manipulability threshold below which damping ramps up
    double w_thresh {0.005};

    int    max_iter{6};
    double tol     {0.001};  // convergence tolerance [m]

    // ── Current joint angles [rad], geometric convention (0 = straight leg) ──
    //   q[0]=hip_roll  q[1]=hip_pitch  q[2]=knee_pitch
    //   q[3]=ankle_pitch  q[4]=ankle_roll
    Vec5 q{0, 0, 0, 0, 0};

    // ════════════════════════════════════════════════════════════════════════
    //  STEP 1 — Forward chain
    //  Propagates joint axes zᵢ (world frame) and origins Qᵢ (world frame)
    //  through the kinematic chain using accumulated DCM rotations.
    //
    //  This builds the data Wampler's Eq. 21 needs:
    //    - zᵢ  (the rotation axis in base frame, used in Eq. 16)
    //    - Qᵢ  (joint origin, used to form p^(R/Q_{i-1}) = p_foot − Qᵢ)
    // ════════════════════════════════════════════════════════════════════════
    struct ChainData {
        Vec3 z[5];     // joint axes in world frame
        Vec3 Q[5];     // joint origins in world frame
        Vec3 p_foot;   // foot position in world frame
    };

    ChainData forward_chain(const Vec3& hip_world) const
    {
        ChainData c;

        // R tracks accumulated rotation (world → current link frame)
        // Columns of Rᵀ = world-frame expressions of local x, y, z axes
        DCM R = {{ {1,0,0}, {0,1,0}, {0,0,1} }};

        // Helper: extract world-frame expression of local axis
        // local X in world = column 0 of Rᵀ = row 0 of R transposed
        auto world_x = [&]() -> Vec3 { return { R[0][0], R[1][0], R[2][0] }; };
        auto world_y = [&]() -> Vec3 { return { R[0][1], R[1][1], R[2][1] }; };
        // Bone extends in local -Z direction
        auto bone_dir = [&]() -> Vec3 { return { -R[0][2], -R[1][2], -R[2][2] }; };

        Vec3 pos = hip_world;

        // ── Joint 0: hip_roll — axis = current local X ──────────────────────
        c.z[0] = world_x();
        c.Q[0] = pos;
        R = apply_local_rot(rotX_dcm(q[0]), R);

        // ── Joint 1: hip_pitch — axis = current local Y (same origin) ────────
        c.z[1] = world_y();
        c.Q[1] = pos;
        R = apply_local_rot(rotY_dcm(q[1]), R);

        // ── Thigh bone: translate L1 along local -Z ───────────────────────────
        Vec3 d1 = bone_dir();
        pos = vadd(pos, vscale(L1, d1));

        // ── Joint 2: knee_pitch — axis = current local Y ──────────────────────
        c.z[2] = world_y();
        c.Q[2] = pos;
        R = apply_local_rot(rotY_dcm(q[2]), R);

        // ── Shank bone: translate L2 along local -Z ───────────────────────────
        Vec3 d2 = bone_dir();
        pos = vadd(pos, vscale(L2, d2));

        // ── Joint 3: ankle_pitch — axis = current local Y ─────────────────────
        c.z[3] = world_y();
        c.Q[3] = pos;
        R = apply_local_rot(rotY_dcm(q[3]), R);

        // ── Joint 4: ankle_roll — axis = current local X (same origin) ────────
        c.z[4] = world_x();
        c.Q[4] = pos;
        // (no further translation — foot origin is at ankle)

        c.p_foot = pos;
        return c;
    }

    // ════════════════════════════════════════════════════════════════════════
    //  STEP 2 — Partial velocity matrix J̃  (Wampler Eq. 16 corollary)
    //
    //  For each revolute joint i:
    //    vᵢᴿ = zᵢ × p^(R/Q_{i-1})
    //
    //  p^(R/Q_{i-1}) in world frame = p_foot − Q[i]
    //  (This is the world-frame equivalent of Wampler's Eq. 21 recursion,
    //  which builds these vectors backward in link frames. In world frame
    //  the recursion reduces to a single subtraction per joint.)
    // ════════════════════════════════════════════════════════════════════════
    struct PartialVel {
        Vec3 v[5];   // vᵢᴿ = zᵢ × (p_foot − Q[i]),  i = 0..4
    };

    PartialVel partial_velocities(const ChainData& c) const
    {
        PartialVel pv;
        for (int i = 0; i < 5; ++i) {
            Vec3 r = vsub(c.p_foot, c.Q[i]);  // p^(R/Q_i) in world frame
            pv.v[i] = cross(c.z[i], r);        // Eq. 16 corollary
        }
        return pv;
    }

    // ════════════════════════════════════════════════════════════════════════
    //  STEP 3 — P = J̃J̃ᵀ  (Wampler Eq. 47)
    //
    //  Pᵢⱼ = vᵢᴿ · vⱼᴿ   (position-only: ωᵢᵀ terms are zero)
    //
    //  Key efficiency: P is 3×3 symmetric.
    //  Only the 6 upper-triangle elements are computed — the paper explicitly
    //  states "only elements in the upper triangle need be computed" (p.100).
    //  Lower triangle filled by symmetry.
    // ════════════════════════════════════════════════════════════════════════
    Mat33 build_P(const PartialVel& pv) const
    {
        Mat33 P{};
        for (int k = 0; k < 5; ++k) {
            // Upper triangle only (6 unique elements)
            P[0][0] += pv.v[k][0] * pv.v[k][0];
            P[0][1] += pv.v[k][0] * pv.v[k][1];
            P[0][2] += pv.v[k][0] * pv.v[k][2];
            P[1][1] += pv.v[k][1] * pv.v[k][1];
            P[1][2] += pv.v[k][1] * pv.v[k][2];
            P[2][2] += pv.v[k][2] * pv.v[k][2];
        }
        // Symmetry: copy upper → lower
        P[1][0]=P[0][1];
        P[2][0]=P[0][2];
        P[2][1]=P[1][2];
        return P;
    }

    // ════════════════════════════════════════════════════════════════════════
    //  STEP 4 — Adaptive damping factor α
    //
    //  Wampler Eq. 42 gives the condition number of (P + α²I):
    //    κ² = (σ₁² + α²) / (σₙ² + α²)
    //  Recommendation: α = 0.03 keeps κ < 1200 (p.99).
    //
    //  Variable extension (Nakamura 1987):
    //  Manipulability w = √det(P) → 0 at singularity.
    //  Scale α up as w → 0 to bound joint velocities smoothly.
    // ════════════════════════════════════════════════════════════════════════
    double adaptive_alpha(const Mat33& P) const
    {
        // det of 3×3 symmetric matrix (exploit symmetry: P[i][j]=P[j][i])
        double a=P[0][0], b=P[0][1], c=P[0][2];
        double e=P[1][1], f=P[1][2], k=P[2][2];
        double det = a*(e*k - f*f) - b*(b*k - c*f) + c*(b*f - e*c);
        double w = (det > 0.0) ? std::sqrt(det) : 0.0;

        if (w >= w_thresh) return alpha;
        double t = 1.0 - w / w_thresh;
        return alpha + alpha_max * t * t;
    }

    // ════════════════════════════════════════════════════════════════════════
    //  STEP 5 — DLS solve  (Wampler Eq. 37 + Eq. 48)
    //
    //  Full formula (Eq. 37):
    //    Δq = J̃ᵀ (P + α²I)⁻¹ e
    //
    //  Split into two steps as Wampler recommends (Eq. 48):
    //    A) Solve  (P + α²I) y = e    ← 3×3 Cholesky (SPD guaranteed)
    //    B) Δq = J̃ᵀ y                 ← 5 dot products of length 3
    //
    //  The dominant cost is the 3×3 solve — independent of n (joint count).
    //  This is Wampler's key insight: solving in task space (3×3) instead of
    //  joint space (5×5) makes the method scale with task DOF, not joint count.
    // ════════════════════════════════════════════════════════════════════════
    Vec5 dls_solve(const PartialVel& pv, const Mat33& P, const Vec3& e) const
    {
        // Build (P + α²I)
        double a_eff = adaptive_alpha(P);
        double a2 = a_eff * a_eff;
        Mat33 A = P;
        A[0][0]+=a2; A[1][1]+=a2; A[2][2]+=a2;

        // Step A: Cholesky solve (P + α²I) y = e  →  y
        Vec3 y{};
        if (!cholesky_solve3(A, e, y)) return {};   // guarded fallback

        // Step B: Δq = J̃ᵀ y  (each element = vᵢᴿ · y)
        Vec5 dq{};
        for (int i = 0; i < 5; ++i)
            dq[i] = dot3(pv.v[i], y);

        return dq;
    }

    // ════════════════════════════════════════════════════════════════════════
    //  PUBLIC — iterative IK solve
    //
    //  Runs Steps 1-5 repeatedly until ‖error‖ < tol or max_iter reached.
    //  Joint limits are clamped after every iteration.
    //  Returns true if converged within tolerance.
    // ════════════════════════════════════════════════════════════════════════
    bool solve(const Vec3& hip_world, const Vec3& foot_target,
               const Vec5& q_lo, const Vec5& q_hi)
    {
        for (int iter = 0; iter < max_iter; ++iter) {
            ChainData  c  = forward_chain(hip_world);         // Step 1
            Vec3       e  = vsub(foot_target, c.p_foot);      // error
            if (vnorm(e) < tol) return true;                   // converged
            PartialVel pv = partial_velocities(c);             // Step 2
            Mat33      P  = build_P(pv);                       // Step 3
            Vec5       dq = dls_solve(pv, P, e);               // Steps 4+5
            for (int i = 0; i < 5; ++i)
                q[i] = std::clamp(q[i]+dq[i], q_lo[i], q_hi[i]);
        }
        return false;
    }
};