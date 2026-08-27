#include "Settings.h"
#include "Joint.h"
#include "Utils.h"

Joint::Joint(JointSettings js) {
    type = js.type;

    a = js.a;
    b = js.b;
    pa = js.pa;
    pb = js.pb;

    rest_angle = js.rest_angle;
    
    target = js.target;
    max_force = js.max_force;

    key1 = js.key1;
    key2 = js.key2;

    hertz = js.hertz;
    damping = js.damping;
    rest_length = js.rest_length;

    axis_pos = js.axis_pos;
    axis_angle = js.axis_angle;

    // min max check to avoid useless bugs
    course_min = fminf(js.course_min, js.course_max);
    course_max = fmaxf(js.course_min, js.course_max);
}

JointSolve Joint::prepare(PixelBody *a, PixelBody *b, float h) {
    // axis joints anchor A on the axis origin, not on a pixel — pa is unused and may not exist
    bool axis_joint = (type == LIN_BEARING || type == PISTON);

    sf::Vector2f ra = Utils::rotVec(axis_joint ? axis_pos : a->offsets[pa.x][pa.y], a->getCS());
    sf::Vector2f rb = Utils::rotVec(b->offsets[pb.x][pb.y], b->getCS());

    sf::Vector2f anchor_a = a->pos + ra;
    sf::Vector2f anchor_b = b->pos + rb;
    sf::Vector2f C = anchor_a - anchor_b;
    float C_ang = a->ang - b->ang - rest_angle;

    float k00 = a->imass() + b->imass() + a->iinertia() * ra.y * ra.y + b->iinertia() * rb.y * rb.y;
    float k01 = - a->iinertia() * ra.x * ra.y - b->iinertia() * rb.x * rb.y;
    float k11 = a->imass() + b->imass() + a->iinertia() * ra.x * ra.x + b->iinertia() * rb.x * rb.x;
    
    float det = k00 * k11 - k01 * k01;
    float inv_det = (det > 1e-12f) ? 1.f / det : 0.f;

    sf::Vector2f bias = (beta / h) * C;
    float bl = bias.length();
    if (SIM_MAX_BIAS > 0.f && bl > SIM_MAX_BIAS) bias *= SIM_MAX_BIAS / bl;

    JointSolve s{ this, a, b, ra, rb, true, {}, 0, k00, k01, k11, inv_det, bias };

    switch (type) {
        case BEARING:
            break;                          // anchor block only

        case FIXED:
            s.rows[s.row_count++] = makeRow(a, b, {0.f, 0.f}, 1.f, 1.f, Utils::wrapPi(C_ang), h, R_ANG);
            break;

        case MOTOR: {
            JointSolve::Row r = makeRow(a, b, {0.f, 0.f}, 1.f, 1.f, 0.f, h, R_ANG);
            r.bias = -target;                 // velocity target, not position error
            r.lo   = -max_force * h;
            r.hi   =  max_force * h;
            s.rows[s.row_count++] = r;
            break;
        }

        case LIN_BEARING: {
            s.has_anchor = false;

            sf::Vector2f u = Utils::rotVec({std::cos(axis_angle), std::sin(axis_angle)}, a->getCS());
            sf::Vector2f t = u.perpendicular();

            s.rows[s.row_count++] = makeRow(a, b, t,
                                            (ra - C).cross(t), rb.cross(t),
                                            C.dot(t), h, R_PERP);
            s.rows[s.row_count++] = makeRow(a, b, {0.f, 0.f}, 1.f, 1.f,
                                            Utils::wrapPi(C_ang), h, R_ANG);

            float pos = -C.dot(u);              // B's coordinate along the axis
            float lo_C = pos - course_min;      // < 0 == past the near stop
            float hi_C = course_max - pos;

            if (lo_C < SIM_LIM_SLOP) {
                JointSolve::Row r = makeRow(a, b, -u,
                                            -(ra - C).cross(u), -rb.cross(u),
                                            fminf(0.f, lo_C), h, R_LIM_LO);
                r.lo = 0.f; r.hi = INFINITY;
                s.rows[s.row_count++] = r;
            } else lambda_r[R_LIM_LO] = 0.f;

            if (hi_C < SIM_LIM_SLOP) {
                JointSolve::Row r = makeRow(a, b, u,
                                            (ra - C).cross(u), rb.cross(u),
                                            fminf(0.f, hi_C), h, R_LIM_HI);
                r.lo = 0.f; r.hi = INFINITY;
                s.rows[s.row_count++] = r;
            } else lambda_r[R_LIM_HI] = 0.f;
            break;
        }

        case PISTON: {
            s.has_anchor = false;

            sf::Vector2f u = Utils::rotVec({std::cos(axis_angle), std::sin(axis_angle)}, a->getCS());
            sf::Vector2f t = u.perpendicular();

            s.rows[s.row_count++] = makeRow(a, b, t,
                                            (ra - C).cross(t), rb.cross(t),
                                            C.dot(t), h, R_PERP);
            s.rows[s.row_count++] = makeRow(a, b, {0.f, 0.f}, 1.f, 1.f,
                                            Utils::wrapPi(C_ang), h, R_ANG);

            float pos = -C.dot(u);              // B's coordinate along the axis
            float lo_C = pos - course_min;      // < 0 == past the near stop
            float hi_C = course_max - pos;

            if (lo_C < SIM_LIM_SLOP) {
                JointSolve::Row r = makeRow(a, b, -u,
                                            -(ra - C).cross(u), -rb.cross(u),
                                            fminf(0.f, lo_C), h, R_LIM_LO);
                r.lo = 0.f; r.hi = INFINITY;
                s.rows[s.row_count++] = r;
            } else lambda_r[R_LIM_LO] = 0.f;

            if (hi_C < SIM_LIM_SLOP) {
                JointSolve::Row r = makeRow(a, b, u,
                                            (ra - C).cross(u), rb.cross(u),
                                            fminf(0.f, hi_C), h, R_LIM_HI);
                r.lo = 0.f; r.hi = INFINITY;
                s.rows[s.row_count++] = r;
            } else lambda_r[R_LIM_HI] = 0.f;
            JointSolve::Row r = makeRow(a, b, -u,
                                        -(ra - C).cross(u), -rb.cross(u),
                                        pos - target, h, R_AXIS);   // pos = -C.dot(u)
            r.lo = -max_force * h;
            r.hi =  max_force * h;
            s.rows[s.row_count++] = r;
            break;
        }

        case SPRING: {
            s.has_anchor = false;
            float len = C.length();
            if (len > 1e-6f) {
                sf::Vector2f n = C / len;
                s.rows[s.row_count++] = makeRow(a, b, n, ra.cross(n), rb.cross(n),
                                                len - rest_length, h, R_AXIS);
            }
            break;
        }

        case ANG_SPRING:
            s.rows[s.row_count++] = makeRow(a, b, {0.f, 0.f}, 1.f, 1.f,
                                            Utils::wrapPi(C_ang), h, R_ANG);
            break;

        default:
            break;
    }

    return s;
}

JointSolve::Row Joint::makeRow(PixelBody *A, PixelBody *B, sf::Vector2f dir, float ja, float jb, float C, float h, RowSlot slot) {
    JointSolve::Row r{};
    r.dir = dir;
    r.ja  = ja;
    r.jb  = jb;
    r.lambda = &lambda_r[slot];
    r.lo = -INFINITY;
    r.hi =  INFINITY;

    float k = (A->imass() + B->imass()) * dir.dot(dir)
            + A->iinertia() * ja * ja
            + B->iinertia() * jb * jb;

    if (hertz > 0.f) {
        float m_eff = (k > 1e-12f) ? 1.f / k : 0.f;
        float w  = 2.f * M_PIf * hertz;
        float ks = m_eff * w * w;         // stiffness
        float c  = 2.f * m_eff * damping * w;  // damping coefficient
        r.gamma = 1.f / (h * (c + h * ks));
        r.bias  = (ks / (c + h * ks)) * C;
    } else {
        r.gamma = 0.f;
        r.bias  = (beta / h) * C;
    }

    // gamma must already be final: it belongs in the denominator
    float kk = k + r.gamma;
    r.inv_k = (kk > 1e-12f) ? 1.f / kk : 0.f;

    return r;
}