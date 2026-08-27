#include "Settings.h"
#include "Joint.h"
#include "Utils.h"

Joint::Joint(const BodyId &a, sf::Vector2u pa, const BodyId &b, sf::Vector2u pb, JointType type, float restAngle) {
    this->a = a;
    this->b = b;
    this->pa = pa;
    this->pb = pb;
    this->type = type;
    this->rest_angle = restAngle;
}

JointSolve Joint::prepare(PixelBody *a, PixelBody *b, float h) {
    sf::Vector2f ra = Utils::rotVec(a->offsets[pa.x][pa.y], a->getCS());
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

    if (type == BEARING) {
        return {
            this,
            a, b,
            ra, rb,
            true, {}, 0,
            k00, k01, k11, inv_det,
            bias
        };
    }

    // nothing else supported yet
    return {
        this,
        a, b,
        ra, rb,
        true, {}, 0,
        k00, k01, k11, inv_det,
        bias
    };
}
