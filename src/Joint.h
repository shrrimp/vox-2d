#pragma once
#include "Handle.h"
#include "PixelBoxy.h"
#include <SFML/System/Vector2.hpp>
#include <memory>
#include <cassert>
#include <type_traits>
#include <algorithm>
#include <vector>

struct JointSolve;

// Joint data
struct Joint {
    
    enum JointType {
        FIXED, // cannot move or rotate
        BEARING, // can only rotate
        MOTOR, // can rotate but controlled by a signal
        LIN_BEARING, // can move along a defined axis
        PISTON, // can move along an axis controlled by a signal
        SPRING, // links two points, tries to put them at it's resting distance and flexion based on it's strength
        ANG_SPRING // bearing but tries to go to it's resting angle based on it's strength
    };

    enum RowSlot { R_ANG = 0, R_PERP, R_AXIS, ROW_COUNT };
    float lambda_r[ROW_COUNT]{};
    
    JointId self;
    
    JointType type = FIXED;
    
    BodyId a, b;
    sf::Vector2u pa, pb;    // anchor pixel in each body
    
    sf::Vector2f lambda{};  // accumulated linear impulse
    float rest_angle = 0.f;
    
    float beta  = 0.2f;     // positional stiffness
    float break_impulse = INFINITY;
    bool  collide = false;  // if false, register a BodyStorage::filter
    
    Joint(const BodyId &a, sf::Vector2u pa, const BodyId &b, sf::Vector2u pb, JointType type, float restAngle);
    
    JointSolve prepare(PixelBody *a, PixelBody *b, float h);
};

// Joint solving data, regenerated each substep
struct JointSolve {
    struct Row {
        sf::Vector2f dir;   // linear direction; {0,0} for a pure angular row
        float ja, jb;       // angular Jacobian entries
        float inv_k;        // 1 / (JM⁻¹Jᵀ + gamma)
        float bias;
        float gamma;        // 0 = rigid
        float *lambda;      // points into the Joint — persistent across steps
        float lo, hi;       // clamp on the ACCUMULATED impulse
    };

    Joint *j;
    PixelBody *A, *B;
    sf::Vector2f ra, rb;

    bool has_anchor;
    Row rows[Joint::ROW_COUNT];
    int row_count;
    
    float k00, k01, k11, inv_det; // 2x2 inverse, prebuilt
    sf::Vector2f bias;
};


struct JointStorage {
    std::vector<std::unique_ptr<Joint>> joints;
    std::vector<uint32_t> gens; // gens
    std::vector<uint32_t> free; // free slots
    std::vector<uint32_t> live; // live slots to iterate

    template <class... Args>
    JointId create(Args&&... args) {
        static_assert(std::is_constructible_v<Joint, Args&&...>,
                        "no Joint constructor matches these arguments"); // check arguments

        auto joint = std::make_unique<Joint>(std::forward<Args>(args)...); // build object

        uint32_t slot; // find slot, if unavailable create one
        if (!free.empty()) { slot = free.back(); free.pop_back(); }
        else {
            slot = static_cast<uint32_t>(joints.size());
            joints.emplace_back();
            gens.push_back(1); // brand new slot -> gen = 1
        }

        JointId id{slot, gens[slot]};
        joint->self = id;
        joints[slot] = std::move(joint); // move object ownership to the slot
        live.push_back(slot); // add the slot to the live array
        return id;
    }

    void destroy(const JointId &id) {
        Joint *joint = get(id);
        if (joint == nullptr) return; // silent, no need to signal that something that we wanted to destroy already is

        // delete joint
        joints[id.slot].reset();

        // add slot to free slot if not present
        free.push_back(id.slot);
        auto lv = std::find(live.begin(), live.end(), id.slot); // when destroys become frequent ; add slot -> live table to make it O(1)
        assert(lv != live.end()); // just in case
        *lv = live.back();
        live.pop_back(); // O(1) instead of the O(n) of erase

        (gens[id.slot])++; // increment generation of slot
    }

    const std::vector<uint32_t>& liveSlots() const { return live; }

    Joint& operator[](uint32_t slot) { // unchecked for performance when iterating through live array
        return *(joints[slot]);
    }
    
    Joint *get(const JointId &id) {
        if (id.slot >= gens.size()) return nullptr;
        if (id.gen != gens[id.slot]) return nullptr;
        return joints[id.slot].get();
    }
    
    const Joint& operator[](uint32_t slot) const { // unchecked for performance when iterating through live array
        return *(joints[slot]);
    }
    
    const Joint *get(const JointId &id) const {
        if (id.slot >= gens.size()) return nullptr;
        if (id.gen != gens[id.slot]) return nullptr;
        return joints[id.slot].get();
    }
    
    JointId jointId(uint32_t slot) const { // unchecked, this goes with live traversal, guaranteed to be valid for normal use
        return {slot, gens[slot]};
    }
};