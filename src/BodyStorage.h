#pragma once
#include <memory>
#include <vector>
#include <algorithm>
#include <cassert>
#include <type_traits>
#include "Handle.h"
#include "PixelBoxy.h"
#include "Joint.h"

struct BodyStorage {
    std::vector<std::unique_ptr<PixelBody>> bodies;
    std::vector<uint32_t> gens; // gens
    std::vector<uint32_t> free; // free slots
    std::vector<uint32_t> live; // live slots to iterate

    template <class... Args>
    BodyId create(Args&&... args) {
        static_assert(std::is_constructible_v<PixelBody, Args&&...>,
                        "no PixelBody constructor matches these arguments"); // check arguments

        auto body = std::make_unique<PixelBody>(std::forward<Args>(args)...); // build object

        uint32_t slot; // find slot, if unavailable create one
        if (!free.empty()) { slot = free.back(); free.pop_back(); }
        else {
            slot = static_cast<uint32_t>(bodies.size());
            bodies.emplace_back();
            gens.push_back(1); // brand new slot -> gen = 1
        }

        BodyId id{slot, gens[slot]};
        body->self = id;
        bodies[slot] = std::move(body); // move object ownership to the slot
        live.push_back(slot); // add the slot to the live array
        return id;
    }

    void destroy(const BodyId &id) {
        PixelBody *body = get(id);
        if (body == nullptr) return; // silent, no need to signal that something that we wanted to destroy already is

        // delete filter references
        for (const BodyId &other_id : body->f_bodies) {
            PixelBody *other = get(other_id);
            if (other == nullptr) continue;
            auto &f_bodies = other->f_bodies;
            for (size_t i = 0; i < f_bodies.size(); i++) {
                if (f_bodies[i] == id) {
                    f_bodies[i] = f_bodies.back();
                    f_bodies.pop_back();
                    break;
                }
            }
        }

        // delete body
        bodies[id.slot].reset();

        // add slot to free slot if not present
        free.push_back(id.slot);
        auto lv = std::find(live.begin(), live.end(), id.slot); // when destroys become frequent ; add slot -> live table to make it O(1)
        assert(lv != live.end()); // just in case
        *lv = live.back();
        live.pop_back(); // O(1) instead of the O(n) of erase

        (gens[id.slot])++; // increment generation of slot
    }

    const std::vector<uint32_t>& liveSlots() const { return live; }

    PixelBody& operator[](uint32_t slot) { // unchecked for performance when iterating through live array
        return *(bodies[slot]);
    }
    
    PixelBody *get(const BodyId &id) {
        if (id.slot >= gens.size()) return nullptr;
        if (id.gen != gens[id.slot]) return nullptr;
        return bodies[id.slot].get();
    }
    
    const PixelBody& operator[](uint32_t slot) const { // unchecked for performance when iterating through live array
        return *(bodies[slot]);
    }
    
    const PixelBody *get(const BodyId &id) const {
        if (id.slot >= gens.size()) return nullptr;
        if (id.gen != gens[id.slot]) return nullptr;
        return bodies[id.slot].get();
    }
    
    BodyId bodyId(uint32_t slot) const { // unchecked, this goes with live traversal, guaranteed to be valid for normal use
        return {slot, gens[slot]};
    }

    void filter(const BodyId &a, const BodyId &b, bool filter) {
        if (a == b) return;
        PixelBody *body_a = get(a);
        PixelBody *body_b = get(b);

        if (body_a == nullptr || body_b == nullptr) return;
        size_t asize = body_a->f_bodies.size();
        size_t bsize = body_b->f_bodies.size();

        // Probably overkill, just in case, this is an operation not done very often it might as well be robust
        bool founda = false;
        bool foundb = false;
        for (size_t i = 0; i < std::max(asize, bsize); i++) {
            if (i < asize && body_a->f_bodies[i] == b) {
                if (filter) {
                    foundb = true;
                } else {
                    body_a->f_bodies[i] = body_a->f_bodies.back();
                    body_a->f_bodies.pop_back();
                    asize = i;
                }
            }
            if (i < bsize && body_b->f_bodies[i] == a) {
                if (filter) {
                    founda = true;
                } else {
                    body_b->f_bodies[i] = body_b->f_bodies.back();
                    body_b->f_bodies.pop_back();
                    bsize = i;
                }
            }
        }

        if (filter && !founda) body_a->f_bodies.push_back(b);
        if (filter && !foundb) body_b->f_bodies.push_back(a);
    }
};