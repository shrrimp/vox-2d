#pragma once
#include <cstdint>

struct BodyId {
    uint32_t slot = 0, gen = 0; 

    bool operator==(const BodyId &other) const {
        return slot == other.slot && gen == other.gen;
    }
};