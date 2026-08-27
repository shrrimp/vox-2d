#pragma once
#include <cstdint>
#include <unordered_map>

struct ContactKey {
    uint64_t a;   // A: slot(24) | gen(8) | px(16) | py(16)
    uint32_t b;   // B: slot(24) | gen(8)
};

struct ContactState {
    float ln = 0.f;
    float lt = 0.f;
    uint32_t stamp = 0;
};

inline bool operator==(const ContactKey &l, const ContactKey &r) {
    return l.a == r.a && l.b == r.b;
}

struct ContactKeyHash {
    size_t operator()(const ContactKey &k) const noexcept {
        uint64_t h = k.a * 0x9E3779B97F4A7C15ull;
        h ^= uint64_t(k.b) + 0x9E3779B97F4A7C15ull + (h << 6) + (h >> 2);
        return size_t(h);
    }
};

struct ContactCache {
    std::unordered_map<ContactKey, ContactState, ContactKeyHash> map;
    uint32_t stamp = 0;

    ContactState find(const ContactKey &k) const {
        auto it = map.find(k);
        return it == map.end() ? ContactState{} : it->second;
    }

    void store(const ContactKey &k, float ln, float lt) {
        map[k] = ContactState{ln, lt, stamp};
    }

    void sweep(uint32_t max_age) {
        for (auto it = map.begin(); it != map.end(); ) {
            if (stamp - it->second.stamp > max_age) it = map.erase(it);
            else ++it;
        }
    }
};