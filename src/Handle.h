#pragma once
#include <cstdint>

template <class Tag>
  struct Handle {
      uint32_t slot = 0, gen = 0;
      bool operator==(const Handle &o) const { return slot == o.slot && gen == o.gen; }
  };
  
  using BodyId  = Handle<struct BodyTag>;
  using JointId = Handle<struct JointTag>;