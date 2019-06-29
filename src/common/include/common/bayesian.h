#pragma once

#include <algorithm>

template <typename Container>
void normalize(Container &c) {
    const auto sum = std::accumulate(c.begin(), c.end());
    std::for_each(c.begin(), c.end(), [sum](auto &each) { return each / sum; });
}
