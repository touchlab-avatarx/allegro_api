// Copyright (c) 2025 Touchlab Limited. All Rights Reserved
// Unauthorized copying or modifications of this file, via any medium is strictly prohibited.

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <algorithm>
#include <chrono>
#include <iostream>
#include <sstream>
#include <exception>

#define THROW(m)                                                                                   \
  {                                                                                                \
    std::stringstream ss;                                                                          \
    ss << m;                                                                                       \
    throw std::runtime_error(ss.str());                                                            \
  }

#define INFO(x) std::cout << x << "\n";
#define WARNING(x) std::cout << "\033[33m" << x << "\033[0m\n";
#define WARNING_THROTTLE(t, x)                                                                     \
  {                                                                                                \
    static auto t0 = std::chrono::high_resolution_clock::now();                                    \
    if (std::chrono::high_resolution_clock::now() - t0 > t) {                                      \
                                                                                                   \
      t0 = std::chrono::high_resolution_clock::now();                                              \
      WARNING(x);                                                                                  \
    }                                                                                              \
  }

namespace allegro
{

inline double clamp(double value, double lower, double upper)
{
  return std::min(upper, std::max(lower, value));
}

}  // namespace allegro

#endif  // UTILS_HPP_
