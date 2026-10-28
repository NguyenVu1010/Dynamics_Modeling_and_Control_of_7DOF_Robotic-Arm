#pragma once

// Core templates are declared in headers and defined in src/*.cpp. Each .cpp instantiates
// them explicitly for the DOF values below, so headers stay free of algorithm bodies and
// compile times stay low. 1 and 2 serve analytic test models (pendulum, planar 2-link arm);
// 7 is the arm. A new robot adds its DOF here.
//
//   #define INSTANTIATE(N) template class ClikReference<N>;
//   COBOT_INSTANTIATE_DOF(INSTANTIATE)
//   #undef INSTANTIATE
#define COBOT_INSTANTIATE_DOF(MACRO) \
  MACRO(1)                           \
  MACRO(2)                           \
  MACRO(7)
