cmake_minimum_required(VERSION 3.8)
################################################################################

option(ABD_WITH_BOX2D_COMPARISONS  "Build Box2D comparison"   ON)
option(ABD_WITH_STIV_COMPARISONS   "Build Box2D comparison"  OFF)

if(ABD_WITH_BOX2D_COMPARISONS)
  add_subdirectory(Box2D)
endif()

if(ABD_WITH_STIV_COMPARISONS)
  add_subdirectory(STIV)
endif()
