set(INFRARED_SRCS
  alinfrared.h
  alinfrared.cpp
  infraredmain.cpp
)

configure_src_module(infrared ${INFRARED_SRCS})
use_lib(infrared ALCOMMON ALMATH LIRCCLIENT ALLOG)

