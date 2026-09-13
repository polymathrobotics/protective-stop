# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_PSTOP)

set(PSTOP_ROOT ${ZEPHYR_CURRENT_MODULE_DIR})
set(PSTOP_SRC ${PSTOP_ROOT}/pstop_c/pstop/src/pstop)

zephyr_library()

zephyr_include_directories(
  ${PSTOP_ROOT}/pstop_c/pstop/include
  ${PSTOP_ROOT}/common
)

# Core: everything both roles need — wire format, ids, checksum, per-peer
# counters.
zephyr_library_sources(
  ${PSTOP_SRC}/checksum.c
  ${PSTOP_SRC}/device_id.c
  ${PSTOP_SRC}/endian.c
  ${PSTOP_SRC}/os.c
  ${PSTOP_SRC}/protocol_data.c
  ${PSTOP_SRC}/pstop_msg.c
)

# Machine side only. A remote never calls into these.
zephyr_library_sources_ifdef(CONFIG_PSTOP_MACHINE
  ${PSTOP_SRC}/machine.c
  ${PSTOP_SRC}/protocol.c
  ${PSTOP_SRC}/pstop_application.c
  ${PSTOP_SRC}/pstop_remote_data.c
)

# protocol.c's debug logging prints uint64 stamps with %ld, which is a hard
# -Werror=format wherever long is 32-bit. Relax the diagnostic for this
# library only; a consuming application keeps its own warning set.
if(CONFIG_PSTOP_MACHINE)
  target_compile_options(${ZEPHYR_CURRENT_LIBRARY} PRIVATE -Wno-format)
endif()

# time_get_now(): Zephyr's kernel clock, or upstream's (Linux-only) original.
zephyr_library_sources_ifdef(CONFIG_PSTOP_ZEPHYR_TIME pstop_time_zephyr.c)
zephyr_library_sources_ifndef(CONFIG_PSTOP_ZEPHYR_TIME ${PSTOP_SRC}/time.c)

if(CONFIG_PSTOP_TRANSPORT_UDP)
  zephyr_include_directories(${PSTOP_ROOT}/pstop_c/transport/include)
  zephyr_library_sources(
    ${PSTOP_ROOT}/pstop_c/transport/src/transport/udp/udp_transport.c)
endif()

endif() # CONFIG_PSTOP
