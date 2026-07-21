# ESP-IDF component wrapper for polymathrobotics/protective-stop's pstop_c.
#
# The certified pstop_c library lives at the repo-root `pstop_c/` folder
# (kept exactly as-is; formatting/linting excluded per .pre-commit-config).
# This component just compiles it for ESP-IDF via the path below.
#
# Layout:
#   ../../pstop_c/pstop/             core C library (pure C, OS-agnostic)
#   ../../pstop_c/transport/         UDP transport (POSIX sockets — works on lwIP)
#   port/                               ESP-IDF-specific overrides
#       pstop_time_esp.c                replaces upstream's time.c (esp_timer-backed)
#
# Headers exposed to consumers of this component:
#   pstop/*.h           (from ../../pstop_c/pstop/include/pstop/)
#   transport/*.h       (from ../../pstop_c/transport/include/transport/)

set(PSTOP_UPSTREAM "${CMAKE_CURRENT_LIST_DIR}/../../pstop_c")

# Core library sources. Pull the full pstop/src/pstop/*.c set EXCEPT time.c
# (we substitute with our esp_timer-backed port).
set(PSTOP_CORE_SRCS
    "${PSTOP_UPSTREAM}/pstop/src/pstop/checksum.c"
    "${PSTOP_UPSTREAM}/pstop/src/pstop/device_id.c"
    "${PSTOP_UPSTREAM}/pstop/src/pstop/endian.c"
    "${PSTOP_UPSTREAM}/pstop/src/pstop/machine.c"
    "${PSTOP_UPSTREAM}/pstop/src/pstop/os.c"
    "${PSTOP_UPSTREAM}/pstop/src/pstop/protocol.c"
    "${PSTOP_UPSTREAM}/pstop/src/pstop/protocol_data.c"
    "${PSTOP_UPSTREAM}/pstop/src/pstop/pstop_application.c"
    # upstream #42 renamed client -> remote (pstop_client_data.c -> pstop_remote_data.c)
    "${PSTOP_UPSTREAM}/pstop/src/pstop/pstop_remote_data.c"
    "${PSTOP_UPSTREAM}/pstop/src/pstop/pstop_msg.c"
    # NOTE: upstream's time.c intentionally NOT in this list — see port/.
    "port/pstop_time_esp.c"
)

# UDP transport sources. POSIX sockets via lwIP. Optional; consumers that
# bring their own transport can REQUIRE only the core headers/sources.
set(PSTOP_TRANSPORT_SRCS
    "${PSTOP_UPSTREAM}/transport/src/transport/udp/udp_transport.c"
)

idf_component_register(
    SRCS
        ${PSTOP_CORE_SRCS}
        ${PSTOP_TRANSPORT_SRCS}
    INCLUDE_DIRS
        "${PSTOP_UPSTREAM}/pstop/include"
        "${PSTOP_UPSTREAM}/transport/include"
        # Put port/ FIRST so port/poll.h shadows any system <poll.h>
        # lookup with our shim (ESP-IDF newlib only ships <sys/poll.h>,
        # not <poll.h> bare).
        "port"
    REQUIRES
        # lwip — for the BSD socket layer that transport_udp.c uses
        lwip
        # esp_timer — for our port/pstop_time_esp.c
        esp_timer
)
