"""Static constants — DSDL bit values, magic numbers, label maps."""

# GPS_DRV_OPTIONS bit (mirrors AP_GPS.h DriverOptions). Only UBX_DebugMessages
# is exposed in the UI — it gates the u-blox driver's UBX-INF debug forwarding.
GPS_DRV_BIT_UBX_DEBUG = 1 << 10
GPS_PARAM_NAME = "GPS_DRV_OPTIONS"

# uavcan.protocol.RestartNode magic (uavcan/protocol/5.RestartNode.uavcan).
RESTART_MAGIC = 0xACCE551B1E

# uavcan.tunnel.Protocol enum (see uavcan/tunnel/Protocol.uavcan).
TUNNEL_PROTOCOL_GPS_GENERIC = 2
TUNNEL_MAX_CHUNK = 120        # buffer<=120 on uavcan.tunnel.Targetted
TUNNEL_KEEPALIVE_S = 0.5      # send empty Targetted at least this often

# uavcan.equipment.gnss.Fix2 enum string maps.
FIX2_MODE = {0: "SINGLE", 1: "DGPS", 2: "RTK"}
FIX2_SUBMODE = {0: "DGPS_OTHER", 1: "DGPS_SBAS", 2: "RTK_FLOAT", 3: "RTK_FIXED"}
FIX2_STATUS = {0: "NO_FIX", 1: "TIME_ONLY", 2: "2D_FIX", 3: "3D_FIX"}
