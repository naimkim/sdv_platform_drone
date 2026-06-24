# Authenticated drone state broadcast on the swarm bus.
# Mirrors an AUTOSAR SecOC PDU: payload + freshness value + truncated MAC.

string drone_id        # claimed sender identity

float64 x              # position [m]
float64 y
float64 z
float64 vx             # velocity [m/s]
float64 vy
float64 vz

uint64 freshness       # monotonic counter (SecOC freshness value, anti-replay)
int64 stamp_ns         # publisher clock [ns]

string mac             # truncated HMAC-SHA256 over (id || freshness || payload), hex
