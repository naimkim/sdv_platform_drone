# Authenticated liveness beacon for a swarm member.

string drone_id
uint64 freshness       # monotonic counter (SecOC freshness value)
int64 stamp_ns
string mac             # truncated HMAC over (id || freshness || stamp), hex
