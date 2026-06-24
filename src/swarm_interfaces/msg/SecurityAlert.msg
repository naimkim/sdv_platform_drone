# Raised by an IDS / monitor node when a swarm message looks malicious.

uint8 SEVERITY_INFO=0
uint8 SEVERITY_WARN=1
uint8 SEVERITY_ERROR=2

string source          # node that raised the alert (e.g. swarm_ids)
string suspect_id      # offending drone_id
string alert_type      # MAC_INVALID | REPLAY | RANGE | RATE | TELEPORT
uint8 severity
string description
int64 stamp_ns
