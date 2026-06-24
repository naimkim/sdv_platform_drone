# Published by the localization node to report the active estimation mode.
# GPS_DENIED is entered when the GPS integrity monitor rejects GPS (loss or
# spoofing), after which the filter runs on VIO only.

string MODE_GPS_FUSED=GPS_FUSED
string MODE_GPS_DENIED=GPS_DENIED

string mode             # GPS_FUSED | GPS_DENIED
string reason           # human-readable explanation
float64 gps_innovation  # last GPS innovation magnitude [m] (gate input)
bool gps_trusted        # whether GPS is currently fused
int64 stamp_ns
