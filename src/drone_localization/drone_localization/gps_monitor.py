"""GPS integrity monitor — detects GPS loss and spoofing.

Compares each GPS fix against the filter's own prediction (the innovation). A
genuine fix stays close to the prediction; a spoofed or drifting fix diverges.
When the innovation exceeds a gate for several consecutive fixes, GPS is
rejected and the system falls back to VIO-only (GPS-denied) navigation.

This is the same idea as a CAN IDS plausibility gate applied to a navigation
sensor: trust the source only while its reports remain physically consistent.
Hysteresis (trip / recover streaks) prevents a single noisy fix from flipping
the mode.
"""


class GPSIntegrityMonitor:
    def __init__(self, gate=2.0, trip_count=3,
                 recover_gate=1.0, recover_count=10):
        # Innovation above `gate` for `trip_count` fixes -> reject GPS.
        self.gate = float(gate)
        self.trip_count = int(trip_count)
        # Innovation below `recover_gate` for `recover_count` fixes -> re-trust.
        self.recover_gate = float(recover_gate)
        self.recover_count = int(recover_count)

        self.trusted = True
        self.over_streak = 0
        self.under_streak = 0
        self.last_innovation = 0.0
        self.reason = 'init'

    def update(self, innovation_distance, gps_available=True):
        """Return (trusted, reason) after folding in one GPS observation."""
        self.last_innovation = float(innovation_distance)

        if not gps_available:
            self.over_streak = 0
            self.under_streak = 0
            if self.trusted:
                self.trusted = False
                self.reason = 'GPS unavailable'
            return self.trusted, self.reason

        if innovation_distance > self.gate:
            self.over_streak += 1
            self.under_streak = 0
            if self.trusted and self.over_streak >= self.trip_count:
                self.trusted = False
                self.reason = (
                    f'GPS rejected: innovation {innovation_distance:.2f} m '
                    f'> gate {self.gate:.2f} m for {self.over_streak} fixes '
                    f'(spoof/divergence)')
        else:
            self.under_streak += 1
            self.over_streak = 0
            if not self.trusted and innovation_distance <= self.recover_gate \
                    and self.under_streak >= self.recover_count:
                self.trusted = True
                self.reason = 'GPS recovered: innovation back within gate'

        return self.trusted, self.reason
