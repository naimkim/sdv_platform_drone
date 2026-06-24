"""SecOC-style message authentication primitives for the swarm bus.

This mirrors AUTOSAR SecOC at the application layer: every authenticated PDU
carries a freshness value (a monotonic counter, anti-replay) and a truncated
MAC computed over (sender id || freshness || payload). The group shares a
symmetric key, exactly like a SecOC key slot shared across an ECU group.

The same code that signs a CAN frame in a vehicle is reused, conceptually, to
sign a drone pose on a DDS topic.
"""

import hashlib
import hmac
import struct

# 64-bit truncated MAC, as is common for SecOC authenticators on constrained
# links. Long enough to make forgery infeasible, short enough to stay cheap.
MAC_TRUNCATION_BYTES = 8

# Pose payload layout: 6 big-endian doubles (x, y, z, vx, vy, vz).
_POSE_STRUCT = struct.Struct('>6d')


def pose_payload(x, y, z, vx, vy, vz):
    """Serialize the pose fields into the canonical signed payload."""
    return _POSE_STRUCT.pack(x, y, z, vx, vy, vz)


def heartbeat_payload(stamp_ns):
    """Serialize the heartbeat payload (just the timestamp)."""
    return struct.pack('>q', int(stamp_ns))


def compute_mac(secret_key, drone_id, freshness, payload):
    """Return the truncated HMAC-SHA256 authenticator as a hex string.

    secret_key : bytes   shared group key
    drone_id   : str     claimed sender identity (bound into the MAC)
    freshness  : int     monotonic freshness value (anti-replay)
    payload    : bytes   message-specific signed bytes
    """
    message = drone_id.encode('utf-8') + struct.pack('>Q', int(freshness)) + payload
    full = hmac.new(secret_key, message, hashlib.sha256).digest()
    return full[:MAC_TRUNCATION_BYTES].hex()


def verify_mac(secret_key, drone_id, freshness, payload, mac_hex):
    """Constant-time verification of a received authenticator."""
    expected = compute_mac(secret_key, drone_id, freshness, payload)
    try:
        return hmac.compare_digest(expected, mac_hex)
    except (TypeError, ValueError):
        return False


def derive_key(secret_text):
    """Derive a fixed-length group key from a human-readable secret.

    Lets launch files pass a readable string while the MAC uses full entropy.
    """
    return hashlib.sha256(secret_text.encode('utf-8')).digest()


class FreshnessVerifier:
    """Per-sender monotonic freshness tracking for replay detection.

    Accepts a message only if its freshness value strictly exceeds the highest
    value seen so far from that sender. This is the SecOC freshness-value
    verification, minus the synchronized-counter handshake.
    """

    def __init__(self):
        self._last = {}

    def check(self, drone_id, freshness):
        """Return True if freshness is fresh; record it. False on replay."""
        last = self._last.get(drone_id)
        if last is not None and freshness <= last:
            return False
        self._last[drone_id] = freshness
        return True

    def peek(self, drone_id):
        return self._last.get(drone_id)
