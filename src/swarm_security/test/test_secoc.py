"""Unit tests for the SecOC-style authentication primitives."""

from swarm_security import secoc


KEY = secoc.derive_key('group-key')


def test_mac_roundtrip_verifies():
    payload = secoc.pose_payload(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)
    mac = secoc.compute_mac(KEY, 'drone_1', 42, payload)
    assert secoc.verify_mac(KEY, 'drone_1', 42, payload, mac)


def test_wrong_key_rejected():
    payload = secoc.pose_payload(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)
    mac = secoc.compute_mac(KEY, 'drone_1', 42, payload)
    forged = secoc.derive_key('group-key:forged')
    assert not secoc.verify_mac(forged, 'drone_1', 42, payload, mac)


def test_identity_is_bound_into_mac():
    payload = secoc.pose_payload(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)
    mac = secoc.compute_mac(KEY, 'drone_1', 42, payload)
    # Same payload, different claimed sender -> MAC must not verify.
    assert not secoc.verify_mac(KEY, 'drone_2', 42, payload, mac)


def test_freshness_change_breaks_mac():
    payload = secoc.pose_payload(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)
    mac = secoc.compute_mac(KEY, 'drone_1', 42, payload)
    assert not secoc.verify_mac(KEY, 'drone_1', 43, payload, mac)


def test_malformed_mac_is_safe():
    payload = secoc.pose_payload(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)
    assert not secoc.verify_mac(KEY, 'drone_1', 42, payload, None)
    assert not secoc.verify_mac(KEY, 'drone_1', 42, payload, 'zz')


def test_freshness_verifier_accepts_monotonic():
    fv = secoc.FreshnessVerifier()
    assert fv.check('drone_1', 1)
    assert fv.check('drone_1', 2)
    assert fv.check('drone_1', 100)


def test_freshness_verifier_rejects_replay():
    fv = secoc.FreshnessVerifier()
    assert fv.check('drone_1', 5)
    assert not fv.check('drone_1', 5)   # exact replay
    assert not fv.check('drone_1', 4)   # older
    assert fv.check('drone_1', 6)       # advances again


def test_freshness_is_per_sender():
    fv = secoc.FreshnessVerifier()
    assert fv.check('drone_1', 10)
    assert fv.check('drone_2', 1)       # independent counter
