"""Unit tests for the GPS integrity (spoofing) monitor."""

from drone_localization.gps_monitor import GPSIntegrityMonitor


def test_starts_trusted():
    mon = GPSIntegrityMonitor()
    assert mon.trusted


def test_sustained_divergence_trips_to_denied():
    mon = GPSIntegrityMonitor(gate=2.0, trip_count=3)
    assert mon.update(5.0)[0] is True    # 1st over-gate
    assert mon.update(5.0)[0] is True    # 2nd
    trusted, reason = mon.update(5.0)    # 3rd -> trip
    assert trusted is False
    assert 'spoof' in reason or 'rejected' in reason


def test_single_spike_does_not_trip():
    mon = GPSIntegrityMonitor(gate=2.0, trip_count=3)
    mon.update(5.0)        # one bad fix
    mon.update(0.5)        # back within gate resets the streak
    mon.update(5.0)
    mon.update(0.5)
    assert mon.trusted is True


def test_gps_unavailable_denies():
    mon = GPSIntegrityMonitor()
    trusted, reason = mon.update(0.0, gps_available=False)
    assert trusted is False
    assert 'unavailable' in reason


def test_recovery_after_sustained_good_fixes():
    mon = GPSIntegrityMonitor(
        gate=2.0, trip_count=2, recover_gate=1.0, recover_count=3)
    mon.update(5.0)
    mon.update(5.0)
    assert mon.trusted is False
    # Sustained good fixes within the recover gate re-trust GPS.
    mon.update(0.2)
    mon.update(0.2)
    trusted, reason = mon.update(0.2)
    assert trusted is True
    assert 'recovered' in reason


def test_no_premature_recovery():
    mon = GPSIntegrityMonitor(
        gate=2.0, trip_count=2, recover_gate=1.0, recover_count=5)
    mon.update(5.0)
    mon.update(5.0)
    mon.update(0.2)        # only one good fix
    assert mon.trusted is False
