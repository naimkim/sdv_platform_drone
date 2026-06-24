# Global trust view published by the consensus guard.
# Agents use the trusted set to deterministically derive their search sector.

string[] trusted_drones        # members in good standing (sorted)
string[] quarantined_drones    # members excluded from consensus / allocation
int64 stamp_ns
