# A swarm member's vote on whether a peer is malicious.
# Used by the Byzantine-resilient quarantine consensus.

string voter_id        # who is voting
string suspect_id      # who is accused
bool malicious         # true = accuse, false = vouch / clear
int64 stamp_ns
