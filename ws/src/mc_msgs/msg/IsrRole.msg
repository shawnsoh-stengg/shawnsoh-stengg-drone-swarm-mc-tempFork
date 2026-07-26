# GAP NOTE: ICD-MC-001 §9's topic table has cited mc_msgs/IsrRole since it
# was added, but no message body existed anywhere in the ICD (v0.1-DRAFT,
# 2026-06-20) until this file. Authored locally 2026-07-26 to close that
# hole — not sourced from an upstream ICD revision; needs ICD-owner
# review/ratification.
#
# This drone's own ISR-election outcome — published by swarm_coordinator_node
# on /mc/swarm/isr_role on change (SDD §6.2). Consumed by mission_manager_node
# to gate the ISR_SELECTION -> VORONOI_PARTITION transition, whose guard is
# "ISR result ACKed by all peers" (SDD §4 transition table).
std_msgs/Header header

uint8 PENDING = 0   # election not yet resolved (still in ISR_CANDIDATE collection window)
uint8 ISR     = 1   # this drone was elected to the ISR subset
uint8 NOT_ISR = 2   # this drone was not elected
uint8 role                # PENDING, ISR, or NOT_ISR — see constants above

float32 score             # this drone's computed election score (§6.2 scoring function)
uint8   coordinator_id    # drone_id running the election (lowest-ID ALIVE peer, §6.2)
bool    result_acked      # true once this drone's ISR_RESULT_ACK round-trip completed
bool    all_peers_acked   # true once every ALIVE peer has ACKed — gates the phase transition
uint32  election_round    # increments each re-run (e.g. coordinator failover, re-broadcast)
