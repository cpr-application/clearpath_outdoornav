# Observed physical state, independent of dock/undock action status.
std_msgs/Header header

uint8 UNKNOWN=0
uint8 UNDOCKED=1
uint8 DOCKED=2
uint8 state

uint8 CHARGE_UNKNOWN=0
uint8 NOT_CHARGING=1
uint8 CHARGING=2
uint8 FULL=3
uint8 charging_state

uint8 EVIDENCE_NONE=0
uint8 EVIDENCE_GEOMETRY=1
uint8 EVIDENCE_HARDWARE=2
uint8 EVIDENCE_BOTH=3
uint8 evidence

# Template being observed, not the identity of a surveyed dock.
string dock_template
bool geometry_fresh
bool hardware_fresh
bool conflict
bool charger_fault

# Valid only when geometry_fresh is true; expressed in the desired dock frame.
geometry_msgs/Pose2D pose_error
string reason
