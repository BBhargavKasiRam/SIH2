# ReplanRequest.msg
# Instructs a specific robot (via its adapter) to abandon its current path and replan

string robot_id
string fleet_name
string reason
float32 expected_delay_sec
