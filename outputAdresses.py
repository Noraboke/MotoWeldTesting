# just an adress collection for welding unit 1 (if there are more they might be mapped elsewhere)

ot = {}
# INPUT GROUPS
ot['Door Locked'] = 21
# these should not be necessary to check, as the logic check should already be happening inside ARCON (ladder)
ot['Heartbeat'] = 50
ot['WelderReady'] = 51
ot['Process active'] = 53
ot['Current flow'] = 54
ot['Arc stable'] = 55
ot['Main current'] = 56
ot['Touch signal'] = 57


# OUTPUT GROUPS
ot['WIRE CUT'] = 10027
ot['Gas on'] = 10060
ot['Wire foward'] = 10061
ot['Wire backward'] = 10062
ot['Touch sensing'] = 10064
ot['Torch blow out'] = 10064

ot['MOVE'] = 15080 #OT4057
ot['ARCON'] = 15081 #OT4058
ot['ARCOF'] = 15082 #OT4059

ot['Avoidance'] = 15120
ot['CLEANING AIR:1'] = 15122
ot['GAS CHECK:1'] = 15123
ot['ARC SHORTAGE:1'] = 15124
ot['RESTART RESET:1'] = 15125
ot['WIRE INCHING:1'] = 15126
ot['WIRE RETRACT:1'] = 15127


# IO ROS FEEDBACK
ot['IO_FEEDBACK_WAITING_MP_INCMOVE'] = 11120 #//output# 889
ot['IO_FEEDBACK_MP_INCMOVE_DONE'] = 11121 #//output# 890
ot['IO_FEEDBACK_MP_INITIALIZATION_DONE'] = 11122 #//output# 891
ot['IO_FEEDBACK_CONNECTSERVERRUNNING'] = 11123 #//output# 892
ot['IO_FEEDBACK_MOTIONSERVERCONNECTED'] = 11124 #//output# 893
ot['IO_FEEDBACK_STATESERVERCONNECTED'] = 11125 #//output# 894
ot['IO_FEEDBACK_FAILURE'] = 11127 #//output# 896
