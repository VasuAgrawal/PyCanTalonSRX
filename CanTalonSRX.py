import Queue
import can
import time
import binascii
from ctypes import *

CONTROL_1               = 0x02040000
CONTROL_2               = 0x02040040
CONTROL_3               = 0x02040080

STATUS_1                = 0x02041400
STATUS_2                = 0x02041440
STATUS_3                = 0x02041480
STATUS_4                = 0x020414C0
STATUS_5                = 0x02041500
STATUS_6                = 0x02041540
STATUS_7                = 0x02041580

PARAM_REQUEST           = 0x02041800
PARAM_RESPONSE          = 0x02041840
PARAM_SET               = 0x02041880

FLOAT_TO_FXP            = 0x400000
FXP_TO_FLOAT            = 0.0000002384185791015625

class TxQueue(Queue.Queue, object):
    def __init__(self, maxsize = 0):
        super(TxQueue, self).__init__(maxsize)
        self.freeze = False

    def put(self, item, block = True, timeout = None):
        if not self.freeze:
            super(TxQueue, self).put(item, block, timeout)

    def put_nowait(self, item):
        self.put(item, block = False)

    def set_freeze(self, value = True):
        self.freeze = value

class Mode:
    DutyCycle = 0
    PositionCloseLoop = 1
    VelocityCloseLoop = 2
    CurrentCloseLoop = 3 # Not implemented
    VoltCompen = 4 # Not implemented
    SlaveFollower = 5
    NoDrive = 6

class LimitSwitchOverride:
    UseDefaultsFromFlash = 1
    DisableFwd_DisableRev = 4
    DisableFwd_EnableRev = 5
    EnableFwd_DisableRev = 6
    EnableFwd_EnableRev = 7

class BrakeOverride:
    UseDefaultsFromFlash = 0
    OverrideCoast = 1
    OverrideBrake = 2

class FeedbackDev:
    DigitalQuadEnc = 0
    AnalogPot = 1
    AnalogEncoder = 2
    CountEveryRisingEdge = 4
    CountEveryFallingEdge = 5
    PosIsPulseWidth = 8

class ProfileSlotSelect:
    Slot0 = 0
    Slot1 = 1

class StatusFrame:
    General = 0
    Feedback = 1
    Encoder = 2
    AnalogTempVbat = 3

class Param:
    eProfileParamSlot0_P = 1
    eProfileParamSlot0_I = 2
    eProfileParamSlot0_D = 3
    eProfileParamSlot0_F = 4
    eProfileParamSlot0_IZone = 5
    eProfileParamSlot0_CloseLoopRampRate = 6
    eProfileParamSlot1_P = 11
    eProfileParamSlot1_I = 12
    eProfileParamSlot1_D = 13
    eProfileParamSlot1_F = 14
    eProfileParamSlot1_IZone = 15
    eProfileParamSlot1_CloseLoopRampRate = 16
    eProfileParamSoftLimitForThreshold = 21
    eProfileParamSoftLimitRevThreshold = 22
    eProfileParamSoftLimitForEnable = 23
    eProfileParamSoftLimitRevEnable = 24
    eOnBoot_BrakeMode = 31
    eOnBoot_LimitSwitch_Forward_NormallyClosed = 32
    eOnBoot_LimitSwitch_Reverse_NormallyClosed = 33
    eOnBoot_LimitSwitch_Forward_Disable = 34
    eOnBoot_LimitSwitch_Reverse_Disable = 35
    eFault_OverTemp = 41
    eFault_UnderVoltage = 42
    eFault_ForLim = 43
    eFault_RevLim = 44
    eFault_HardwareFailure = 45
    eFault_ForSoftLim = 46
    eFault_RevSoftLim = 47
    eStckyFault_OverTemp = 48
    eStckyFault_UnderVoltage = 49
    eStckyFault_ForLim = 50
    eStckyFault_RevLim = 51
    eStckyFault_ForSoftLim = 52
    eStckyFault_RevSoftLim = 53
    eAppliedThrottle = 61
    eCloseLoopErr = 62
    eFeedbackDeviceSelect = 63
    eRevMotDuringCloseLoopEn = 64
    eModeSelect = 65
    eProfileSlotSelect = 66
    eRampThrottle = 67
    eRevFeedbackSensor = 68
    eLimitSwitchEn = 69
    eLimitSwitchClosedFor = 70
    eLimitSwitchClosedRev = 71
    eSensorPosition = 73
    eSensorVelocity = 74
    eCurrent = 75
    eBrakeIsEnabled = 76
    eEncPosition = 77
    eEncVel = 78
    eEncIndexRiseEvents = 79
    eQuadApin = 80
    eQuadBpin = 81
    eQuadIdxpin = 82
    eAnalogInWithOv = 83
    eAnalogInVel = 84
    eTemp = 85
    eBatteryV = 86
    eResetCount = 87
    eResetFlags = 88
    eFirmVers = 89
    eSettingsChanged = 90
    eQuadFilterEn = 91
    ePidIaccum = 93

    testing = 0

    strings = {
        1 : "eProfileParamSlot0_P",
        2 : "eProfileParamSlot0_I",
        3 : "eProfileParamSlot0_D",
        4 : "eProfileParamSlot0_F",
        5 : "eProfileParamSlot0_IZone",
        6 : "eProfileParamSlot0_CloseLoopRampRate",
        11 : "eProfileParamSlot1_P",
        12 : "eProfileParamSlot1_I",
        13 : "eProfileParamSlot1_D",
        14 : "eProfileParamSlot1_F",
        15 : "eProfileParamSlot1_IZone",
        16 : "eProfileParamSlot1_CloseLoopRampRate",
        21 : "eProfileParamSoftLimitForThreshold",
        22 : "eProfileParamSoftLimitRevThreshold",
        23 : "eProfileParamSoftLimitForEnable",
        24 : "eProfileParamSoftLimitRevEnable",
        31 : "eOnBoot_BrakeMode",
        32 : "eOnBoot_LimitSwitch_Forward_NormallyClosed",
        33 : "eOnBoot_LimitSwitch_Reverse_NormallyClosed",
        34 : "eOnBoot_LimitSwitch_Forward_Disable",
        35 : "eOnBoot_LimitSwitch_Reverse_Disable",
        41 : "eFault_OverTemp",
        42 : "eFault_UnderVoltage",
        43 : "eFault_ForLim",
        44 : "eFault_RevLim",
        45 : "eFault_HardwareFailure",
        46 : "eFault_ForSoftLim",
        47 : "eFault_RevSoftLim",
        48 : "eStckyFault_OverTemp",
        49 : "eStckyFault_UnderVoltage",
        50 : "eStckyFault_ForLim",
        51 : "eStckyFault_RevLim",
        52 : "eStckyFault_ForSoftLim",
        53 : "eStckyFault_RevSoftLim",
        61 : "eAppliedThrottle",
        62 : "eCloseLoopErr",
        63 : "eFeedbackDeviceSelect",
        64 : "eRevMotDuringCloseLoopEn",
        65 : "eModeSelect",
        66 : "eProfileSlotSelect",
        67 : "eRampThrottle",
        68 : "eRevFeedbackSensor",
        69 : "eLimitSwitchEn",
        70 : "eLimitSwitchClosedFor",
        71 : "eLimitSwitchClosedRev",
        73 : "eSensorPosition",
        74 : "eSensorVelocity",
        75 : "eCurrent",
        76 : "eBrakeIsEnabled",
        77 : "eEncPosition",
        78 : "eEncVel",
        79 : "eEncIndexRiseEvents",
        80 : "eQuadApin",
        81 : "eQuadBpin",
        82 : "eQuadIdxpin",
        83 : "eAnalogInWithOv",
        84 : "eAnalogInVel",
        85 : "eTemp",
        86 : "eBatteryV",
        87 : "eResetCount",
        88 : "eResetFlags",
        89 : "eFirmVers",
        90 : "eSettingsChanged",
        91 : "eQuadFilterEn",
        93 : "ePidIaccum"
    }

# Control structs
class TALON_Control_1_General_10ms(object):

    #TODO: Initialize with the proper settings (using enum values) instead of 0
    def __init__(self):
        self.TokenH = 0
        self.TokenL = 0
        self.DemandH = 0
        self.DemandM = 0
        self.DemandL = 0
        self.ProfileSlotSelect = 0
        self.FeedbackDeviceSelect = 0
        self.OverrideLimitSwitchEn = 0
        self.RevFeedbackSensor = 0
        self.RevMotDuringCloseLoopEn = 0
        self.OverrideBrakeType = 0
        self.ModeSelect = 0
        self.RampThrottle = 0

        self.Token = 0
        self.Demand = 0

    def set_demand(self, demand):
        # print "received demand:", demand
        self.Demand = int(demand)
        self.DemandH = (self.Demand & 0x00FF0000) >> 16
        self.DemandM = (self.Demand & 0x0000FF00) >> 8
        self.DemandL = self.Demand & 0x000000FF

    def set_token(self, token):
        self.Token = int(token)
        self.TokenH = (self.Token & 0x0000FF00) >> 8
        self.TokenL = self.Token & 0x000000FF

    def pack(self):
        return bytearray([self.TokenH,
                          self.TokenL,
                          self.DemandH,
                          self.DemandM,
                          self.DemandL,
                          (self.OverrideLimitSwitchEn << 5) +
                          (self.FeedbackDeviceSelect << 1) +
                          self.ProfileSlotSelect,
                          (self.ModeSelect << 4) +
                          (self.OverrideBrakeType << 2) +
                          (self.RevMotDuringCloseLoopEn << 1) +
                          (self.RevFeedbackSensor),
                          self.RampThrottle])

# Used to control the rates for values being sent back
class TALON_Control_2_Rates_OneShot(object):
    def __init__(self):
        self.Status1Ms = 0
        self.Status2Ms = 0
        self.Status3Ms = 0
        self.Status4Ms = 0

    def pack(self):
        return bytearray([self.Status1Ms,
                          self.Status2Ms,
                          self.Status3Ms,
                          self.Status4Ms])

# Used to do something
class TALON_Control_3_ClearFlags_OneShot(object):
    def __init__(self):
        self.ZeroFeedbackSensor = 0
        self.ClearStickyFaults = 0

    def pack(self):
        return bytearray([(self.ClearStickyFaults << 1) +
                          self.ZeroFeedbackSensor])

# Status structs
# General status, returned every 10 ms by default
class TALON_Status_1_General_10ms(object):
    def __init__(self, msg):
        CloseLoopErrH = msg.data[0] # Byte 0
        CloseLoopErrM = msg.data[1] # Byte 1
        CloseLoopErrL = msg.data[2] # Byte 2
        # Byte 3
        AppliedThrottle_h3 = msg.data[3] & 0x07
        self.Fault_RevSoftLim = (msg.data[3] & 0x08) >> 3
        self.Fault_ForSoftLim = (msg.data[3] & 0x10) >> 4
        # Toklocked seems useless
        self.LimitSwitchClosedRev = (msg.data[3] & 0x40) >> 6
        self.LimitSwitchClosedFor = (msg.data[4] & 0x80) >> 7
        AppliedThrottle_l8 = msg.data[4] # Byte 4
        # Byte 5
        ModeSelect_h1 = msg.data[5] & 0x01
        self.FeedbackDeviceSelect = (msg.data[5] & 0x01E) >> 1
        self.LimitSwitchEn = (msg.data[5] & 0xE0) >> 5
        # Byte 6
        self.Fault_HardwareFailure = (msg.data[6] & 0x01)
        self.Fault_RevLim = (msg.data[6] & 0x02) >> 1
        self.Fault_ForLim = (msg.data[6] & 0x04) >> 2
        self.Fault_UnderVoltage = (msg.data[6] & 0x80) >> 3
        self.Fault_OverTemp = (msg.data[6] & 0x10) >> 4
        ModeSelect_b3 = (msg.data[6] & 0xE0) >> 5
        # TokenSeed = msg.data[7]
        self.CloseLoopErr = ((CloseLoopErrH << 16) + (CloseLoopErrM << 8)
                + CloseLoopErrL)
        self.CloseLoopErr = sign_extend(self.CloseLoopErr, 24)
        self.AppliedThrottle = ((AppliedThrottle_h3 << 8) +
                AppliedThrottle_l8)
        self.AppliedThrottle = sign_extend(self.AppliedThrottle, 11)
        self.ModeSelect = (ModeSelect_h1 << 3) + ModeSelect_b3

    def __str__(self):
        return str(self.__dict__)

class TALON_Status_2_Feedback_20ms(object):
    def __init__(self, msg):
        SensorPositionH = msg.data[0]
        SensorPositionM = msg.data[1]
        SensorPositionL = msg.data[2]
        SensorVelocityH = msg.data[3]
        SensorVelocityL = msg.data[4]
        Current_h8 = msg.data[5]
        self.StckyFault_RevSoftLim = (msg.data[6] & 0x01)
        self.StckyFault_ForSoftLim = (msg.data[6] & 0x02) >> 1
        self.StckyFault_RevLim = (msg.data[6] & 0x04) >> 2
        self.StckyFault_ForLim = (msg.data[6] & 0x08) >> 3
        self.StckyFault_UnderVoltage = (msg.data[6] & 0x10) >> 4
        self.StckyFault_OverTemp = (msg.data[6] & 0x20) >> 5
        Current_l2 = (msg.data[6] & 0xC0) >> 6

        self.ProfileSlotSelect = (msg.data[7] & 0x40) >> 6
        self.BrakeIsEnabled = (msg.data[7] & 0x80) >> 7

        # these might be wrong?
        self.SensorPosition = ((SensorPositionH << 16) +
                               (SensorPositionM << 8) +
                               SensorPositionL)
        self.SensorPosition = sign_extend(self.SensorPosition, 24)
        self.SensorVelocity = ((SensorVelocityH << 8) +
                               SensorVelocityL)
        self.SensorVelocity = sign_extend(self.SensorVelocity, 16)
        self.Current = ((Current_h8 << 2) + Current_l2) * .125 + 0

    def __str__(self):
        return str(self.__dict__)

class TALON_Status_3_Enc_100ms(object):
    def __init__(self, msg):
        EncPositionH = msg.data[0]
        EncPositionM = msg.data[1]
        EncPositionL = msg.data[2]
        EncVelH = msg.data[3]
        EncVelL = msg.data[4]
        EncIndexRiseEventsH = msg.data[5]
        EncIndexRiseEventsL = msg.data[6]
        self.QuadIdxpin = (msg.data[7] & 0x20) >> 5
        self.QuadBpin = (msg.data[7] & 0x40) >> 6
        self.QuadApin = (msg.data[7] & 0x80) >> 7

        self.EncPosition = ((EncPositionH << 16) + (EncPositionM << 8) +
                            EncPositionL)
        self.EncPosition = sign_extend(self.EncPosition, 24)
        self.EncVel = ((EncVelH << 8) + EncVelL)
        self.EncVel = sign_extend(self.EncVel, 16)
        self.EncIndexRiseEvents = ((EncIndexRiseEventsH << 8) +
                                   EncIndexRiseEventsL)

    def __str__(self):
        return str(self.__dict__)

class TALON_Status_4_AinTempVbat_100ms(object):
    def __init__(self, msg):
        AnalogInWithOvH = msg.data[0]
        AnalogInWithOvM = msg.data[1]
        AnalogInWithOvL = msg.data[2]
        AnalogInVelH = msg.data[3]
        AnalogInVelL = msg.data[4]
        self.Temp = msg.data[5] * 0.6451612903 - 50;
        self.BatteryV = msg.data[6] * .05 + 4
        # reserved = msg.data[7] # Unnecessary

        self.AnalogInWithOv = ((AnalogInWithOvH << 16) +
                               (AnalogInWithOvM << 8) +
                               AnalogInWithOvL)
        self.AnalogInWithOv = sign_extend(self.AnalogInWithOv, 24)

        self.AnalogInVel = ((AnalogInVelH << 8) + AnalogInVelL)
        self.AnalogInVel = sign_extend(self.AnalogInVel, 16)

    def __str__(self):
        return str(self.__dict__)

class TALON_Status_5_Startup_OneShot(object):
    def __init__(self, msg):
        self.ResetCountH = msg.data[0]
        self.ResetCountL = msg.data[1]
        self.ResetFlagsH = msg.data[2]
        self.ResetFlagsL = msg.data[3]
        self.FirmVersH = msg.data[4]
        self.FirmVersL = msg.data[5]

        self.ResetCount = ((self.ResetCountH << 8) + self.ResetCountL)
        self.ResetFlagsH = ((self.ResetFlagsH << 8) + self.ResetFlagsL)
        self.FirmVers = ((self.FirmVersH << 8) + self.FirmVersL)

    def __str__(self):
        return str(self.__dict__)

class TALON_Status_6_Eol(object):
    def __init__(self, msg):
        currentAdcUncal_h2 = (msg.data[0] & 0x03)
        self.SpiCsPin_GadgeteerPin6 = (msg.data[0] & 0x80) >> 7
        currentAdcUncal_l8 = msg.data[1]
        tempAdcUncal_h2 = (msg.data[2] & 0x03)
        tempAdcUncal_l8 = msg.data[3]
        vbatAdcUncal_h2 = (msg.data[4] & 0x03)
        vbatAdcUncal_l8 = msg.data[5]
        analogAdcUncal_h2 = (msg.data[6] & 0x03)
        analogAdcUncal_l8 = msg.data[7]

        self.currentAdcUncal = ((currentAdcUncal_h2 << 8) + currentAdcUncal_l8)
        self.tempAdcUncal = ((tempAdcUncal_h2 << 8) + tempAdcUncal_l8)
        self.vbatAdcUncal = ((vbatAdcUncal_h2 << 8) + vbatAdcUncal_l8)
        self.analogAdcUncal = ((analogAdcUncal_h2 << 8) + analogAdcUncal_l8)

    def __str__(self):
        return str(self.__dict__)

class TALON_Status_7_Debug_200ms(object):
    def __init__(self, msg):
        TokenizationFails_h8 = msg.data[0]
        TokenizationFails_l8 = msg.data[1]
        LastFailedToken_h8 = msg.data[2]
        LastFailedToken_l8 = msg.data[3]
        TokenizationSuccesses_h8 = msg.data[4]
        TokenizationSuccesses_l8 = msg.data[5]

        self.TokenizationFails = ((TokenizationFails_h8 << 8) +
                                  TokenizationFails_l8)
        self.LastFailedToken = ((LastFailedToken_h8 << 8) + LastFailedToken_l8)
        self.TokenizationSuccesses = ((TokenizationSuccesses_h8 << 8) +
                                      TokenizationSuccesses_l8)

    def __str__(self):
        return str(self.__dict__)

class TALON_Param_Request(object):
    def __init__(self):
        self.ParamEnum = -1

    def pack(self):
        return bytearray([self.ParamEnum])

    def __str__(self):
        return str(self.__dict__)

class TALON_Param_Response(object):
    def __init__(self, msg = None):
        # Used when creating a "request" response to send
        if msg == None:
            self.ParamEnum = -1
            self.ParamValue = -1
        # Used to process a received message
        else:
            ParamValueL = msg.data[1]
            ParamValueML = msg.data[2]
            ParamValueMH = msg.data[3]
            ParamValueH = msg.data[4]

            ParamValue = ((ParamValueH << 24) + (ParamValueMH << 16) +
                          (ParamValueML << 8) + (ParamValueL))

            # This probably shouldn't be here, but it's only really used for
            # debugging purposes
            self.ParamEnum = msg.data[0]
            self.__dict__[Param.strings[msg.data[0]]] = ParamValue

    def setParamEnum(self, ParamEnum):
        self.ParamEnum = ParamEnum

    # def setParamValue(self, ParamValue):
    #     if (self.ParamEnum == Param.

    def pack(self):
        return bytearray([self.ParamEnum,
                          (self.ParamValue & 0x000000FF),
                          (self.ParamValue & 0x0000FF00) >> 8,
                          (self.ParamValue & 0x00FF0000) >> 16,
                          (self.ParamValue & 0xFF000000) >> 24])

    def __str__(self):
        return str(self.__dict__)

class CanTalonSRX(object):
    def __init__(self, deviceNumber, controlPeriodMs = 10):

        # Logic here
        self.controlPeriodMs = controlPeriodMs
        if (controlPeriodMs < 1):
            self.controlPeriodMs = 1
        elif (controlPeriodMs> 95):
            self.controlPeriodMs = 95
        if (deviceNumber > 15 or deviceNumber < 0):
            raise Exception("Invalid device number!")
        self.deviceNumber = deviceNumber
        self.txQueue = TxQueue()

        self.SensorPosition = 0
        self.SensorVelocity = -1
        self.prevSensorVelocity = -1
        self.failsafe = False
        self.AppliedThrottle = 0

        #Copied and pasted definitions from various structs below

        #Control 1
        self.TokenH = 0
        self.TokenL = 0
        self.DemandH = 0
        self.DemandM = 0
        self.DemandL = 0
        self.ProfileSlotSelect = 0
        self.FeedbackDeviceSelect = 0
        self.OverrideLimitSwitchEn = 0
        self.RevFeedbackSensor = 0
        self.RevMotDuringCloseLoopEn = 0
        self.OverrideBrakeType = 0
        self.ModeSelect = 0
        self.RampThrottle = 0

        self.Token = 0
        self.Demand = 0

        #Control 2
        self.Status1Ms = 0
        self.Status2Ms = 0
        self.Status3Ms = 0
        self.Status4Ms = 0

        #Control 3
        self.ZeroFeedbackSensor = 0
        self.ClearStickyFaults = 0

        #Status 1
        self.Fault_RevSoftLim = 0
        self.Fault_ForSoftLim = 0
        self.LimitSwitchClosedRev = 0
        self.LimitSwitchClosedFor = 0
        self.FeedbackDeviceSelect = 0
        self.LimitSwitchEn = 0
        self.Fault_HardwareFailure = 0
        self.Fault_RevLim = 0
        self.Fault_ForLim = 0
        self.Fault_UnderVoltage = 0
        self.Fault_OverTemp = 0
        self.CloseLoopErr = 0
        self.AppliedThrottle = 0
        self.ModeSelect = 0

        #Status 2
        self.StckyFault_RevSoftLim = 0
        self.StckyFault_ForSoftLim = 0
        self.StckyFault_RevLim = 0
        self.StckyFault_ForLim = 0
        self.StckyFault_UnderVoltage = 0
        self.StckyFault_OverTemp = 0
        self.ProfileSlotSelect = 0
        self.BrakeIsEnabled = 0
        self.SensorPosition = 0
        self.SensorVelocity = 0
        self.Current = 0

        #Status 3
        self.QuadIdxpin = 0
        self.QuadBpin = 0
        self.QuadApin = 0
        self.EncPosition = 0
        self.EncVel = 0
        self.EncIndexRiseEvents = 0

        #Status 4
        self.Temp = 0
        self.BatteryV = 0
        self.AnalogInWithOv = 0
        self.AnalogInVel = 0

        #Status 5
        self.ResetCountH = 0
        self.ResetCountL = 0
        self.ResetFlagsH = 0
        self.ResetFlagsL = 0
        self.FirmVersH = 0
        self.FirmVersL = 0

        self.ResetCount = 0
        self.ResetFlagsH = 0
        self.FirmVers = 0

        #Status 6
        self.SpiCsPin_GadgeteerPin6 = 0
        self.currentAdcUncal = 0
        self.tempAdcUncal = 0
        self.vbatAdcUncal = 0
        self.analogAdcUncal = 0

        # Status 7
        self.TokenizationFails = 0
        self.LastFailedToken = 0
        self.TokenizationSuccesses = 0

    def update(self, msg):
        """ Takes in a status message and updates proper fields within
        the object so that getters don't have trouble later. """
        if msg.arbitration_id & self.deviceNumber != self.deviceNumber:
            return # not ours!
        if msg.arbitration_id == STATUS_1 | self.deviceNumber:
            self.__dict__.update(TALON_Status_1_General_10ms(msg).__dict__)
        elif msg.arbitration_id == STATUS_2 | self.deviceNumber:
            self.__dict__.update(TALON_Status_2_Feedback_20ms(msg).__dict__)
        elif msg.arbitration_id == STATUS_3 | self.deviceNumber:
            self.__dict__.update(TALON_Status_3_Enc_100ms(msg).__dict__)
        elif msg.arbitration_id == STATUS_4 | self.deviceNumber:
            self.__dict__.update(TALON_Status_4_AinTempVbat_100ms(msg).__dict__)
        elif msg.arbitration_id == STATUS_5 | self.deviceNumber:
            self.__dict__.update(TALON_Status_5_Startup_OneShot(msg).__dict__)
        elif msg.arbitration_id == STATUS_6 | self.deviceNumber:
            self.__dict__.update(TALON_Status_6_Eol(msg).__dict__)
        elif msg.arbitration_id == STATUS_7 | self.deviceNumber:
            self.__dict__.update(TALON_Status_7_Debug_200ms(msg).__dict__)
        elif msg.arbitration_id == PARAM_RESPONSE | self.deviceNumber:
            response = TALON_Param_Response(msg)
            self.__dict__.update(TALON_Param_Response(msg).__dict__)

            print response.__dict__
            print Param.strings[response.ParamEnum]
            print "Value:", response.__dict__[Param.strings[response.ParamEnum]]

            print self.get_param_response(response.ParamEnum)
            print

    def setpoint_callback(self, msg):
        # Since this will be threaded, we're going to create the message
        # here in order to boost efficiency.
        # From deg / sec -> encoder ticks / 100 ms
        # x (deg / sec) * 1/10 (sec / 100 ms) * 1/360 (rev / deg) * 5000 (t/rev)
        # self.set_throttle(msg.velocity_setpoint)
        self.set_velocity(msg.velocity_setpoint * 5000.0 / 3600.0)

    # should really be set_raw
    def set_velocity(self, value):
        if value > 1023:
            value = 1023
        elif value < -1023:
            value = -1023
        self.set_demand(value)

    def set_throttle(self, value):
        if (value > 1):
            value = 1.0
        elif value < -1:
            value = -1.0
        self.set_demand(1023 * value)

    # Combined function for efficiency
    def set_throttle_and_mode(self, value, mode):
        if (value > 1):
            value = 1.0
        elif value < -1:
            value = -1.0
        data = TALON_Control_1_General_10ms()
        data.set_demand(1023 * value)
        data.ModeSelect = mode
        self.txQueue.put(can.Message(arbitration_id = CONTROL_1 |
                                     self.deviceNumber, data = data.pack()))

    def set_mode(self, value):
        data = TALON_Control_1_General_10ms()
        data.ModeSelect = value
        self.txQueue.put(can.Message(arbitration_id = CONTROL_1 |
                                     self.deviceNumber, data = data.pack()))

    def set_demand(self, param):
        # if param != 0:
        #     print "setting demand:", param
        data = TALON_Control_1_General_10ms()
        data.set_demand(param)
        self.txQueue.put(can.Message(arbitration_id = CONTROL_1 |
                                     self.deviceNumber, data = data.pack()))

    # Works fine for ints, doesn't work for floats
    def set_param(self, paramEnum, value):
        rawbits = 0
        if (paramEnum == Param.eProfileParamSlot0_P or
                paramEnum == Param.eProfileParamSlot0_I or
                paramEnum == Param.eProfileParamSlot0_D or
                paramEnum == Param.eProfileParamSlot1_P or
                paramEnum == Param.eProfileParamSlot1_I or
                paramEnum == Param.eProfileParamSlot1_D):
            value = min(value, 1023.0)
            value = max(value, 0)
            # works fine for positive numbers
            rawbits = int(value * FLOAT_TO_FXP)
        elif (paramEnum == Param.eProfileParamSlot0_F or
                paramEnum == Param.eProfileParamSlot1_F):
            value = min(value, 512)
            value = max(value, -512)
            # works fine for positive numbers, DOES NOT WORK for negatives
            rawbits = int(value * FLOAT_TO_FXP)
        else:
            rawbits = int(value)
        self.set_param_raw(paramEnum, rawbits)

    def set_param_raw(self, paramEnum, rawbits):
        data = TALON_Param_Response()
        data.ParamEnum = paramEnum
        data.ParamValue = rawbits
        self.txQueue.put(can.Message(arbitration_id = PARAM_SET |
                                     self.deviceNumber, data = data.pack()))

    def request_param(self, paramEnum):
        data = TALON_Param_Request()
        data.ParamEnum = paramEnum
        self.txQueue.put(can.Message(arbitration_id = PARAM_REQUEST |
                                     self.deviceNumber, data = data.pack()))

    def get_param_response(self, paramEnum):
        if (paramEnum == Param.eProfileParamSlot0_P or
                paramEnum == Param.eProfileParamSlot0_I or
                paramEnum == Param.eProfileParamSlot0_D or
                paramEnum == Param.eProfileParamSlot0_F or
                paramEnum == Param.eProfileParamSlot1_P or
                paramEnum == Param.eProfileParamSlot1_I or
                paramEnum == Param.eProfileParamSlot1_D or
                paramEnum == Param.eProfileParamSlot1_F or
                paramEnum == Param.eCurrent or
                paramEnum == Param.eTemp or
                paramEnum == Param.eBatteryV):
            return self.__dict__[Param.strings[paramEnum]] * FXP_TO_FLOAT
        else:
            return self.__dict__[Param.strings[paramEnum]]

    def safety_check(self, kill = True):
        # this means that the safety check should be the first operation in loop
        # if self.failsafe and kill:
        #     print "should be in here!"
        #     self.txQueue.set_freeze()
        #     with self.txQueue.mutex:
        #         self.txQueue.queue.clear()
        #     self.set_throttle(0)
        #     return

        if self.failsafe:
            return

        # print "velocity:",self.SensorVelocity

        cutoff = 10
        throttle_cutoff = 80
        # If we go from a high magnitude velocity to low magnitude, cutoff
        if ((self.prevSensorVelocity < (-1 * cutoff) or
              self.prevSensorVelocity > cutoff) and
             abs(self.SensorVelocity) <= cutoff):
            self.txQueue = TxQueue() # make a new queue, effectively clear
            self.set_throttle_and_mode(0, Mode.DutyCycle)
            self.failsafe = True
            self.txQueue.set_freeze()
            print "Activating failsafe!"
        # elif (abs(self.AppliedThrottle) > throttle_cutoff and
        #       self.SensorVelocity == 0 and self.prevSensorVelocity == 0):
        #     print self.AppliedThrottle, self.SensorVelocity
        #     print "other failsafe?"
        #     self.txQueue = TxQueue() # make a new queue, effectively clear
        #     self.set_throttle_and_mode(0, Mode.DutyCycle)
        #     self.failsafe = True
        #     self.txQueue.set_freeze()

        self.prevSensorVelocity = self.SensorVelocity

    def resume(self):
        self.failsafe = False
        self.txQueue.set_freeze(False)

def sign_extend(value, bits):
    sign_bit = 1 << (bits - 1)
    return (value & (sign_bit - 1)) - (value & sign_bit)
