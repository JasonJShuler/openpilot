from cereal import car
from common.numpy_fast import mean
from selfdrive.config import Conversions as CV
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.gm.values import DBC, CAR, AccState, CanBus, \
                                    CruiseButtons, STEER_THRESHOLD, \
                                    REGEN_CARS


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["ECMPRDNL"]["PRNDL"]

  def update(self, pt_cp):
    ret = car.CarState.new_message()

    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = pt_cp.vl["ASCMSteeringButton"]['ACCButtons']

    ret.wheelSpeeds.fl = pt_cp.vl["EBCMWheelSpdFront"]['FLWheelSpd'] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = pt_cp.vl["EBCMWheelSpdFront"]['FRWheelSpd'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = pt_cp.vl["EBCMWheelSpdRear"]['RLWheelSpd'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = pt_cp.vl["EBCMWheelSpdRear"]['RRWheelSpd'] * CV.KPH_TO_MS
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    #ret.vEgo = pt_cp.vl["ECMVehicleSpeed"]["VehicleSpeed"] * CV.MPH_TO_MS
    ret.standstill = not ret.vEgoRaw > 0.1

    ret.steeringAngleDeg = pt_cp.vl["PSCMSteeringAngle"]['SteeringWheelAngle']
    ret.steeringRateDeg = pt_cp.vl["PSCMSteeringAngle"]['SteeringWheelRate']
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["ECMPRDNL"]['PRNDL'], None))
    ret.brake = pt_cp.vl["EBCMBrakePedalPosition"]['BrakePedalPosition'] / 0xd0
    # Brake pedal's potentiometer returns near-zero reading even when pedal is not pressed.
    if ret.brake < 10/0xd0:
      ret.brake = 0.


    ret.gas = pt_cp.vl["AcceleratorPedal"]['AcceleratorPedal'] / 254.
    # Disable gaspress event for gas interceptor, indistinguishable
    # TODO: compute whether user is applying gas, and use that to disable OP
    if not self.CP.enableGasInterceptor:
      ret.gasPressed = ret.gas > 1e-5
    else:
      ret.gasPressed = False

    ret.steeringTorque = pt_cp.vl["PSCMStatus"]['LKADriverAppldTrq']
    ret.steeringTorqueEps = pt_cp.vl["PSCMStatus"]['LKATotalTorqueDelivered']
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # 1 - open, 0 - closed
    ret.doorOpen = (pt_cp.vl["BCMDoorBeltStatus"]['FrontLeftDoor'] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]['FrontRightDoor'] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]['RearLeftDoor'] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]['RearRightDoor'] == 1)

    # 1 - latched
    ret.seatbeltUnlatched = pt_cp.vl["BCMDoorBeltStatus"]['LeftSeatBelt'] == 0
    ret.leftBlinker = pt_cp.vl["BCMTurnSignals"]['TurnSignals'] == 1
    ret.rightBlinker = pt_cp.vl["BCMTurnSignals"]['TurnSignals'] == 2

    self.park_brake = pt_cp.vl["EPBStatus"]['EPBClosed']
    self.main_on = bool(pt_cp.vl["ECMEngineStatus"]['CruiseMainOn'])
    ret.espDisabled = pt_cp.vl["ESPStatus"]['TractionControlOn'] != 1
    self.pcm_acc_status = pt_cp.vl["ASCMActiveCruiseControlStatus"]['ACCCmdActive']
    if self.CP.enableGasInterceptor:
      ret.cruiseState.available = not bool(pt_cp.vl["ECMEngineStatus"]['CruiseMainOn'])
    else:
      ret.cruiseState.available = bool(pt_cp.vl["ECMEngineStatus"]['CruiseMainOn'])
    ret.cruiseState.enabled = self.pcm_acc_status != AccState.OFF
    ret.cruiseState.standstill = self.pcm_acc_status == AccState.STANDSTILL

    # Regen braking is braking
    self.regen_pressed = False
    if self.car_fingerprint in REGEN_CARS:
      self.regen_pressed = bool(pt_cp.vl["EBCMRegenPaddle"]['RegenPaddle'])
    ret.brakePressed = ret.brake > 1e-5 or self.regen_pressed

    brake_light_enable = False
    if self.car_fingerprint == CAR.BOLT:
      if ret.aEgo < -1.3:
        brake_light_enable = True

    ret.brakeLights = ret.brakePressed or self.regen_pressed or brake_light_enable
    # 0 - inactive, 1 - active, 2 - temporary limited, 3 - failed
    self.lkas_status = pt_cp.vl["PSCMStatus"]['LKATorqueDeliveredStatus']
    ret.steerWarning = self.lkas_status not in [0, 1]

    if self.car_fingerprint == CAR.BOLT:
      self.HVBvoltage = pt_cp.vl["BECMBatteryVoltageCurrent"]['HVBatteryVoltage']
      self.HVBcurrent = pt_cp.vl["BECMBatteryVoltageCurrent"]['HVBatteryCurrent']
      ret.hvBpower = self.HVBvoltage * self.HVBcurrent / 1000   #kW
    return ret

  @staticmethod
  def get_can_parser(CP):
    # this function generates lists for signal, messages and initial values
    signals = [
      # sig_name, sig_address, default
      ("BrakePedalPosition", "EBCMBrakePedalPosition", 0),
      ("FrontLeftDoor", "BCMDoorBeltStatus", 0),
      ("FrontRightDoor", "BCMDoorBeltStatus", 0),
      ("RearLeftDoor", "BCMDoorBeltStatus", 0),
      ("RearRightDoor", "BCMDoorBeltStatus", 0),
      ("LeftSeatBelt", "BCMDoorBeltStatus", 0),
      ("RightSeatBelt", "BCMDoorBeltStatus", 0),
      ("TurnSignals", "BCMTurnSignals", 0),
      ("AcceleratorPedal", "AcceleratorPedal", 0),
      ("CruiseState", "AcceleratorPedal2", 0),
      ("ACCButtons", "ASCMSteeringButton", CruiseButtons.UNPRESS),
      ("SteeringWheelAngle", "PSCMSteeringAngle", 0),
      ("SteeringWheelRate", "PSCMSteeringAngle", 0),
      ("FLWheelSpd", "EBCMWheelSpdFront", 0),
      ("FRWheelSpd", "EBCMWheelSpdFront", 0),
      ("RLWheelSpd", "EBCMWheelSpdRear", 0),
      ("RRWheelSpd", "EBCMWheelSpdRear", 0),
      ("PRNDL", "ECMPRDNL", 0),
      ("LKADriverAppldTrq", "PSCMStatus", 0),
      ("LKATorqueDeliveredStatus", "PSCMStatus", 0),
      ("TractionControlOn", "ESPStatus", 0),
      ("EPBClosed", "EPBStatus", 0),
      ("CruiseMainOn", "ECMEngineStatus", 0),
      ("ACCCmdActive", "ASCMActiveCruiseControlStatus", 0),
      ("LKATotalTorqueDelivered", "PSCMStatus", 0),
      ("VehicleSpeed", "ECMVehicleSpeed", 0),
    ]

    if CP.carFingerprint == CAR.VOLT or CP.carFingerprint == CAR.BOLT:
      signals += [
        ("RegenPaddle", "EBCMRegenPaddle", 0),
        ("HVBatteryVoltage", "BECMBatteryVoltageCurrent", 0),
        ("HVBatteryCurrent", "BECMBatteryVoltageCurrent", 0),
      ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, [], CanBus.POWERTRAIN)
