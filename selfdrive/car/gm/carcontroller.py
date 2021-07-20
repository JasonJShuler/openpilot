from cereal import car
from common.realtime import DT_CTRL
from common.numpy_fast import interp, clip
from selfdrive.config import Conversions as CV
from selfdrive.car import apply_std_steer_torque_limits, create_gas_command
from selfdrive.car.gm import gmcan
from selfdrive.car.gm.values import DBC, CanBus, CarControllerParams, REGEN_CARS
from opendbc.can.packer import CANPacker

VisualAlert = car.CarControl.HUDControl.VisualAlert

def actuator_hystereses(final_pedal, pedal_steady):
  # hyst params... TODO: move these to VehicleParams
  pedal_hyst_gap = 0.01    # don't change pedal command for small oscillations within this value

  # for small pedal oscillations within pedal_hyst_gap, don't change the pedal command
  if final_pedal == 0.:
    pedal_steady = 0.
  elif final_pedal > pedal_steady + pedal_hyst_gap:
    pedal_steady = final_pedal - pedal_hyst_gap
  elif final_pedal < pedal_steady - pedal_hyst_gap:
    pedal_steady = final_pedal + pedal_hyst_gap
  final_pedal = pedal_steady

  return final_pedal, pedal_steady

last_logged_pedal = 0.0
class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.pedal_steady = 0.
    self.start_time = 0.
    self.apply_steer_last = 0
    self.lka_icon_status_last = (False, False)
    self.steer_rate_limited = False
    self.car_fingerprint = CP.carFingerprint

    self.params = CarControllerParams()

    self.packer_pt = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.packer_obj = CANPacker(DBC[CP.carFingerprint]['radar'])
    self.packer_ch = CANPacker(DBC[CP.carFingerprint]['chassis'])

  def update(self, enabled, CS, frame, actuators,
             hud_v_cruise, hud_show_lanes, hud_show_car, hud_alert):

    P = self.params

    # Send CAN commands.
    can_sends = []

    # STEER
    lkas_enabled = enabled and not CS.out.steerWarning and CS.out.vEgo > P.MIN_STEER_SPEED
    if (frame % P.STEER_STEP) == 0:
      if lkas_enabled:
        new_steer = actuators.steer * P.STEER_MAX
        apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, P)
        self.steer_rate_limited = new_steer != apply_steer
      else:
        apply_steer = 0

      self.apply_steer_last = apply_steer
      idx = (frame // P.STEER_STEP) % 4

      can_sends.append(gmcan.create_steering_control(self.packer_pt, CanBus.POWERTRAIN, apply_steer, idx, lkas_enabled))

    ### GAS/BRAKE ###
    # no output if not enabled, but keep sending keepalive messages
    # treat pedals as one
    
    if CS.CP.enableGasInterceptor and self.car_fingerprint in REGEN_CARS:
      #It seems in L mode, accel / decel point is around 1/5
      #-1-------AEB------0----regen---0.15-------accel----------+1
      # Shrink gas request to 0.85, have it start at 0.2
      # Shrink brake request to 0.85, first 0.15 gives regen, rest gives AEB
      zero = 40/256
      gas = (1-zero) * actuators.gas + zero
      regen = clip(actuators.brake, 0., zero) # Make brake the same size as gas, but clip to regen
      # aeb = actuators.brake*(1-zero)-regen # For use later, braking more than regen
      final_pedal = gas - regen
      if not enabled:
        # Since no input technically maps to 0.15, send 0.0 when not enabled to avoid
        # controls mismatch.
        final_pedal = 0.0
      #TODO: Use friction brake via AEB for harder braking
      
      # apply pedal hysteresis and clip the final output to valid values.
      final_pedal, self.pedal_steady = actuator_hystereses(final_pedal, self.pedal_steady)
      pedal_gas = clip(final_pedal, 0., 1.)

      if (frame % 4) == 0:
        idx = (frame // 4) % 4
        # send exactly zero if apply_gas is zero. Interceptor will send the max between read value and apply_gas.
        # This prevents unexpected pedal range rescaling
        can_sends.append(create_gas_command(self.packer_pt, pedal_gas, idx))

    # Send dashboard UI commands (ACC status), 25hz
    if (frame % 4) == 0:
      send_fcw = hud_alert == VisualAlert.fcw
      can_sends.append(gmcan.create_acc_dashboard_command(self.packer_pt, CanBus.POWERTRAIN, enabled, hud_v_cruise * CV.MS_TO_KPH, hud_show_car, send_fcw))

    # Radar needs to know current speed and yaw rate (50hz),
    # and that ADAS is alive (10hz)
    time_and_headlights_step = 10
    # tt = frame * DT_CTRL

    if frame % time_and_headlights_step == 0:
      idx = (frame // time_and_headlights_step) % 4
      #can_sends.append(gmcan.create_adas_time_status(CanBus.OBSTACLE, int((tt - self.start_time) * 60), idx))
      #can_sends.append(gmcan.create_adas_headlights_status(self.packer_obj, CanBus.OBSTACLE))

    speed_and_accelerometer_step = 2
    if frame % speed_and_accelerometer_step == 0:
      idx = (frame // speed_and_accelerometer_step) % 4
      #can_sends.append(gmcan.create_adas_steering_status(CanBus.OBSTACLE, idx))
      #can_sends.append(gmcan.create_adas_accelerometer_speed_status(CanBus.OBSTACLE, CS.out.vEgo, idx))

    if frame % P.ADAS_KEEPALIVE_STEP == 0:
      can_sends += gmcan.create_adas_keepalive(CanBus.POWERTRAIN)

    # Show green icon when LKA torque is applied, and
    # alarming orange icon when approaching torque limit.
    # If not sent again, LKA icon disappears in about 5 seconds.
    # Conveniently, sending camera message periodically also works as a keepalive.
    lka_active = lkas_enabled == 1
    lka_critical = lka_active and abs(actuators.steer) > 0.9
    lka_icon_status = (lka_active, lka_critical)
    if frame % P.CAMERA_KEEPALIVE_STEP == 0 or lka_icon_status != self.lka_icon_status_last:
      steer_alert = hud_alert == VisualAlert.steerRequired
      can_sends.append(gmcan.create_lka_icon_command(CanBus.SW_GMLAN, lka_active, lka_critical, steer_alert))
      self.lka_icon_status_last = lka_icon_status

    return can_sends
