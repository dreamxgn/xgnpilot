from cereal import car
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.volkswagen import volkswagencan
from selfdrive.car.volkswagen.values import DBC_FILES, CANBUS, MQB_LDW_MESSAGES, BUTTON_STATES, CarControllerParams as P
from opendbc.can.packer import CANPacker
from common.params import Params
import cereal.messaging as messaging
from selfdrive.config import Conversions as CV

VisualAlert = car.CarControl.HUDControl.VisualAlert

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0

    self.packer_pt = CANPacker(DBC_FILES.mqb)

    self.hcaSameTorqueCount = 0
    self.hcaEnabledFrameCount = 0
    self.graButtonStatesToSend = None
    self.graMsgSentCount = 0
    self.graMsgStartFramePrev = 0
    self.graMsgBusCounterPrev = None

    self.params = Params()
    self.timer = 0
    self.final_speed_kph = 0
    self.init_speed = 0
    self.current_speed = 0
    self.v_set_dis = 0
    self.button_type = 0
    self.button_select = 0
    self.button_count = 0
    self.target_speed = 0
    self.resume_count = 0
    self.t_interval = 7
    self.sl_force_active_timer = 0
    self.slc_state = 0
    self.slc_active_stock = False
    self.is_metric = self.params.get_bool("IsMetric")
    self.is_vison_control = self.params.get_bool("TurnVisionControl")
    self.sm = messaging.SubMaster(['controlsState', 'longitudinalPlan'])
    self.v_cruise_kph_prev = self.sm['controlsState'].vCruise


    self.steer_rate_limited = False

  def update(self, c, enabled, CS, frame, ext_bus, actuators, visual_alert, left_lane_visible, right_lane_visible, left_lane_depart, right_lane_depart):
    """ Controls thread """

    self.sm.update(0)
    self.v_cruise_kph_prev = self.sm['controlsState'].vCruise

    can_sends = []

    # **** Steering Controls ************************************************ #

    if frame % P.HCA_STEP == 0:
      self.steer_ctl(c=c,CS=CS,actuators=actuators,frame=frame,can_sends=can_sends)

    # **** HUD Controls ***************************************************** #

    if frame % P.LDW_STEP == 0:
      self.hud_ctl(visual_alert=visual_alert,can_sends=can_sends,enabled=enabled,CS=CS,
                  left_lane_visible=left_lane_visible,right_lane_visible=right_lane_visible,
                  left_lane_depart=left_lane_depart,right_lane_depart=right_lane_depart)

    # **** ACC Button Controls ********************************************** #
    if CS.CP.pcmCruise:
      # ***** cancel acc 、stop_and_go ***************************** #
      self.acc_std_ctl(enabled=enabled,CS=CS,frame=frame,can_sends=can_sends,ext_bus=ext_bus)

      # ***** vison speed control ***************************** #
      self.acc_vison_speed_ctl(enabled=enabled,CS=CS,frame=frame,can_sends=can_sends,ext_bus=ext_bus)

    self.graMsgBusCounterPrev = CS.graMsgBusCounter
    new_actuators = actuators.copy()
    new_actuators.steer = self.apply_steer_last / P.STEER_MAX

    return new_actuators, can_sends

  def steer_ctl(self,c,CS,actuators,frame,can_sends):
      # Logic to avoid HCA state 4 "refused":
      #   * Don't steer unless HCA is in state 3 "ready" or 5 "active"
      #   * Don't steer at standstill
      #   * Don't send > 3.00 Newton-meters torque
      #   * Don't send the same torque for > 6 seconds
      #   * Don't send uninterrupted steering for > 360 seconds
      # One frame of HCA disabled is enough to reset the timer, without zeroing the
      # torque value. Do that anytime we happen to have 0 torque, or failing that,
      # when exceeding ~1/3 the 360 second timer.

      if c.active and CS.out.vEgo > CS.CP.minSteerSpeed and not (CS.out.standstill or CS.out.steerError or CS.out.steerWarning):
        new_steer = int(round(actuators.steer * P.STEER_MAX))
        apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, P)
        self.steer_rate_limited = new_steer != apply_steer
        if apply_steer == 0:
          hcaEnabled = False
          self.hcaEnabledFrameCount = 0
        else:
          self.hcaEnabledFrameCount += 1
          if self.hcaEnabledFrameCount >= 118 * (100 / P.HCA_STEP):  # 118s
            hcaEnabled = False
            self.hcaEnabledFrameCount = 0
          else:
            hcaEnabled = True
            if self.apply_steer_last == apply_steer:
              self.hcaSameTorqueCount += 1
              if self.hcaSameTorqueCount > 1.9 * (100 / P.HCA_STEP):  # 1.9s
                apply_steer -= (1, -1)[apply_steer < 0]
                self.hcaSameTorqueCount = 0
            else:
              self.hcaSameTorqueCount = 0
      else:
        hcaEnabled = False
        apply_steer = 0

      self.apply_steer_last = apply_steer
      idx = (frame / P.HCA_STEP) % 16
      can_sends.append(volkswagencan.create_mqb_steering_control(self.packer_pt, CANBUS.pt, apply_steer,
                                                                 idx, hcaEnabled))

  def hud_ctl(self,visual_alert,can_sends,enabled,CS,left_lane_visible,right_lane_visible,
              left_lane_depart, right_lane_depart):
      if visual_alert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = MQB_LDW_MESSAGES["laneAssistTakeOverSilent"]
      else:
        hud_alert = MQB_LDW_MESSAGES["none"]

      can_sends.append(volkswagencan.create_mqb_hud_control(self.packer_pt, CANBUS.pt, enabled,
                                                            CS.out.steeringPressed, hud_alert, left_lane_visible,
                                                            right_lane_visible, CS.ldw_stock_values,
                                                            left_lane_depart, right_lane_depart))

  def acc_std_ctl(self,enabled,CS,frame,can_sends,ext_bus):
    cancel_acc = (not enabled and CS.out.cruiseState.enabled)
    stop_ang_go = enabled and CS.out.cruiseState.enabled and CS.esp_hold_confirmation
    send_ready = CS.graMsgBusCounter != self.graMsgBusCounterPrev

    if cancel_acc:
      self.graButtonStatesToSend = BUTTON_STATES.copy()
      self.graButtonStatesToSend["cancel"] = True

    if stop_ang_go and (frame % P.GRA_VBP_COUNT == 0):
      self.graButtonStatesToSend = BUTTON_STATES.copy()
      self.graButtonStatesToSend["resumeCruise"] = True

    if send_ready and (cancel_acc or stop_ang_go) and self.graButtonStatesToSend is not None:
      idx = (CS.graMsgBusCounter + 1) % 16
      can_sends.append(volkswagencan.create_mqb_acc_buttons_control(self.packer_pt, ext_bus, 
      self.graButtonStatesToSend, CS, idx))
  
  def acc_vison_speed_ctl(self,enabled,CS,frame,can_sends,ext_bus):
    runing = (not CS.esp_hold_confirmation) and CS.out.cruiseState.enabled \
              and (not CS.out.gasPressed) and CS.out.cruiseState.enabled
    if runing and self.is_vison_control:
      cruise_button = self.get_cruise_buttons(CS,self.v_cruise_kph_prev)
      if (cruise_button is not None) and (frame % P.GRA_VBP_COUNT == 0):
        if cruise_button == 1:
          self.graButtonStatesToSend = BUTTON_STATES.copy()
          self.graButtonStatesToSend["resumeCruise"] = True
        elif cruise_button == 2:
          self.graButtonStatesToSend = BUTTON_STATES.copy()
          self.graButtonStatesToSend["setCruise"] = True
        
        idx = (CS.graMsgBusCounter + 1) % 16
        can_sends.append(volkswagencan.create_mqb_acc_buttons_control(self.packer_pt, ext_bus, \
                        self.graButtonStatesToSend, CS, idx))

  def get_cruise_buttons_status(self, CS):
    if not CS.cruiseState.enabled or CS.buttonStates["accelCruise"] or CS.buttonStates["decelCruise"] or CS.buttonStates["setCruise"] or CS.buttonStates["resumeCruise"]:
      self.timer = 80
    elif self.timer:
      self.timer -= 1
    else:
      return 1
    return 0

  def get_target_speed(self, v_cruise_kph_prev):
    v_cruise_kph = v_cruise_kph_prev
    return v_cruise_kph

  def get_button_type(self, button_type):
    self.type_status = "type_" + str(button_type)
    self.button_picker = getattr(self, self.type_status, lambda:"default")
    return self.button_picker()

  def reset_button(self):
    if self.button_type != 3:
      self.button_type = 0

  def type_default(self):
    self.button_type = 0
    return None

  def type_0(self):
    self.button_count = 0
    self.target_speed = self.init_speed
    speed_diff = round(self.target_speed - self.v_set_dis)
    if speed_diff > 0:
      self.button_type = 1
    elif speed_diff < 0:
      self.button_type = 2
    return None

  def type_1(self):
    cruise_button = 1
    self.button_count += 1
    if self.target_speed == self.v_set_dis:
      self.button_count = 0
      self.button_type = 3
    elif self.button_count > 10:
      self.button_count = 0
      self.button_type = 3
    return cruise_button

  def type_2(self):
    cruise_button = 2
    self.button_count += 1
    if self.target_speed == self.v_set_dis:
      self.button_count = 0
      self.button_type = 3
    elif self.button_count > 10:
      self.button_count = 0
      self.button_type = 3
    return cruise_button

  def type_3(self):
    cruise_button = None
    self.button_count += 1
    if self.button_count > self.t_interval:
      self.button_type = 0
    return cruise_button

  def get_curve_speed(self, target_speed_kph, v_cruise_kph_prev):
    vision_v_cruise_kph = 255
    if self.is_vison_control:
      vision_v_cruise_kph = float(float(self.sm['longitudinalPlan'].visionTurnSpeed) * CV.MS_TO_KPH)
      if int(vision_v_cruise_kph) == int(v_cruise_kph_prev):
        vision_v_cruise_kph = 255
      vision_v_cruise_kph = min(target_speed_kph, vision_v_cruise_kph)
    
    return min(target_speed_kph, vision_v_cruise_kph)

  def get_button_control(self, CS, final_speed, v_cruise_kph_prev):
    self.init_speed = round(min(final_speed, v_cruise_kph_prev) * CV.KPH_TO_MPH) if not self.is_metric else round(min(final_speed, v_cruise_kph_prev))
    self.v_set_dis = round(CS.out.cruiseState.speed * CV.MS_TO_MPH) if not self.is_metric else round(CS.out.cruiseState.speed * CV.MS_TO_KPH)
    cruise_button = self.get_button_type(self.button_type)
    return cruise_button

  def get_cruise_buttons(self, CS,v_cruise_kph_prev):
    cruise_button = None
    if not self.get_cruise_buttons_status(CS):
      pass
    elif CS.cruiseState.enabled:
      set_speed_kph = self.get_target_speed(v_cruise_kph_prev)
      target_speed_kph = min(v_cruise_kph_prev, set_speed_kph)

      if self.is_vison_control:
        self.final_speed_kph = self.get_curve_speed(target_speed_kph,v_cruise_kph_prev)
      else:
        self.final_speed_kph = target_speed_kph

      cruise_button = self.get_button_control(CS, self.final_speed_kph,v_cruise_kph_prev)
    return cruise_button