// board enforces
//   in-state
//      accel set/resume
//   out-state
//      cancel button
//      regen paddle
//      accel rising edge
//      brake rising edge
//      brake > 0mph

const int GM_MAX_STEER = 300;
const int GM_MAX_RT_DELTA = 128;          // max delta torque allowed for real time checks
const uint32_t GM_RT_INTERVAL = 250000;    // 250ms between real time checks
const int GM_MAX_RATE_UP = 7;
const int GM_MAX_RATE_DOWN = 17;
const int GM_DRIVER_TORQUE_ALLOWANCE = 50;
const int GM_DRIVER_TORQUE_FACTOR = 4;
const int GM_MAX_GAS = 3072;
const int GM_MAX_REGEN = 1404;
const int GM_MAX_BRAKE = 350;

int gm_lkas_counter_prev = 0;
int gm_stock_lkas = 1;
int gm_brake_prev = 0;
int gm_gas_prev = 0;
bool gm_moving = false;
// silence everything if stock car control ECUs are still online
bool gm_ascm_detected = 0;
bool gm_ignition_started = 0;
int gm_rt_torque_last = 0;
int gm_desired_torque_last = 0;
uint32_t gm_ts_last = 0;
struct sample_t gm_torque_driver;         // last few driver torques measured



static void gm_set_controlsallowed(bool val) {
  controls_allowed = val;
  gm_stock_lkas = !val;
}



static void gm_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  int bus_number = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  if (addr == 388) {
    int torque_driver_new = ((GET_BYTE(to_push, 6) & 0x7) << 8) | GET_BYTE(to_push, 7);
    torque_driver_new = to_signed(torque_driver_new, 11);
    // update array of samples
    update_sample(&gm_torque_driver, torque_driver_new);
  }

  if ((addr == 0x1F1) && (bus_number == 0)) {
    //Bit 5 should be ignition "on"
    //Backup plan is Bit 2 (accessory power)
    bool ign = (GET_BYTE(to_push, 0) & 0x20) != 0;
    gm_ignition_started = ign;
  }

  // sample speed, really only care if car is moving or not
  // rear left wheel speed
  if (addr == 842) {
    gm_moving = GET_BYTE(to_push, 0) | GET_BYTE(to_push, 1);
  }

  // Check if ASCM or LKA camera are online
  // on powertrain bus.
  // 384 = ASCMLKASteeringCmd
  // 715 = ASCMGasRegenCmd
  if ((bus_number == 0) && ((addr == 384) || (addr == 715))) {
    gm_ascm_detected = 1;
    gm_set_controlsallowed(0);
  }

  // ACC steering wheel buttons
  if (addr == 481) {
    int button = (GET_BYTE(to_push, 5) & 0x70) >> 4;
    switch (button) {
      case 2:  // resume
      case 3:  // set
        gm_set_controlsallowed(1);
        break;
      case 6:  // cancel
        gm_set_controlsallowed(0);
        break;
      default:
        break;  // any other button is irrelevant
    }
  }

  // exit controls on rising edge of brake press or on brake press when
  // speed > 0
  if (addr == 241) {
    int brake = GET_BYTE(to_push, 1);
    // Brake pedal's potentiometer returns near-zero reading
    // even when pedal is not pressed
    if (brake < 10) {
      brake = 0;
    }
    if (brake && (!gm_brake_prev || gm_moving)) {
       gm_set_controlsallowed(0);
    }
    gm_brake_prev = brake;
  }

  // exit controls on rising edge of gas press
  if (addr == 417) {
    int gas = GET_BYTE(to_push, 6);
    if (gas && !gm_gas_prev && long_controls_allowed) {
      gm_set_controlsallowed(0);
    }
    gm_gas_prev = gas;
  }

  // exit controls on regen paddle
  if (addr == 189) {
    bool regen = GET_BYTE(to_push, 0) & 0x20;
    if (regen) {
      gm_set_controlsallowed(0);
    }
  }
}




// all commands: gas/regen, friction brake and steering
// if controls_allowed and no pedals pressed
//     allow all commands up to limit
// else
//     block all commands that produce actuation

static int gm_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {

  int tx = 1;

  // There can be only one! (ASCM)
  if (gm_ascm_detected) {
    tx = 0;
  }

  // disallow actuator commands if gas or brake (with vehicle moving) are pressed
  // and the the latching controls_allowed flag is True
  int pedal_pressed = gm_gas_prev || (gm_brake_prev && gm_moving);
  bool current_controls_allowed = controls_allowed && !pedal_pressed;

  int addr = GET_ADDR(to_send);

  // BRAKE: safety check
  if (addr == 789) {
    int brake = ((GET_BYTE(to_send, 0) & 0xFU) << 8) + GET_BYTE(to_send, 1);
    brake = (0x1000 - brake) & 0xFFF;
    if (!current_controls_allowed || !long_controls_allowed) {
      if (brake != 0) {
        tx = 0;
      }
    }
    if (brake > GM_MAX_BRAKE) {
      tx = 0;
    }
  }

  // LKA STEER: safety check
  if (addr == 384) {
    if (gm_stock_lkas) {
      tx = 1;
      return tx;
      }


    uint32_t vals[4];
    vals[0] = 0x00000000U;
    vals[1] = 0x10000fffU;
    vals[2] = 0x20000ffeU;
    vals[3] = 0x30000ffdU;

    int rolling_counter = (GET_BYTE(to_send, 0) & 0x3U) >> 4;
    //TODO: critical section
    int expected_counter = (gm_lkas_counter_prev + 1) % 4;
    
    int desired_torque = ((GET_BYTE(to_send, 0) & 0x7U) << 8) + GET_BYTE(to_send, 1);
    uint32_t ts = TIM2->CNT;
    bool violation = 0;
    desired_torque = to_signed(desired_torque, 11);

    if (current_controls_allowed) {
      violation |= rolling_counter != expected_counter;
      // *** global torque limit check ***
      violation |= max_limit_check(desired_torque, GM_MAX_STEER, -GM_MAX_STEER);

      // *** torque rate limit check ***
      violation |= driver_limit_check(desired_torque, gm_desired_torque_last, &gm_torque_driver,
        GM_MAX_STEER, GM_MAX_RATE_UP, GM_MAX_RATE_DOWN,
        GM_DRIVER_TORQUE_ALLOWANCE, GM_DRIVER_TORQUE_FACTOR);

      // used next time
      gm_desired_torque_last = desired_torque;

      // *** torque real time rate limit check ***
      violation |= rt_rate_limit_check(desired_torque, gm_rt_torque_last, GM_MAX_RT_DELTA);

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = get_ts_elapsed(ts, gm_ts_last);
      if (ts_elapsed > GM_RT_INTERVAL) {
        gm_rt_torque_last = desired_torque;
        gm_ts_last = ts;
      }
    }

    // no torque if controls is not allowed
    if (!current_controls_allowed && (desired_torque != 0)) {
      violation = 1;
    }

    // reset to 0 if either controls is not allowed or there's a violation
    if (violation || !current_controls_allowed) {
      gm_desired_torque_last = 0;
      gm_rt_torque_last = 0;
      gm_ts_last = ts;
    }

    if (violation) {
      //Replace payload with appropriate zero value for supplied rolling counter
      to_send->RDHR = vals[rolling_counter];
      //tx = 0;
    }

    if (tx != 0) {
      gm_lkas_counter_prev = rolling_counter;
    }

  }

  // PARK ASSIST STEER: unlimited torque, no thanks
  if (addr == 823) {
    tx = 0;
  }

  // GAS/REGEN: safety check
  if (addr == 715) {
    int gas_regen = ((GET_BYTE(to_send, 2) & 0x7FU) << 5) + ((GET_BYTE(to_send, 3) & 0xF8U) >> 3);
    // Disabled message is !engaged with gas
    // value that corresponds to max regen.
    if (!current_controls_allowed || !long_controls_allowed) {
      bool apply = GET_BYTE(to_send, 0) & 1U;
      if (apply || (gas_regen != GM_MAX_REGEN)) {
        tx = 0;
      }
    }
    if (gas_regen > GM_MAX_GAS) {
      tx = 0;
    }
  }

  // 1 allows the message through
  return tx;
}

static void gm_init(int16_t param) {
  UNUSED(param);
  controls_allowed = 0;
  gm_ignition_started = 0;
}

static int gm_ign_hook(void) {
  return gm_ignition_started;
}

// All sending is disallowed.
// The only difference from "no output" model
// is using GM ignition hook.

static void gm_passive_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  int bus_number = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  if ((addr == 0x1F1) && (bus_number == 0)) {
    bool ign = (GET_BYTE(to_push, 0) & 0x20) != 0;
    gm_ignition_started = ign;
  }
}

static int gm_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {

  int bus_fwd = -1;
  if (bus_num == 0) {
    bus_fwd = 2;  // Camera CAN
  }
  if (bus_num == 2) {
    int addr = GET_ADDR(to_fwd);

    if (addr != 384) return 0;
    if (!gm_stock_lkas)  return -1;

    puts("stock lkas active");

    int lkas_counter = (GET_BYTE(to_fwd, 0) & 0x3U) >> 4;
    int required_counter = (gm_lkas_counter_prev + 1) % 4;

    char buffer[50];
    sprintf(buffer, "lkas_counter: %d, required_counter: %d",lkas_counter,required_counter  );
    puts(buffer);

    //TODO: find out if this even happens...
    if (lkas_counter == required_counter) {
      bus_fwd = 0;
      gm_lkas_counter_prev = lkas_counter;
    }
  }

  // fallback to do not forward
  return bus_fwd;
}




const safety_hooks gm_hooks = {
  .init = gm_init,
  .rx = gm_rx_hook,
  .tx = gm_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .ignition = gm_ign_hook,
  .fwd = gm_fwd_hook,
};

const safety_hooks gm_passive_hooks = {
  .init = gm_init,
  .rx = gm_passive_rx_hook,
  .tx = nooutput_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .ignition = gm_ign_hook,
  .fwd = default_fwd_hook,
};
