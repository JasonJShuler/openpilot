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

int gm_brake_prev = 0;
int gm_gas_prev = 0;
bool gm_moving = false;
// silence everything if stock car control ECUs are still online
bool gm_ascm_detected = 0;
int gm_rt_torque_last = 0;
int gm_desired_torque_last = 0;
uint32_t gm_ts_last = 0;
struct sample_t gm_torque_driver;         // last few driver torques measured



//bool lkas_pump_enabled = false;
//bool use_stock_lkas = true;
CAN_FIFOMailBox_TypeDef * stock_lkas;
bool have_stock_lkas = false;
CAN_FIFOMailBox_TypeDef * op_lkas;
bool have_op_lkas = false;
int lkas_rolling_counter = 0;

// //TODO: make the frequency / interval adjustable
// //TODO: can we change the frequency on the fly?
// static void ENABLE_LKAS_PUMP(void) {
//   //Setup LKAS 20ms timer
//   timer_init(TIM5, 15);
//   NVIC_EnableIRQ(TIM5_IRQn);
//   lkas_pump_enabled = true;
// }

// public void DISABLE_LKAS_PUMP() {
//   lkas_pump_enabled = false;
// }


static void SET_STOCK_LKAS(CAN_FIFOMailBox_TypeDef *to_send) {
  stock_lkas = to_send;
  // stock_lkas.RIR = to_send->RIR;
  // stock_lkas.RDTR = to_send->RDTR;
  // stock_lkas.RDLR = to_send->RDLR;
  // stock_lkas.RDHR = to_send->RDHR;
  have_stock_lkas = true;
}

static void SET_OP_LKAS(CAN_FIFOMailBox_TypeDef *to_send) {
  op_lkas = to_send;
  // op_lkas.RIR = to_send->RIR;
  // op_lkas.RDTR = to_send->RDTR;
  // op_lkas.RDLR = to_send->RDLR;
  // op_lkas.RDHR = to_send->RDHR;
  have_op_lkas = true;
}


// static void CALCULATE_LKAS_CHECKSUM(CAN_FIFOMailBox_TypeDef *to_send) {
// /*  values = {
//     "LKASteeringCmdActive": lkas_active,
//     "LKASteeringCmd": apply_steer,
//     "RollingCounter": idx,
//     "LKASteeringCmdChecksum": 0x1000 - (lkas_active << 11) - (apply_steer & 0x7ff) - idx
//   }*/

//   //0x30 00 0f fd U
//   int rolling_counter = GET_BYTE(to_send, 0) >> 4;
//   int lkas_active = (GET_BYTE(to_send, 0) & 8) >> 3;
//   int apply_steer = (GET_BYTE(to_send, 0) & 7 << 8) + GET_BYTE(to_send, 1);
//   int checksum = 0x1000 - (lkas_active << 11) - (apply_steer & 0x7ffU) - rolling_counter;

//   puts("original Value: ");
//   puth(to_send->RDHR);
//   puts("\n");

//   puts("rolling_counter: ");
//   puth(rolling_counter);
//   puts("\n");

//   puts("lkas_active: ");
//   puth(lkas_active);
//   puts("\n");

//   puts("apply_steer: ");
//   puth(apply_steer);
//   puts("\n");

//   puts("checksum: ");
//   puth(checksum);
//   puts("\n");

//   to_send->RDHR = (to_send->RDHR & 0xFFFF0000U) + checksum;

//   puts("New Value: ");
//   puth(to_send->RDHR);
//   puts("\n");

// }








static void gm_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  int bus_number = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  if (addr == 388) {
    int torque_driver_new = ((GET_BYTE(to_push, 6) & 0x7) << 8) | GET_BYTE(to_push, 7);
    torque_driver_new = to_signed(torque_driver_new, 11);
    // update array of samples
    update_sample(&gm_torque_driver, torque_driver_new);
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
    controls_allowed = 0;
  }

  // ACC steering wheel buttons
  if (addr == 481) {
    int button = (GET_BYTE(to_push, 5) & 0x70) >> 4;
    switch (button) {
      case 2:  // resume
      case 3:  // set
        controls_allowed = 1;
        break;
      case 6:  // cancel
        controls_allowed = 0;
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
       controls_allowed = 0;
    }
    gm_brake_prev = brake;
  }

  // exit controls on rising edge of gas press
  if (addr == 417) {
    int gas = GET_BYTE(to_push, 6);
    if (gas && !gm_gas_prev && long_controls_allowed) {
      controls_allowed = 0;
    }
    gm_gas_prev = gas;
  }

  // exit controls on regen paddle
  if (addr == 189) {
    bool regen = GET_BYTE(to_push, 0) & 0x20;
    if (regen) {
      controls_allowed = 0;
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
    uint32_t vals[4];
    vals[0] = 0x00000000U;
    vals[1] = 0x10000fffU;
    vals[2] = 0x20000ffeU;
    vals[3] = 0x30000ffdU;
    int rolling_counter = GET_BYTE(to_send, 0) >> 4;

    int desired_torque = ((GET_BYTE(to_send, 0) & 0x7U) << 8) + GET_BYTE(to_send, 1);
    uint32_t ts = TIM2->CNT;
    bool violation = 0;
    desired_torque = to_signed(desired_torque, 11);

    if (current_controls_allowed) {


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
      //Replace payload with appropriate zero value for expected rolling counter
      to_send->RDLR = vals[rolling_counter];
      //tx = 0;
    }
    tx = 0;
    SET_OP_LKAS(to_send);
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
}


static int gm_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  int bus_fwd = -1;
  if (bus_num == 0) {
    bus_fwd = 2;  // Camera CAN
  }
  if (bus_num == 2) {
    int addr = GET_ADDR(to_fwd);
    if (addr != 384) return 0;
    SET_STOCK_LKAS(to_fwd);
  }

  // fallback to do not forward
  return bus_fwd;
}



//TODO this should check for stalling and fall back to 0
static CAN_FIFOMailBox_TypeDef * gm_lkas_hook(void) {
  puts("gm_lkas_hook\n");
  CAN_FIFOMailBox_TypeDef to_send;

  if (!controls_allowed) {
    if (!have_stock_lkas) return NULL;
    puts("using stock lkas\n");
    to_send.RIR = stock_lkas->RIR;
    to_send.RDTR = stock_lkas->RDTR;
    to_send.RDLR = stock_lkas->RDLR;
    to_send.RDHR = stock_lkas->RDHR;
  } else {
    if (!have_op_lkas) return NULL;
    puts("using OP lkas\n");
    to_send.RIR = op_lkas->RIR;
    to_send.RDTR = op_lkas->RDTR;
    to_send.RDLR = op_lkas->RDLR;
    to_send.RDHR = op_lkas->RDHR;
  }

  puts("preval: ");
  puth(to_send.RDLR);
  puts("\n");

//Thanks Andrew C
  // //this should somehow be controlled in safety code
  lkas_rolling_counter = (lkas_rolling_counter + 1) % 4;

  // Replacement rolling counter 
  uint32_t newidx = lkas_rolling_counter;
  
  // Pull out LKA Steering CMD data and swap endianness (not including rolling counter)
  uint32_t dataswap = ((to_send.RDLR << 8) & 0x0F00U) | ((to_send.RDLR >> 8) &0xFFU);

  // Compute Checksum
  uint32_t checksum = (0x1000 - dataswap - newidx) & 0x0fff;
  
  //Swap endianness of checksum back to what GM expects
  uint32_t checksumswap = (checksum >> 8) | ((checksum << 8) & 0xFF00U);
  
  // Merge the rewritten checksum back into the BxCAN frame RDLR
  to_send.RDLR &= 0x0000FFFF;
  to_send.RDLR |= (checksumswap << 16);



  //int rolling_counter = GET_BYTE(to_send, 0) >> 4;
  puts("postval: ");
  puth(to_send.RDLR);
  puts("\n");

  return &to_send;
  // //0x30000ffdU
  // //update the rolling counter
  //to_send->RDLR = (0x00111111U & to_send->RDLR) + (lkas_rolling_counter << 7);


  // puts("postval: ");
  // puth(to_send->RDLR);
  // puts("\n");

  // CALCULATE_LKAS_CHECKSUM(to_send);
  //return to_send;
}



const safety_hooks gm_hooks = {
  .init = gm_init,
  .rx = gm_rx_hook,
  .tx = gm_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = gm_fwd_hook,
  .lkas = gm_lkas_hook
};
