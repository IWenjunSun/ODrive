
#include "odrive_main.h"
#include "as5047p.h"

uint16_t cal_table[]={0, 9, 22, 32, 41, 48, 62, 68, 81, 88, 95, 108, 114, 126, 136, 143, 152, 163, 171, 186, 189, 201, 210, 220, 227, 238, 247, 257, 266, 272, 284, 292, 302, 312, 322, 333, 337, 350, 359, 372, 379, 390, 398, 405, 418, 426, 434, 441, 455, 465, 475, 481, 493, 503, 511, 522, 529, 540, 553, 561, 573, 585, 592, 602, 615, 625, 631, 643, 653, 663, 671, 680, 692, 698, 713, 720, 731, 742, 749, 764, 773, 778, 790, 799, 810, 820, 828, 840, 847, 860, 868, 877, 886, 897, 904, 915, 928, 936, 946, 953, 965, 969, 982, 992, 1002, 1013, 1021, 1032, 1039, 1049, 1060, 1070, 1078, 1087, 1096, 1105, 1118, 1126, 1137, 1145, 1158, 1167, 1175, 1185, 1193, 1204, 1215, 1224, 1232, 1244, 1253, 1262, 1273, 1284, 1296, 1307, 1319, 1329, 1338, 1350, 1361, 1372, 1384, 1393, 1407, 1419, 1434, 1448, 1460, 1476, 1492, 1504, 1515, 1529, 1539, 1555, 1571, 1585, 1598, 1613, 1623, 1638, 1652, 1667, 1681, 1694, 1709, 1723, 1735, 1750, 1765, 1779, 1795, 1809, 1829, 1845, 1859, 1877, 1888, 1903, 1923, 1939, 1951, 1966, 1986, 2001, 2020, 2034, 2051, 2068, 2082, 2100, 2115, 2132, 2151, 2169, 2186, 2205, 2220, 2242, 2260, 2278, 2298, 2313, 2333, 2350, 2370, 2389, 2408, 2429, 2450, 2470, 2492, 2512, 2532, 2550, 2571, 2593, 2617, 2635, 2658, 2675, 2700, 2724, 2745, 2765, 2788, 2804, 2825, 2843, 2867, 2890, 2914, 2936, 2959, 2985, 3008, 3030, 3056, 3079, 3102, 3127, 3148, 3173, 3198, 3223, 3250, 3273, 3298, 3323, 3352, 3375, 3399, 3426, 3448, 3477, 3501, 3526, 3552, 3575, 3602, 3630, 3662, 3688, 3715, 3737, 3767, 3792, 3822, 3848, 3872, 3901, 3927, 3954, 3986, 4010, 4037, 4061, 4087, 4117, 4145, 4172, 4198, 4227, 4254, 4282, 4306, 4338, 4363, 4393, 4418, 4444, 4467, 4495, 4528, 4550, 4575, 4603, 4631, 4658, 4686, 4710, 4738, 4766, 4789, 4814, 4843, 4869, 4895, 4921, 4949, 4975, 5004, 5028, 5052, 5074, 5100, 5124, 5154, 5180, 5211, 5231, 5263, 5288, 5311, 5337, 5362, 5388, 5411, 5433, 5459, 5481, 5503, 5527, 5554, 5577, 5600, 5622, 5645, 5668, 5690, 5710, 5727, 5747, 5772, 5792, 5814, 5839, 5856, 5875, 5894, 5914, 5935, 5955, 5972, 5991, 6013, 6026, 6048, 6066, 6087, 6105, 6119, 6140, 6153, 6175, 6189, 6210, 6223, 6242, 6254, 6270, 6286, 6305, 6324, 6336, 6354, 6370, 6381, 6395, 6413, 6428, 6446, 6464, 6476, 6494, 6512, 6523, 6540, 6549, 6569, 6584, 6598, 6614, 6628, 6642, 6659, 6676, 6695, 6707, 6723, 6736, 6753, 6767, 6780, 6795, 6806, 6822, 6838, 6852, 6870, 6883, 6901, 6914, 6928, 6940, 6955, 6972, 6985, 6999, 7014, 7030, 7045, 7059, 7069, 7085, 7099, 7107, 7125, 7139, 7157, 7167, 7182, 7194, 7206, 7222, 7235, 7251, 7262, 7274, 7290, 7302, 7315, 7327, 7335, 7353, 7363, 7377, 7392, 7403, 7416, 7430, 7440, 7450, 7462, 7474, 7487, 7498, 7510, 7521, 7533, 7545, 7557, 7568, 7576, 7587, 7598, 7610, 7617, 7630, 7642, 7650, 7664, 7674, 7690, 7697, 7710, 7720, 7731, 7741, 7750, 7759, 7769, 7780, 7787, 7801, 7810, 7820, 7832, 7842, 7850, 7861, 7871, 7879, 7889, 7898, 7908, 7918, 7927, 7939, 7950, 7959, 7968, 7981, 7988, 7996, 8009, 8020, 8030, 8042, 8051, 8063, 8071, 8083, 8094, 8105, 8112, 8121, 8132, 8145, 8150, 8167, 8176, 8185, 8193, 8203, 8213, 8221, 8230, 8240, 8252, 8264, 8275, 8281, 8293, 8303, 8317, 8323, 8333, 8344, 8355, 8366, 8378, 8390, 8400, 8412, 8419, 8430, 8441, 8452, 8460, 8474, 8483, 8492, 8505, 8514, 8524, 8533, 8544, 8555, 8563, 8575, 8586, 8597, 8608, 8617, 8628, 8631, 8645, 8657, 8667, 8678, 8689, 8700, 8714, 8723, 8733, 8742, 8755, 8768, 8779, 8790, 8802, 8812, 8826, 8837, 8847, 8861, 8871, 8882, 8893, 8905, 8917, 8926, 8937, 8947, 8959, 8971, 8978, 8991, 9007, 9014, 9027, 9037, 9047, 9061, 9071, 9080, 9094, 9103, 9110, 9125, 9133, 9142, 9152, 9161, 9172, 9180, 9192, 9200, 9209, 9219, 9229, 9239, 9250, 9257, 9269, 9280, 9290, 9296, 9308, 9317, 9326, 9338, 9347, 9361, 9367, 9378, 9386, 9395, 9402, 9415, 9427, 9433, 9444, 9454, 9463, 9471, 9483, 9492, 9501, 9512, 9520, 9534, 9539, 9550, 9558, 9570, 9580, 9588, 9601, 9610, 9625, 9633, 9644, 9651, 9660, 9673, 9681, 9693, 9704, 9715, 9728, 9740, 9745, 9761, 9772, 9781, 9796, 9803, 9818, 9830, 9840, 9849, 9864, 9875, 9890, 9904, 9916, 9924, 9936, 9949, 9964, 9973, 9989, 10002, 10014, 10029, 10043, 10058, 10073, 10085, 10099, 10113, 10127, 10143, 10156, 10174, 10190, 10206, 10220, 10235, 10251, 10266, 10279, 10298, 10312, 10331, 10346, 10361, 10381, 10393, 10412, 10428, 10447, 10461, 10478, 10494, 10514, 10531, 10550, 10567, 10590, 10606, 10627, 10645, 10665, 10684, 10704, 10724, 10743, 10763, 10782, 10805, 10825, 10846, 10867, 10889, 10907, 10930, 10951, 10974, 10995, 11018, 11042, 11064, 11088, 11114, 11138, 11162, 11185, 11208, 11229, 11253, 11277, 11300, 11326, 11349, 11375, 11408, 11436, 11461, 11490, 11514, 11540, 11565, 11591, 11617, 11646, 11673, 11707, 11733, 11768, 11796, 11819, 11847, 11880, 11905, 11936, 11962, 11998, 12024, 12054, 12082, 12106, 12136, 12169, 12198, 12233, 12266, 12292, 12323, 12358, 12388, 12421, 12453, 12487, 12518, 12554, 12587, 12621, 12652, 12684, 12715, 12748, 12783, 12812, 12846, 12876, 12907, 12944, 12977, 13009, 13040, 13068, 13101, 13127, 13154, 13186, 13217, 13247, 13280, 13307, 13338, 13369, 13397, 13424, 13453, 13480, 13508, 13538, 13571, 13602, 13634, 13660, 13689, 13718, 13745, 13774, 13798, 13824, 13845, 13875, 13901, 13927, 13955, 13981, 14005, 14030, 14055, 14080, 14101, 14129, 14147, 14163, 14184, 14207, 14232, 14255, 14279, 14304, 14327, 14349, 14367, 14389, 14413, 14425, 14449, 14469, 14492, 14514, 14531, 14552, 14569, 14590, 14612, 14626, 14644, 14661, 14677, 14692, 14714, 14731, 14745, 14763, 14780, 14797, 14812, 14828, 14841, 14858, 14873, 14887, 14903, 14913, 14929, 14944, 14959, 14973, 14986, 14998, 15012, 15024, 15037, 15048, 15064, 15078, 15094, 15106, 15119, 15134, 15142, 15153, 15167, 15178, 15189, 15204, 15214, 15228, 15240, 15252, 15263, 15275, 15281, 15301, 15309, 15316, 15328, 15343, 15350, 15362, 15368, 15384, 15393, 15404, 15414, 15422, 15431, 15441, 15451, 15464, 15473, 15479, 15488, 15503, 15511, 15522, 15531, 15540, 15546, 15561, 15567, 15575, 15589, 15594, 15603, 15615, 15623, 15633, 15641, 15652, 15660, 15670, 15676, 15688, 15701, 15707, 15720, 15730, 15739, 15747, 15758, 15768, 15774, 15787, 15791, 15804, 15812, 15824, 15832, 15847, 15854, 15868, 15878, 15889, 15900, 15907, 15921, 15929, 15938, 15951, 15964, 15970, 15982, 15992, 16001, 16018, 16029, 16036, 16047, 16055, 16068, 16077, 16089, 16097, 16112, 16123, 16132, 16145, 16154, 16164, 16175, 16185, 16196, 16201, 16216, 16225, 16236, 16249, 16260, 16267, 16275, 16292, 16298, 16313, 16318, 16328};


Encoder::Encoder(const EncoderHardwareConfig_t& hw_config,
                Config_t& config) :
        hw_config_(hw_config),
        config_(config),
        AS5047PEncoder({
             .spiHandle = hw_config_.spi, //&hspi3,
             .nCSgpioHandle = hw_config_.nCS_port,//GPIO_1_GPIO_Port,
             .nCSgpioNumber = hw_config_.nCS_pin,// GPIO_1_Pin,
             .encoder_angle = 0.0f,
         })
{
    if (config.pre_calibrated && (config.mode == Encoder::MODE_HALL)) {
        offset_ = config.offset;
        is_ready_ = true;
    }
    if (config.pre_calibrated && (config.mode == Encoder::MODE_SPI)) {
         offset_ = config.offset;
         is_ready_ = true;
     }
}

static void enc_index_cb_wrapper(void* ctx) {
    reinterpret_cast<Encoder*>(ctx)->enc_index_cb();
}

void Encoder::setup() {
    HAL_TIM_Encoder_Start(hw_config_.timer, TIM_CHANNEL_ALL);
    GPIO_subscribe(hw_config_.index_port, hw_config_.index_pin, GPIO_NOPULL,
            enc_index_cb_wrapper, this);
}

void Encoder::set_error(Encoder::Error_t error) {
    error_ |= error;
    axis_->error_ |= Axis::ERROR_ENCODER_FAILED;
}

bool Encoder::do_checks(){
    return error_ == ERROR_NONE;
}

//--------------------
// Hardware Dependent
//--------------------

// Triggered when an encoder passes over the "Index" pin
// TODO: only arm index edge interrupt when we know encoder has powered up
// TODO: disable interrupt once we found the index
void Encoder::enc_index_cb() {
    if (config_.use_index && !index_found_) {
        set_circular_count(0);
        if (config_.pre_calibrated) {
            offset_ = config_.offset;
            is_ready_ = true;
        }
        index_found_ = true;
    }
}

// Function that sets the current encoder count to a desired 32-bit value.
void Encoder::set_linear_count(int32_t count) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = __get_PRIMASK();
    __disable_irq();

    // Update states
    shadow_count_ = count;
    pos_estimate_ = (float)count;
    //Write hardware last
    hw_config_.timer->Instance->CNT = count;

    __set_PRIMASK(prim);
}

// Function that sets the CPR circular tracking encoder count to a desired 32-bit value.
// Note that this will get mod'ed down to [0, cpr)
void Encoder::set_circular_count(int32_t count) {
    // Disable interrupts to make a critical section to avoid race condition
    uint32_t prim = __get_PRIMASK();
    __disable_irq();

    // Offset and state must be shifted by the same amount
    offset_ += count - count_in_cpr_;
    offset_ = mod(offset_, config_.cpr);
    // Update states
    count_in_cpr_ = mod(count, config_.cpr);
    pos_cpr_ = (float)count_in_cpr_;

    __set_PRIMASK(prim);
}


// @brief Slowly turns the motor in one direction until the
// encoder index is found.
// TODO: Do the scan with current, not voltage!
bool Encoder::run_index_search() {
    float voltage_magnitude;
    if (axis_->motor_.config_.motor_type == MOTOR_TYPE_HIGH_CURRENT)
        voltage_magnitude = axis_->motor_.config_.calibration_current * axis_->motor_.config_.phase_resistance;
    else if (axis_->motor_.config_.motor_type == MOTOR_TYPE_GIMBAL)
        voltage_magnitude = axis_->motor_.config_.calibration_current;
    else
        return false;

    float omega = (float)(axis_->motor_.config_.direction) * config_.idx_search_speed;

    index_found_ = false;
    float phase = 0.0f;
    axis_->run_control_loop([&](){
        phase = wrap_pm_pi(phase + omega * current_meas_period);

        float v_alpha = voltage_magnitude * arm_cos_f32(phase);
        float v_beta = voltage_magnitude * arm_sin_f32(phase);
        if (!axis_->motor_.enqueue_voltage_timings(v_alpha, v_beta))
            return false; // error set inside enqueue_voltage_timings
        axis_->motor_.log_timing(Motor::TIMING_LOG_IDX_SEARCH);

        // continue until the index is found
        return !index_found_;
    });
    return true;
}

// @brief Turns the motor in one direction for a bit and then in the other
// direction in order to find the offset between the electrical phase 0
// and the encoder state 0.
// TODO: Do the scan with current, not voltage!
bool Encoder::run_offset_calibration() {
    static const float start_lock_duration = 1.0f;
    static const float scan_omega = 4.0f * M_PI;
    static const float scan_distance = 16.0f * M_PI;
    // static const float scan_distance = axis_->motor_.config_.pole_pairs * 2 * M_PI;
    static const int num_steps = (int)(scan_distance / scan_omega * (float)current_meas_hz);

    // Temporarily disable index search so it doesn't mess
    // with the offset calibration
    bool old_use_index = config_.use_index;
    config_.use_index = false;

    // We use shadow_count_ to do the calibration, but the offset is used by count_in_cpr_
    // Therefore we have to sync them for calibration
    shadow_count_ = count_in_cpr_;

    float voltage_magnitude;
    if (axis_->motor_.config_.motor_type == MOTOR_TYPE_HIGH_CURRENT)
        voltage_magnitude = axis_->motor_.config_.calibration_current * axis_->motor_.config_.phase_resistance;
    else if (axis_->motor_.config_.motor_type == MOTOR_TYPE_GIMBAL)
        voltage_magnitude = axis_->motor_.config_.calibration_current;
    else
        return false;

    // go to motor zero phase for start_lock_duration to get ready to scan
    int i = 0;
    axis_->run_control_loop([&](){
        if (!axis_->motor_.enqueue_voltage_timings(voltage_magnitude, 0.0f))
            return false; // error set inside enqueue_voltage_timings
        axis_->motor_.log_timing(Motor::TIMING_LOG_ENC_CALIB);
        return ++i < start_lock_duration * current_meas_hz;
    });
    if (axis_->error_ != Axis::ERROR_NONE)
        return false;

    int32_t init_enc_val = shadow_count_;
    int64_t encvaluesum = 0;

    // scan forward
    i = 0;
    axis_->run_control_loop([&](){
        float phase = wrap_pm_pi(scan_distance * (float)i / (float)num_steps - scan_distance / 2.0f);
        float v_alpha = voltage_magnitude * arm_cos_f32(phase);
        float v_beta = voltage_magnitude * arm_sin_f32(phase);
        if (!axis_->motor_.enqueue_voltage_timings(v_alpha, v_beta))
            return false; // error set inside enqueue_voltage_timings
        axis_->motor_.log_timing(Motor::TIMING_LOG_ENC_CALIB);

        encvaluesum += shadow_count_;

        return ++i < num_steps;
    });
    if (axis_->error_ != Axis::ERROR_NONE)
        return false;

    //TODO avoid recomputing elec_rad_per_enc every time
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float expected_encoder_delta = scan_distance / elec_rad_per_enc;
    float actual_encoder_delta_abs = fabsf(shadow_count_-init_enc_val);
    if(fabsf(actual_encoder_delta_abs - expected_encoder_delta)/expected_encoder_delta > config_.calib_range)
    {
        set_error(ERROR_CPR_OUT_OF_RANGE);
        return false;
    }
    // check direction
    if (shadow_count_ > init_enc_val + 8) {
        // motor same dir as encoder
        axis_->motor_.config_.direction = 1;
    } else if (shadow_count_ < init_enc_val - 8) {
        // motor opposite dir as encoder
        axis_->motor_.config_.direction = -1;
    } else {
        // Encoder response error
        set_error(ERROR_RESPONSE);
        return false;
    }

    // scan backwards
    i = 0;
    axis_->run_control_loop([&](){
        float phase = wrap_pm_pi(-scan_distance * (float)i / (float)num_steps + scan_distance / 2.0f);
        float v_alpha = voltage_magnitude * arm_cos_f32(phase);
        float v_beta = voltage_magnitude * arm_sin_f32(phase);
        if (!axis_->motor_.enqueue_voltage_timings(v_alpha, v_beta))
            return false; // error set inside enqueue_voltage_timings
        axis_->motor_.log_timing(Motor::TIMING_LOG_ENC_CALIB);

        encvaluesum += shadow_count_;

        return ++i < num_steps;
    });
    if (axis_->error_ != Axis::ERROR_NONE)
        return false;

    offset_ = encvaluesum / (num_steps * 2);
    config_.offset = offset_;
    int32_t residual = encvaluesum - ((int64_t)offset_ * (int64_t)(num_steps * 2));
    config_.offset_float = (float)residual / (float)(num_steps * 2) + 0.5f; // add 0.5 to center-align state to phase
    is_ready_ = true;
    config_.use_index = old_use_index;
    return true;
}

uint16_t decode(uint16_t in)
{

    int i=0;
    while(in>cal_table[i])
    {
        i=i+10;
        if(i>1036)
        {
            i=1036;
            break;
        }

    }
    while(in<cal_table[i])
        i--;
    if(cal_table[i]==16328)
        return 16328;
    uint16_t a1=cal_table[i];
    uint16_t a2=cal_table[i+1];

    double percent=(float)(in-a1)/(a2-a1);
    return (i+percent)*14*1.125534;

}

static bool decode_hall(uint8_t hall_state, int32_t* hall_cnt) {
    switch (hall_state) {
        case 0b001: *hall_cnt = 0; return true;
        case 0b011: *hall_cnt = 1; return true;
        case 0b010: *hall_cnt = 2; return true;
        case 0b110: *hall_cnt = 3; return true;
        case 0b100: *hall_cnt = 4; return true;
        case 0b101: *hall_cnt = 5; return true;
        default: return false;
    }
}

bool Encoder::update() {
    // Calculate encoder pll gains
    float pll_kp = 2.0f * config_.bandwidth;  // basic conversion to discrete time
    float pll_ki = 0.25f * (pll_kp * pll_kp); // Critically damped

    // Check that we don't get problems with discrete time approximation
    if (!(current_meas_period * pll_kp < 1.0f)) {
        set_error(ERROR_UNSTABLE_GAIN);
        return false;
    }

    // update internal encoder state.
    int32_t delta_enc = 0;
    switch (config_.mode) {
        case MODE_INCREMENTAL: {
            //TODO: use count_in_cpr_ instead as shadow_count_ can overflow
            //or use 64 bit
            int16_t delta_enc_16 = (int16_t)hw_config_.timer->Instance->CNT - (int16_t)shadow_count_;
            delta_enc = (int32_t)delta_enc_16; //sign extend

            shadow_count_ += delta_enc;
            count_in_cpr_ += delta_enc;
            count_in_cpr_ = mod(count_in_cpr_, config_.cpr);
        } break;

        case MODE_HALL: {
            int32_t hall_cnt;
            if (decode_hall(hall_state_, &hall_cnt)) {
                delta_enc = hall_cnt - count_in_cpr_;
                delta_enc = mod(delta_enc, 6);
                if (delta_enc > 3)
                    delta_enc -= 6;
            } else {
                set_error(ERROR_ILLEGAL_HALL_STATE);
                return false;
            }

            shadow_count_ += delta_enc;
            count_in_cpr_ += delta_enc;
            count_in_cpr_ = mod(count_in_cpr_, config_.cpr);
        } break;

        case MODE_ANALOG: {
          // float a = get_adc_voltage(GPIO_3_GPIO_Port, GPIO_3_Pin);
          // float b = get_adc_voltage(GPIO_4_GPIO_Port, GPIO_4_Pin);

          // if(a>max_t1)
          //   max_t1=a;
          // if(a<min_t1)
          //   min_t1=a;
          // if(b>max_t2)
          //   max_t2=b;
          // if(b<min_t2)
          //   min_t2=b;

          float x = float(get_adc_voltage(GPIO_1_GPIO_Port, GPIO_1_Pin)-shift_1)/range_1; //do not change to double ??? or usb doesn't connect
          float y = float(get_adc_voltage(GPIO_2_GPIO_Port, GPIO_2_Pin)-shift_2)/range_2;
          int count_per_elec_turn = config_.cpr / axis_->motor_.config_.pole_pairs;
          int count = (int)(count_per_elec_turn * atan2(x,y) * (1.0f/6.28318530718f));
          delta_enc = count - count_in_cpr_;
          delta_enc = mod(delta_enc, count_per_elec_turn);
          if (delta_enc > count_per_elec_turn/2)
              delta_enc -= count_per_elec_turn;
          shadow_count_ += delta_enc;
          count_in_cpr_ += delta_enc;
          count_in_cpr_ = mod(count_in_cpr_, config_.cpr);
        } break;

        case MODE_SPI: {
          uint16_t as5047p_data = AS5047P_readPosition(&AS5047PEncoder);
          // osDelay(100);
          as5047p_data = as5047p_data & 0x3FFF;
          AS5047PEncoder.encoder_angle = (as5047p_data/16383.0)*360;
          // AS5047PEncoder.encoder_cnt = (as5047p_data) * 4000/16383; //Old update when count was out of 4000
          AS5047PEncoder.encoder_cnt = (decode(as5047p_data));

          count_in_cpr_ = AS5047PEncoder.encoder_cnt;
          count_in_cpr_ = mod(count_in_cpr_, config_.cpr);

          if(shadow_flag_==0)
          {
            shadow_count_ = config_.cpr*shadow_loop_counter_ + count_in_cpr_;
            shadow_count_prev_ = shadow_count_;
            shadow_flag_ = 1;
            break;
          }
          // Need this code for shadow_count_ because raw absolute encoder loops around.

          if(((config_.cpr*shadow_loop_counter_ + count_in_cpr_)-shadow_count_prev_)<(-config_.cpr/2)){ //dEncdt>0
            shadow_loop_counter_++;
          }else if(((config_.cpr*shadow_loop_counter_ + count_in_cpr_)-shadow_count_prev_)>(config_.cpr/2)){ //dEncdt<0
            shadow_loop_counter_--;
          }

          shadow_count_ = config_.cpr*shadow_loop_counter_ + count_in_cpr_;
          shadow_count_prev_ = shadow_count_;

        } break;

        default: {
           set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
           return false;
        } break;
    }

    //// run pll (for now pll is in units of encoder counts)
    // Predict current pos
    pos_estimate_ += current_meas_period * pll_vel_;
    pos_cpr_      += current_meas_period * pll_vel_;
    // discrete phase detector
    float delta_pos     = (float)(shadow_count_ - (int32_t)floorf(pos_estimate_));
    float delta_pos_cpr = (float)(count_in_cpr_ - (int32_t)floorf(pos_cpr_));
    delta_pos_cpr = wrap_pm(delta_pos_cpr, 0.5f * (float)(config_.cpr));
    // pll feedback
    pos_estimate_ += current_meas_period * pll_kp * delta_pos;
    pos_cpr_      += current_meas_period * pll_kp * delta_pos_cpr;
    pos_cpr_ = fmodf_pos(pos_cpr_, (float)(config_.cpr));
    pll_vel_      += current_meas_period * pll_ki * delta_pos_cpr;
    bool snap_to_zero_vel = false;
    if (fabsf(pll_vel_) < 0.5f * current_meas_period * pll_ki) {
        pll_vel_ = 0.0f; //align delta-sigma on zero to prevent jitter
        snap_to_zero_vel = true;
    }

    //// run encoder count interpolation
    int32_t corrected_enc = count_in_cpr_ - offset_;
    // if we are stopped, make sure we don't randomly drift
    if (snap_to_zero_vel) {
        interpolation_ = 0.5f;
    // reset interpolation if encoder edge comes
    } else if (delta_enc > 0) {
        interpolation_ = 0.0f;
    } else if (delta_enc < 0) {
        interpolation_ = 1.0f;
    } else {
        // Interpolate (predict) between encoder counts using pll_vel,
        interpolation_ += current_meas_period * pll_vel_;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        if (interpolation_ > 1.0f) interpolation_ = 1.0f;
        if (interpolation_ < 0.0f) interpolation_ = 0.0f;
    }
    float interpolated_enc = corrected_enc + interpolation_;

    //// compute electrical phase
    //TODO avoid recomputing elec_rad_per_enc every time
    float elec_rad_per_enc = axis_->motor_.config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float ph = elec_rad_per_enc * (interpolated_enc - config_.offset_float);
    // ph = fmodf(ph, 2*M_PI);
    phase_ = wrap_pm_pi(ph);

    return true;
}
