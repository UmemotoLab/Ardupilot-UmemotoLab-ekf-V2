#define AP_OBSERVER_REPLAY_TEST 1
#include "AP_Observer.h"

// パラメータテーブル定義
const AP_Param::GroupInfo AP_Observer::var_info[] = {
    // @Param: CORR_GAIN
    // @DisplayName: Observer Correction Gain
    // @Description: Gain for attitude correction based on external force estimation
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("CORR_GAIN", 0, AP_Observer, _correction_gain, 0.0f),

    // @Param: EKF_Q_D
    // @DisplayName: EKF Process Noise D
    // @Description: Process noise variance for disturbance state d
    // @Range: 0.0 100.0
    // @User: Advanced
    AP_GROUPINFO("EKF_Q_D", 4, AP_Observer, _ekf_q_d, 9.5367432e-12f),

    // @Param: EKF_Q_DD
    // @DisplayName: EKF Process Noise DDot
    // @Description: Process noise variance for disturbance velocity state d_dot
    // @Range: 0.0 100.0
    // @User: Advanced
    AP_GROUPINFO("EKF_Q_DD", 5, AP_Observer, _ekf_q_d_dot, 2.3841858e-11f),

    // @Param: EKF_Q_C
    // @DisplayName: EKF Process Noise Offset
    // @Description: Process noise variance for DC offset state c
    // @Range: 0.0 100.0
    // @User: Advanced
    AP_GROUPINFO("EKF_Q_C", 6, AP_Observer, _ekf_q_c, 4.7683716e-13f),

    // @Param: EKF_Q_W
    // @DisplayName: EKF Process Noise Omega
    // @Description: Process noise variance for frequency state omega
    // @Range: 0.0 100.0
    // @User: Advanced
    AP_GROUPINFO("EKF_Q_W", 7, AP_Observer, _ekf_q_omega, 0.0005f),

    // @Param: EKF_R_MEAS
    // @DisplayName: EKF Measurement Noise
    // @Description: Measurement noise variance for payload force observations
    // @Range: 0.0001 1000.0
    // @User: Advanced
    AP_GROUPINFO("EKF_R_MEAS", 8, AP_Observer, _ekf_r_meas, 46.0f),

    // @Param: EKF_W_INIT
    // @DisplayName: EKF Initial Omega
    // @Description: Initial angular frequency [rad/s]
    // @Range: 1.0 20.0
    // @User: Advanced
    AP_GROUPINFO("EKF_W_INIT", 9, AP_Observer, _ekf_omega_init, 3.7699f),

    // @Param: EKF_W_MIN
    // @DisplayName: EKF Minimum Omega
    // @Description: Minimum angular frequency [rad/s]
    // @Range: 1.0 20.0
    // @User: Advanced
    AP_GROUPINFO("EKF_W_MIN", 10, AP_Observer, _ekf_omega_min, 2.1991f),

    // @Param: EKF_W_MAX
    // @DisplayName: EKF Maximum Omega
    // @Description: Maximum angular frequency [rad/s]
    // @Range: 1.0 20.0
    // @User: Advanced
    AP_GROUPINFO("EKF_W_MAX", 11, AP_Observer, _ekf_omega_max, 5.7180f),
    
    // @Param: PRED_TIME
    // @DisplayName: Prediction Time
    // @Description: Time ahead for force prediction [seconds]
    // @Range: 0.0 0.5
    // @User: Advanced
    AP_GROUPINFO("PRED_TIME", 13, AP_Observer, _prediction_time, 0.00f),
    
    // @Param: MAX_CORR_ANG
    // @DisplayName: Maximum Correction Angle
    // @Description: Maximum attitude correction angle for roll and pitch [rad]
    // @Range: 0.0 1.0
    // @User: Advanced
    AP_GROUPINFO("MAX_CORR_ANG", 19, AP_Observer, _max_correction_angle, 0.5f),

    // @Param: EKF_INN_MAX
    // @DisplayName: EKF Innovation Maximum
    // @Description: Maximum absolute innovation for including axis in fused frequency update [N]
    // @Range: 0.01 20.0
    // @User: Advanced
    AP_GROUPINFO("EKF_INN_MAX", 24, AP_Observer, _ekf_innov_max, 0.70f),

    // @Param: FADE_TH
    // @DisplayName: Output Fade Amplitude Threshold
    // @Description: Amplitude threshold [N] to reset the fade timer
    // @Range: 0.0 10.0
    // @User: Advanced
    AP_GROUPINFO("FADE_TH", 26, AP_Observer, _out_fade_th, 1.0f),

    // @Param: FADE_DLY
    // @DisplayName: Output Fade Delay
    // @Description: Time [s] below threshold before starting to fade out
    // @Range: 0.0 5.0
    // @User: Advanced
    AP_GROUPINFO("FADE_DLY", 27, AP_Observer, _out_fade_dly, 2.0f),

    // @Param: FADE_IN_T
    // @DisplayName: Output Fade In Time Constant
    // @Description: Time constant [s] when fading in (typically fast)
    // @Range: 0.01 2.0
    // @User: Advanced
    AP_GROUPINFO("FADE_IN_T", 28, AP_Observer, _out_fade_in_t, 0.1f),

    // @Param: FADE_OUT_T
    // @DisplayName: Output Fade Out Time Constant
    // @Description: Time constant [s] when fading out (typically slow)
    // @Range: 0.01 5.0
    // @User: Advanced
    AP_GROUPINFO("FADE_OUT_T", 29, AP_Observer, _out_fade_out_t, 1.0f),

    // @Param: W_FREEZE
    // @DisplayName: EKF Omega Freeze
    // @Description: Freeze omega state to initial value (0=disabled, 1=enabled)
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("W_FREEZE", 30, AP_Observer, _ekf_w_freeze, 0),

    AP_GROUPEND
};

void AP_Observer::init() {
    // パラメータのデフォルト値設定
    AP_Param::setup_object_defaults(this, var_info);

    // フィルタ初期化
    float sample_freq = 100.0f; // サンプリング周波数 [Hz]
    _energy_bandpass_fast.set_cutoff_frequency(sample_freq, 0.80f);
    _energy_bandpass_slow.set_cutoff_frequency(sample_freq, 0.25f);

    // 基本変数初期化
    current_filtered_force = Vector3f();
    current_correction_quat = Quaternion(1, 0, 0, 0);
    last_update_ms = 0;
    _payload_filtered = Vector3f();
    _energy_band_proxy = Vector3f();
    filter_initialized = true;
    _control_enabled = true;

    // EKF初期化
    ekf_init();
    
    // 予測用キャッシュ初期化
    update_prediction_cache();
    
}

void AP_Observer::ekf_init() {
    const float init_cov = EKF_INIT_COVARIANCE;
    const float init_omega = constrain_value(_ekf_omega_init.get(), _ekf_omega_min.get(), _ekf_omega_max.get());

    for (uint8_t axis = 0; axis < EKF_NUM_AXES; axis++) {
        ekf_state[axis][0] = 0.0f;
        ekf_state[axis][1] = 0.0f;
        ekf_state[axis][2] = 0.0f;
        ekf_state[axis][3] = init_omega;
        ekf_axis_innovation[axis] = 0.0f;
        ekf_axis_nis[axis] = 0.0f;
        ekf_axis_amp[axis] = 0.0f;
        ekf_axis_force_abs[axis] = 0.0f;
        ekf_axis_energy_power[axis] = 0.0f;
        ekf_axis_energy_trusted[axis] = 0U;
        ekf_axis_omega_updated[axis] = 0U;
        ekf_axis_hold_omega[axis] = 0U;
        ekf_axis_p00[axis] = 0.0f;
        ekf_axis_p22[axis] = 0.0f;
        ekf_axis_s[axis] = 0.0f;
        ekf_axis_k0[axis] = 0.0f;
        ekf_axis_k2[axis] = 0.0f;
        ekf_axis_dbg_valid[axis] = 0U;

        for (uint8_t i = 0; i < EKF_STATE_SIZE; i++) {
            for (uint8_t j = 0; j < EKF_STATE_SIZE; j++) {
                ekf_P[axis][i][j] = (i == j) ? init_cov : 0.0f;
            }
        }
    }

    ekf_sample_count = 0;
    ekf_start_time_ms = get_current_time_ms();
    ekf_initialized = true;
}

void AP_Observer::reset_frequency_estimation() {
    // 周波数推定のみをリセット（EKF本体は再初期化）
#if HAL_GCS_ENABLED
    gcs().send_text(MAV_SEVERITY_INFO, "AP_Observer: Resetting frequency estimation only");
#endif

    ekf_init();
    
#if HAL_GCS_ENABLED
    gcs().send_text(MAV_SEVERITY_INFO, "AP_Observer: Frequency reset to %.3fHz (EKF)", (double)(_ekf_omega_init.get() / (2.0f * M_PI)));
#endif
}

void AP_Observer::set_control_enabled(bool enabled) {
    _control_enabled = enabled;
}

void AP_Observer::reset_ekf_to_initial_state() {
    const float init_cov = EKF_INIT_COVARIANCE;
    const float init_omega = constrain_value(_ekf_omega_init.get(), _ekf_omega_min.get(), _ekf_omega_max.get());

    for (uint8_t axis = 0; axis < EKF_NUM_AXES; axis++) {
        // Reset all states: d(0), d_dot(1), c(2) to zero, omega(3) to init_omega
        ekf_state[axis][0] = 0.0f;
        ekf_state[axis][1] = 0.0f;
        ekf_state[axis][2] = 0.0f;
        ekf_state[axis][3] = init_omega;

        // Reset covariance matrix to diag(EKF_INIT_COVARIANCE)
        for (uint8_t i = 0; i < EKF_STATE_SIZE; i++) {
            for (uint8_t j = 0; j < EKF_STATE_SIZE; j++) {
                ekf_P[axis][i][j] = (i == j) ? init_cov : 0.0f;
            }
        }

        // Reset per-axis diagnostic variables
        ekf_axis_innovation[axis] = 0.0f;
        ekf_axis_nis[axis] = 0.0f;
        ekf_axis_amp[axis] = 0.0f;
        ekf_axis_force_abs[axis] = 0.0f;
        ekf_axis_energy_power[axis] = 0.0f;
        ekf_axis_energy_trusted[axis] = 0U;
        ekf_axis_omega_updated[axis] = 0U;
        ekf_axis_hold_omega[axis] = 0U;
        ekf_axis_p00[axis] = 0.0f;
        ekf_axis_p22[axis] = 0.0f;
        ekf_axis_s[axis] = 0.0f;
        ekf_axis_k0[axis] = 0.0f;
        ekf_axis_k2[axis] = 0.0f;
        ekf_axis_dbg_valid[axis] = 0U;

        // Reset fade control variables
        _fade_timer[axis] = 0.0f;
        _fade_gain[axis] = 1.0f;
    }

    // Mark reset triggered for logging
    _ekf_reset_triggered = true;

    // Maintain ekf_initialized state (do not change it)

#if HAL_GCS_ENABLED
    gcs().send_text(MAV_SEVERITY_INFO, "AP_Observer: EKF reset to frequency %.3fHz",
                    (double)(init_omega / (2.0f * M_PI)));
#endif
}



void AP_Observer::ekf_update(const Vector3f& y_output, float dt) {
    if (!ekf_initialized) {
#if HAL_GCS_ENABLED
        gcs().send_text(MAV_SEVERITY_WARNING, "EKF: not initialized!");
#endif
        return;
    }

    dt = constrain_value(dt, 0.001f, 0.05f);

    const Vector3f energy_fast = _energy_bandpass_fast.apply(y_output);
    const Vector3f energy_slow = _energy_bandpass_slow.apply(y_output);
    _energy_band_proxy = energy_fast - energy_slow;

    for (uint8_t axis = 0; axis < EKF_NUM_AXES; axis++) {
        if (axis == 2) {
            continue; // Skip Z-axis entirely to save CPU and prevent divergence
        }
        float measurement = 0.0f;
        switch (axis) {
            case 0: measurement = y_output.x; break;
            case 1: measurement = y_output.y; break;
            case 2: measurement = y_output.z; break;
        }
        ekf_update_axis(axis, measurement, dt);
    }

    // 各軸独立推定: 軸間の周波数融合は行わない。
    // 各軸のEKFが独立して周波数を推定し、互いに干渉しない。
    ekf_sample_count++;
}

void AP_Observer::ekf_update_axis(uint8_t axis, float measurement, float dt) {
    float* x = ekf_state[axis];
    float (*P)[EKF_STATE_SIZE] = ekf_P[axis];
    const float force_abs = fabsf(measurement);
    ekf_axis_force_abs[axis] = force_abs;

    const float omega = constrain_value(x[3], _ekf_omega_min.get(), _ekf_omega_max.get());
    const float d = x[0];
    const float d_dot = x[1];
    const float c = x[2];

    float x_pred[EKF_STATE_SIZE];
    x_pred[1] = d_dot + dt * (-(omega * omega) * d);
    x_pred[0] = d + dt * x_pred[1];
    x_pred[2] = c;
    x_pred[3] = omega;
    x_pred[3] = constrain_value(x_pred[3], _ekf_omega_min.get(), _ekf_omega_max.get());

    float F[EKF_STATE_SIZE][EKF_STATE_SIZE] = {
        {1.0f, dt, 0.0f, 0.0f},
        {-dt * omega * omega, 1.0f, 0.0f, -2.0f * dt * omega * d},
        {0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}
    };

    float FP[EKF_STATE_SIZE][EKF_STATE_SIZE];
    for (uint8_t i = 0; i < EKF_STATE_SIZE; i++) {
        for (uint8_t j = 0; j < EKF_STATE_SIZE; j++) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < EKF_STATE_SIZE; k++) {
                sum += F[i][k] * P[k][j];
            }
            FP[i][j] = sum;
        }
    }

    float P_pred[EKF_STATE_SIZE][EKF_STATE_SIZE];
    for (uint8_t i = 0; i < EKF_STATE_SIZE; i++) {
        for (uint8_t j = 0; j < EKF_STATE_SIZE; j++) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < EKF_STATE_SIZE; k++) {
                sum += FP[i][k] * F[j][k];
            }
            P_pred[i][j] = sum;
        }
    }

    P_pred[0][0] += _ekf_q_d.get();
    P_pred[1][1] += _ekf_q_d_dot.get();
    P_pred[2][2] += _ekf_q_c.get();
    P_pred[3][3] += _ekf_q_omega.get();

    ekf_axis_p00[axis] = P_pred[0][0];
    ekf_axis_p22[axis] = P_pred[2][2];
    ekf_axis_s[axis] = 0.0f;
    ekf_axis_k0[axis] = 0.0f;
    ekf_axis_k2[axis] = 0.0f;
    ekf_axis_dbg_valid[axis] = 0U;

    const float y_pred = x_pred[0] + x_pred[2];
    const float innov_raw = measurement - y_pred;

    const float innov_max = MAX(1.0e-3f, _ekf_innov_max.get());
    float innov_used = constrain_value(innov_raw, -innov_max, innov_max);

    float R = MAX(1.0e-6f, _ekf_r_meas.get());
    float PHt[EKF_STATE_SIZE];
    for (uint8_t i = 0; i < EKF_STATE_SIZE; i++) {
        PHt[i] = P_pred[i][0] + P_pred[i][2];
    }

    float S = PHt[0] + PHt[2] + R;
    ekf_axis_s[axis] = S;
    if (!isfinite(S) || fabsf(S) < 1.0e-6f) {
        ekf_axis_innovation[axis] = innov_raw;
        ekf_axis_amp[axis] = fabsf(x[0]);
        return;
    }

    float K[EKF_STATE_SIZE];
    for (uint8_t i = 0; i < EKF_STATE_SIZE; i++) {
        K[i] = PHt[i] / S;
    }

    ekf_axis_k0[axis] = K[0];
    ekf_axis_k2[axis] = K[2];
    ekf_axis_dbg_valid[axis] = 1U;
    ekf_axis_innovation[axis] = innov_raw;

    for (uint8_t i = 0; i < EKF_STATE_SIZE; i++) {
        x[i] = x_pred[i] + K[i] * innov_used;
    }

    // Freeze omega state to initial value if W_FREEZE is enabled
    if (_ekf_w_freeze.get() != 0) {
        x[3] = constrain_value(_ekf_omega_init.get(), _ekf_omega_min.get(), _ekf_omega_max.get());
    }

    float KH[EKF_STATE_SIZE][EKF_STATE_SIZE];
    for (uint8_t i = 0; i < EKF_STATE_SIZE; i++) {
        for (uint8_t j = 0; j < EKF_STATE_SIZE; j++) {
            KH[i][j] = K[i] * ((j == 0 || j == 2) ? 1.0f : 0.0f);
        }
    }

    float I_KH[EKF_STATE_SIZE][EKF_STATE_SIZE];
    for (uint8_t i = 0; i < EKF_STATE_SIZE; i++) {
        for (uint8_t j = 0; j < EKF_STATE_SIZE; j++) {
            I_KH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
        }
    }

    float P_new[EKF_STATE_SIZE][EKF_STATE_SIZE];
    for (uint8_t i = 0; i < EKF_STATE_SIZE; i++) {
        for (uint8_t j = 0; j < EKF_STATE_SIZE; j++) {
            float sum = 0.0f;
            for (uint8_t k = 0; k < EKF_STATE_SIZE; k++) {
                sum += I_KH[i][k] * P_pred[k][j];
            }
            P_new[i][j] = sum;
        }
    }

    for (uint8_t i = 0; i < EKF_STATE_SIZE; i++) {
        for (uint8_t j = 0; j < EKF_STATE_SIZE; j++) {
            P[i][j] = 0.5f * (P_new[i][j] + P_new[j][i]);
        }
        P[i][i] = MAX(P[i][i], 0.0f);
    }

    // フェード処理
    const float new_omega = MAX(1e-3f, x[3]);
    const float term = x[1] / new_omega;
    const float A = sqrtf(x[0]*x[0] + term*term);
    ekf_axis_amp[axis] = A;

    const float fade_th = _out_fade_th.get();
    if (A < fade_th) {
        _fade_timer[axis] += dt;
    } else {
        _fade_timer[axis] = 0.0f;
    }

    const float target_gain = (_fade_timer[axis] > _out_fade_dly.get()) ? 0.0f : 1.0f;
    const float tau = (target_gain > _fade_gain[axis]) ? MAX(1e-3f, _out_fade_in_t.get()) : MAX(1e-3f, _out_fade_out_t.get());
    const float alpha = constrain_value(dt / (dt + tau), 0.0f, 1.0f);
    _fade_gain[axis] += alpha * (target_gain - _fade_gain[axis]);

    ekf_axis_omega_updated[axis] = 1U;
}

void AP_Observer::update_prediction_cache() {
    // 各軸独立推定: X軸の周波数を予測用キャッシュとして使用
    const float omega_x = constrain_value(ekf_state[0][3], _ekf_omega_min.get(), _ekf_omega_max.get());
    _omega_rad = omega_x;
}

void AP_Observer::update() {
    // モータポインタの安全チェック
    AP_Motors* motors = AP::motors();
    if (!motors) {
        return;  // エラーメッセージは出さずに静かに終了
    }

    // スラスト計算
    float throttle = motors->get_throttle_out();
    float thrust = -(THRUST_SCALE * throttle + THRUST_OFFSET) * g;
    
    // ペイロード力計算
    Vector3f payload;
    Vector3f accel = AP::ins().get_accel();
    payload.x = UAV_mass * accel.x;
    payload.y = UAV_mass * accel.y;
    payload.z = UAV_mass * accel.z - thrust;
    
    // フィルタ適用（無効化）
    // _payload_filtered = _payload_filter.apply(payload);
    _payload_filtered = payload; // フィルタなしで生データを使用

    const uint32_t now_ms = get_current_time_ms();
    float dt = 0.01f;
    if (last_update_ms != 0) {
        dt = 0.001f * (float)(now_ms - last_update_ms);
    }

    if (ekf_initialized) {
        ekf_update(_payload_filtered, dt);
    }
    
    update_prediction_cache();

    // EKF予測外力を使用
    current_filtered_force = get_predicted_force();  // Δt秒後の予測外力

    if (!_control_enabled) {
        // When control is disabled, output identity correction (no attitude change)
        current_correction_quat = Quaternion(1, 0, 0, 0);
        current_correction_euler = Vector3f(0, 0, 0);
    } else {
        current_correction_quat = calculate_correction_from_force(current_filtered_force);
        current_correction_euler = calculate_correction_euler_from_force(current_filtered_force);
    }
    last_update_ms = get_current_time_ms();

    // ログをSDカードに記録（毎回記録）
    Write_Observer_Log();
    
}
    
Quaternion AP_Observer::calculate_correction_from_force(const Vector3f& force) const {
    // FORCE_THRESHOLD によるマイルストーン足切りは完全に削除

    float correction_gain = _correction_gain.get();
    float roll  =  force.y * correction_gain * _fade_gain[1] / UAV_mass;
    float pitch = -force.x * correction_gain * _fade_gain[0] / UAV_mass;

    float max_angle = _max_correction_angle.get();
    roll = constrain_value(roll, -max_angle, max_angle);
    pitch = constrain_value(pitch, -max_angle, max_angle);

    Quaternion q;
    q.from_euler(roll, pitch, 0.0f);
    q.normalize();
    return q;
}

// オイラー角形式で補正値を計算（ヨー角は常に0）
Vector3f AP_Observer::calculate_correction_euler_from_force(const Vector3f& force) const {
    // FORCE_THRESHOLD によるマイルストーン足切りは完全に削除

    float correction_gain = _correction_gain.get();
    float roll  =  force.y * correction_gain * _fade_gain[1] / UAV_mass;
    float pitch = -force.x * correction_gain * _fade_gain[0] / UAV_mass;

    float max_angle = _max_correction_angle.get();
    roll = constrain_value(roll, -max_angle, max_angle);
    pitch = constrain_value(pitch, -max_angle, max_angle);

    // ヨー角は常に0.0fに固定
    return Vector3f(roll, pitch, 0.0f);
}

Vector3f AP_Observer::get_harmonic_sin_coeff() const {
    return Vector3f(ekf_state[0][0], ekf_state[1][0], ekf_state[2][0]);
}

Vector3f AP_Observer::get_harmonic_cos_coeff() const {
    return Vector3f(ekf_state[0][1], ekf_state[1][1], ekf_state[2][1]);
}

Vector3f AP_Observer::get_dc_offset() const {
    return Vector3f(ekf_state[0][2], ekf_state[1][2], ekf_state[2][2]);
}

Vector3f AP_Observer::get_predicted_force() const {
    if (!ekf_initialized) {
        return _payload_filtered;  // 初期化前は通常の外力を返す
    }
    const float pred_dt = _prediction_time.get();
    Vector3f predicted;
    for (uint8_t axis = 0; axis < EKF_NUM_AXES; axis++) {
        const float* state = ekf_state[axis];
        const float omega = constrain_value(state[3], _ekf_omega_min.get(), _ekf_omega_max.get());
        const float d = state[0];
        const float d_dot = state[1];
        const float c = state[2];
        const float d_pred = d + pred_dt * d_dot;
        const float d_dot_pred = d_dot + pred_dt * (-(omega * omega) * d);
        const float force = d_pred + c;
        (void)d_dot_pred;

        if (axis == 0) {
            predicted.x = force;
        } else if (axis == 1) {
            predicted.y = force;
        } else {
            predicted.z = force;
        }
    }
    
    return predicted;
}

// ログをSDカードに記録
void AP_Observer::Write_Observer_Log() {
#if HAL_LOGGING_ENABLED
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        return;
    }

    // ログメッセージをカスタムフォーマットで書き込み
    // OBSV: TimeUS, PLX, PLY, PLZ, PFX, PFY, PFZ, FX, FY, CR, CP, SW, CE, ER
    // PFX/PFY/PFZ = predicted force from EKF (replaces D, V, C internal states)
    // FX/FY = per-axis estimated frequency (Hz), no fused frequency field
    // CR/CP = correction Euler Roll/Pitch [rad]
    // CE = control enabled flag, ER = EKF reset triggered flag
    const Vector3f predicted = get_predicted_force();
    logger->Write("OBSV", "TimeUS,PLX,PLY,PLZ,PFX,PFY,PFZ,FX,FY,CR,CP,SW,CE,ER",
                  "s-------------", "F-------------",
                  "QffffffffffBBB",
                  AP_HAL::micros64(),
                  _payload_filtered.x,
                  _payload_filtered.y,
                  _payload_filtered.z,
                  predicted.x,           // PFX: predicted force X
                  predicted.y,           // PFY: predicted force Y
                  predicted.z,           // PFZ: predicted force Z
                  ekf_state[0][3] / (2.0f * M_PI), // FX: X-axis frequency
                  ekf_state[1][3] / (2.0f * M_PI), // FY: Y-axis frequency
                  current_correction_euler.x,       // CR: correction roll [rad]
                  current_correction_euler.y,       // CP: correction pitch [rad]
                  (uint8_t)1,                       // SW: 常にON
                  (uint8_t)_control_enabled,        // CE: control enabled
                  (uint8_t)_ekf_reset_triggered);   // ER: EKF reset triggered

    // Clear reset trigger flag after logging
    _ekf_reset_triggered = false;

    for (uint8_t axis = 0; axis < 2; axis++) {
        logger->Write("OBEK", "TimeUS,AX,P00,P22,SS,K0,K2,DV",
                      "s-------", "F-------",
                      "QBfffffB",
                      AP_HAL::micros64(),
                      axis,
                      ekf_axis_p00[axis],
                      ekf_axis_p22[axis],
                      ekf_axis_s[axis],
                      ekf_axis_k0[axis],
                      ekf_axis_k2[axis],
                      ekf_axis_dbg_valid[axis]);
    }
#endif
}



#ifdef AP_OBSERVER_REPLAY_TEST
void AP_Observer::force_frequency_estimation_update(const Vector3f& payload) {
    _payload_filtered = payload;
    
    if (ekf_initialized) {
        ekf_update(_payload_filtered, 0.01f);
    }

}

void AP_Observer::set_params_for_replay(float freq, float gain) {
    _correction_gain.set(gain);

    // Reinitialize frequency state for replay runs.
    const float omega = constrain_value(freq * 2.0f * float(M_PI), _ekf_omega_min.get(), _ekf_omega_max.get());
    _ekf_omega_init.set(omega);

    // Ensure EKF re-init usage of new params
    update_prediction_cache();
    ekf_init();
}

void AP_Observer::set_ekf_w_init_hz_for_replay(float freq_hz) {
    const float omega = freq_hz * 2.0f * M_PI;
    _ekf_omega_init.set(omega);
    update_prediction_cache();
    ekf_init();
}

void AP_Observer::set_ekf_process_noises_for_replay(float q_d,
                                                    float q_dd,
                                                    float q_c) {
    _ekf_q_d.set(MAX(0.0f, q_d));
    _ekf_q_d_dot.set(MAX(0.0f, q_dd));
    _ekf_q_c.set(MAX(0.0f, q_c));
}

void AP_Observer::set_ekf_q_w_for_replay(float q_w) {
    _ekf_q_omega.set(q_w);
}

void AP_Observer::set_ekf_r_meas_for_replay(float r_meas) {
    _ekf_r_meas.set(r_meas);
}

void AP_Observer::set_prediction_time_for_replay(float pred_time_sec) {
    _prediction_time.set(constrain_value(pred_time_sec, 0.0f, 0.5f));
}

void AP_Observer::set_ekf_innovation_limits_for_replay(float innov_max) {
    _ekf_innov_max.set(MAX(1.0e-3f, innov_max));
}






#endif
