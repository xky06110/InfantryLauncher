#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args: []
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "CMD.hpp"
#ifdef DEBUG
#include "DebugCore.hpp"
#include "ramfs.hpp"
#endif
#include "Motor.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "pid.hpp"
#include "timebase.hpp"

namespace launcher::param {
constexpr float TRIG_STEP = static_cast<float>(M_2PI) / 10.0f;
constexpr float JAM_TORQUE = 0.1f;
constexpr float FRIC_DROP_RPM = 50.0f;
constexpr float JAM_TOGGLE_INTERVAL_SEC = 0.02f;
constexpr float LONG_PRESS_THRESHOLD_SEC = 0.5f;
constexpr float HEAT_TICK_SEC = 0.05f;
}  // namespace launcher::param

/**
 * @brief 步兵发射机构实现
 * @details 负责摩擦轮、拨弹盘控制与热量约束发射逻辑。
 *          作为 Launcher<InfantryLauncher> 的内部逻辑类，不拥有线程和事件注册。
 */
class InfantryLauncher {
 public:
  enum class LauncherState : uint8_t {
    RELAX,
    STOP,
    NORMAL,
    JAMMED,
  };

  enum class LauncherEvent : uint8_t {
    SET_FRICMODE_RELAX,
    SET_FRICMODE_SAFE,
    SET_FRICMODE_READY,
  };

  enum class TrigMode : uint8_t {
    RELAX,
    SAFE,
    SINGLE,
    CONTINUE,
    JAM,
  };

  struct RefereeData {
    float heat_limit;
    float heat_cooling;
  };

  struct LauncherParam {
    float fric1_setpoint_speed;
    float fric2_setpoint_speed;
    float trig_gear_ratio;
    uint8_t num_trig_tooth;
    float expect_trig_freq_;
  };

  struct HeatLimit {
    float single_heat;
    float launched_num;
    float current_heat;
    float heat_threshold;
    bool allow_fire;
  };

  /**
   * @brief 步兵发射器构造函数
   * @param hw 硬件容器
   * @param app 应用管理器
   * @param motor_fric_front_left 左摩擦轮电机
   * @param motor_fric_front_right 右摩擦轮电机
   * @param motor_fric_back_left 后左摩擦轮电机（当前实现未使用）
   * @param motor_fric_back_right 后右摩擦轮电机（当前实现未使用）
   * @param motor_trig 拨弹电机
   * @param task_stack_depth 控制线程栈深度（由外壳使用）
   * @param pid_param_trig_angle 拨弹角度环参数
   * @param pid_param_trig_speed 拨弹速度环参数
   * @param pid_param_fric_0 摩擦轮0 PID参数
   * @param pid_param_fric_1 摩擦轮1 PID参数
   * @param pid_param_fric_2 预留参数（当前实现未使用）
   * @param pid_param_fric_3 预留参数（当前实现未使用）
   * @param launch_param 发射机构参数
   * @param cmd CMD模块指针
   */
  InfantryLauncher(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                   RMMotor* motor_fric_front_left,
                   RMMotor* motor_fric_front_right,
                   RMMotor* motor_fric_back_left,
                   RMMotor* motor_fric_back_right, RMMotor* motor_trig,
                   uint32_t task_stack_depth,
                   LibXR::PID<float>::Param pid_param_trig_angle,
                   LibXR::PID<float>::Param pid_param_trig_speed,
                   LibXR::PID<float>::Param pid_param_fric_0,
                   LibXR::PID<float>::Param pid_param_fric_1,
                   LibXR::PID<float>::Param pid_param_fric_2,
                   LibXR::PID<float>::Param pid_param_fric_3,
                   LauncherParam launch_param, CMD* cmd)
      : motor_fric_0_(motor_fric_front_left),
        motor_fric_1_(motor_fric_front_right),
        motor_trig_(motor_trig),
        pid_trig_angle_(pid_param_trig_angle),
        pid_trig_sp_(pid_param_trig_speed),
        pid_fric_0_(pid_param_fric_0),
        pid_fric_1_(pid_param_fric_1),
        param_(launch_param) {
    UNUSED(hw);
    UNUSED(app);
    UNUSED(task_stack_depth);
    UNUSED(pid_param_fric_2);
    UNUSED(pid_param_fric_3);
    UNUSED(motor_fric_back_left);
    UNUSED(motor_fric_back_right);
    UNUSED(cmd);
    thread_.Create(this, ThreadFunction, "LauncherThread", task_stack_depth,
                   LibXR::Thread::Priority::HIGH);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, InfantryLauncher* launcher, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          launcher->LostCtrl();
        },
        this);

    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);

    auto launcher_cmd_callback = LibXR::Callback<LibXR::RawData&>::Create(
        [](bool in_isr, InfantryLauncher* Launcher, LibXR::RawData& raw_data) {
          UNUSED(in_isr);
          CMD::LauncherCMD cmd_lau =
              *reinterpret_cast<CMD::LauncherCMD*>(raw_data.addr_);
          Launcher->launcher_cmd_.isfire = cmd_lau.isfire;
        },
        this);

    auto tp_cmd_launcher =
        LibXR::Topic(LibXR::Topic::Find("launcher_cmd", nullptr));

    tp_cmd_launcher.RegisterCallback(launcher_cmd_callback);
  }

  static void ThreadFunction(InfantryLauncher* launcher) {
    LibXR::Topic::ASyncSubscriber<CMD::LauncherCMD> launcher_cmd_tp(
        "launcher_cmd");
    launcher_cmd_tp.StartWaiting();

    while (1) {
      launcher->Update();
      launcher->Heat();
      launcher->ShootTime();
      launcher->Control();
      launcher->shoot_wating_.Publish(launcher->shoot_dt_);
      launcher->shoot_number_.Publish(launcher->number_);
      launcher->shoot_freq_.Publish(launcher->trig_freq_);
      LibXR::Thread::Sleep(2);
    }
  }
  /**
   * @brief 数据处理主入口
   * @details 更新周期时间、电机反馈、拨弹角度，并刷新发射器总状态。
   */
  void Update() {
    last_online_time_ = LibXR::Timebase::GetMicroseconds();

    referee_data_.heat_limit = 260.0f;
    referee_data_.heat_cooling = 20.0f;
    heat_limit_.single_heat = 10.0f;
    heat_limit_.heat_threshold = 2.0f;

    motor_fric_0_->Update();
    motor_fric_1_->Update();
    motor_trig_->Update();

    param_fric_0_ = motor_fric_0_->GetFeedback();
    param_fric_1_ = motor_fric_1_->GetFeedback();
    param_trig_ = motor_trig_->GetFeedback();

    float current_motor_angle = param_trig_.position;
    float delta_trig_angle = LibXR::CycleValue<float>(current_motor_angle) -
                             LibXR::CycleValue<float>(last_motor_angle_);
    trig_angle_ += delta_trig_angle / param_.trig_gear_ratio;
    last_motor_angle_ = current_motor_angle;

    if (fabs(param_trig_.torque) > launcher::param::JAM_TOR) {
      launcherstate_ = LauncherState::JAMMED;
    } else if (launcher_event_ != LauncherEvent::SET_FRICMODE_READY) {
      launcherstate_ = LauncherState::RELAX;
    } else if (!heat_limit_.allow_fire) {
      launcherstate_ = LauncherState::STOP;
    } else {
      launcherstate_ =
          launcher_cmd_.isfire ? LauncherState::NORMAL : LauncherState::STOP;
    }
  }
  void SetFric() {
    switch (launcher_event_) {
      case LauncherEvent::SET_FRICMODE_RELAX: {
        motor_fric_0_->Relax();
        motor_fric_1_->Relax();
      } break;
      case LauncherEvent::SET_FRICMODE_SAFE: {
        target_rpm_ = 0;
      } break;
      case LauncherEvent::SET_FRICMODE_READY: {
        target_rpm_ = param_.fric1_setpoint_speed;
      } break;
      default:
        break;
    }
  }

  void SetTrig() {
    auto now = LibXR::Timebase::GetMilliseconds();
    /*根据状态选择拨弹盘模式*/
    switch (launcherstate_) {
      case LauncherState::RELAX:
        trig_mod_ = TRIGMODE::RELAX;
        break;
      case LauncherState::STOP:
        trig_mod_ = TRIGMODE::SAFE;
        break;
      case LauncherState::NORMAL:
        if (!last_fire_notify_) {
          fire_press_time_ = now;
          press_continue_ = false;
          trig_mod_ = TRIGMODE::SINGLE;
        } else {
          if (!press_continue_ && (now - fire_press_time_ > 500)) {
            press_continue_ = true;
          }
          if (press_continue_) {
            trig_mod_ = TRIGMODE::CONTINUE;
          }
        }

        break;
      case LauncherState::JAMMED:
        trig_mod_ = TRIGMODE::JAM;
        break;

      default:
        break;
    }
    /*不同拨弹模式*/
    switch (trig_mod_) {
      case TRIGMODE::RELAX:
        motor_trig_->Relax();
        shoot_active_ = false;
        break;
      case TRIGMODE::SAFE: {
        target_trig_angle_ = trig_angle_;
        shoot_active_ = false;
      } break;
      case TRIGMODE::SINGLE: {
        if (last_trig_mod_ == TRIGMODE::SAFE ||
            last_trig_mod_ == TRIGMODE::RELAX) {
          target_trig_angle_ = trig_angle_ + launcher::param::TRIGSTEP;
        }
        if (target_trig_angle_ > last_trig_angle_) {
          shoot_active_ = true;
        }
        last_trig_angle_ = target_trig_angle_;
      } break;

      case TRIGMODE::CONTINUE: {
        float since_last = (now - last_trig_time_).ToSecondf();
        if (trig_freq_ > 0.0f) {
          float trig_speed = 1.0f / trig_freq_;
          if (since_last >= trig_speed) {
            target_trig_angle_ = trig_angle_ + launcher::param::TRIGSTEP;
            last_trig_time_ = now;
          }
        }
        if (target_trig_angle_ > last_trig_angle_) {
          shoot_active_ = true;
        }
        last_trig_angle_ = target_trig_angle_;
      } break;
      case TRIGMODE::JAM: {
        /*正转卡弹时反转，反转卡弹时正转*/
        jam_keep_time_ =
            static_cast<uint32_t>((now - last_jam_time_).ToSecondf());
        if (jam_keep_time_ > 0.02) {
          if (last_trig_mod_ != TRIGMODE::JAM) {
            is_reverse_ = 1;
          }
          if (is_reverse_) {
            target_trig_angle_ = trig_angle_ - 2 * launcher::param::TRIGSTEP;
          } else if (!is_reverse_) {
            target_trig_angle_ = trig_angle_ + launcher::param::TRIGSTEP;
          }
          is_reverse_ = !is_reverse_;
          last_jam_time_ = now;
        }
      } break;

      default:
        break;
    }
    last_trig_mod_ = trig_mod_;
    /*判断是否成功发射*/
    UpdateShotJudge();
    last_fire_notify_ = launcher_cmd_.isfire;
  }
  void Control() {
    float out_trig = 0.0f;
    float out_fric_0 = 0.0f;
    float out_fric_1 = 0.0f;
    Motor::Feedback trig_fb{};
    Motor::Feedback fric_0_fb{};
    Motor::Feedback fric_1_fb{};
    bool relax = false;

    SetFricTargetByEvent();

    if (launcher_event_ == LauncherEvent::SET_FRICMODE_RELAX) {
      relax = true;
    } else {
      if (trig_mode_ != TrigMode::RELAX) {
        TrigControl(out_trig, target_trig_angle_, dt_);
      }
      FricControl(out_fric_0, out_fric_1, target_rpm_, dt_);
      trig_fb = param_trig_;
      fric_0_fb = param_fric_0_;
      fric_1_fb = param_fric_1_;
    }

    if (relax) {
      motor_trig_->Relax();
      motor_fric_0_->Relax();
      motor_fric_1_->Relax();
      return;
    }

    auto cmd_trig = Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                                    .reduction_ratio = 36.0f,
                                    .velocity = out_trig};
    auto cmd_fric_0 = Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                                      .reduction_ratio = 19.0f,
                                      .velocity = out_fric_0};
    auto cmd_fric_1 = Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                                      .reduction_ratio = 19.0f,
                                      .velocity = out_fric_1};
    auto motor_control = [&](Motor* motor, const Motor::Feedback& fb,
                             const Motor::MotorCmd& cmd) {
      if (fb.state == 0) {
        motor->Enable();
      } else if (fb.state != 0 && fb.state != 1) {
        motor->ClearError();
      } else {
        motor->Control(cmd);
      }
    };

    motor_control(motor_trig_, trig_fb, cmd_trig);
    motor_control(motor_fric_0_, fric_0_fb, cmd_fric_0);
    motor_control(motor_fric_1_, fric_1_fb, cmd_fric_1);
  }

  void SetControlDt(float dt) { dt_ = dt; }

  /**
   * @brief 设置摩擦轮模式事件
   * @param mode 事件ID，对应 LauncherEvent
   * @details 切换模式后同步复位相关PID，避免模式切换瞬态冲击。
   */
  void SetMode(uint32_t mode) {
    launcher_event_ = static_cast<LauncherEvent>(mode);
    pid_fric_0_.Reset();
    pid_fric_1_.Reset();
    pid_trig_angle_.Reset();
    pid_trig_sp_.Reset();
  }

  /**
   * @brief 失控处理
   * @details 复位状态机与PID并关闭输出，确保发射机构进入安全状态。
   */
  void LostCtrl() {
    launcher_event_ = LauncherEvent::SET_FRICMODE_RELAX;
    launcher_state_ = LauncherState::RELAX;
    trig_mode_ = TrigMode::RELAX;

    pid_fric_0_.Reset();
    pid_fric_1_.Reset();
    pid_trig_angle_.Reset();
    pid_trig_sp_.Reset();

    target_trig_angle_ = trig_angle_;
    shoot_active_ = false;
    shot_start_time_ = 0;
    press_continue_ = false;
    launcher_cmd_.isfire = false;

    motor_trig_->Disable();
    motor_fric_0_->Relax();
    motor_fric_1_->Relax();
  }

  /**
   * @brief 监控回调
   */
  void OnMonitor() {}

  /**
   * @brief 调试命令入口
   */
#ifdef DEBUG
  int DebugCommand(int argc, char** argv);
#endif

  /* 外壳可直接写入的命令数据 */
  CMD::LauncherCMD launcher_cmd_{};  // NOLINT

 private:
  RMMotor* motor_fric_0_;
  RMMotor* motor_fric_1_;
  RMMotor* motor_trig_;

  Motor::Feedback param_fric_0_{};
  Motor::Feedback param_fric_1_{};
  Motor::Feedback param_trig_{};

  LibXR::PID<float> pid_trig_angle_;
  LibXR::PID<float> pid_trig_sp_;
  LibXR::PID<float> pid_fric_0_;
  LibXR::PID<float> pid_fric_1_;

  LauncherParam param_;

  float dt_ = 0.0f;
  float target_rpm_ = 0.0f;
  float trig_freq_ = 0.0f;

  float trig_angle_ = 0.0f;
  float target_trig_angle_ = 0.0f;
  float last_motor_angle_ = 0.0f;

  float number_ = 0.0f;
  float shoot_dt_ = 0.0f;

  bool last_fire_notify_ = false;
  bool press_continue_ = false;
  bool is_reverse_ = false;
  bool shoot_active_ = false;

  float jam_keep_time_s_ = 0.0f;

  LibXR::MillisecondTimestamp fire_press_time_ = 0;
  LibXR::MillisecondTimestamp last_trig_time_ = 0;
  LibXR::MillisecondTimestamp last_jam_time_ = 0;
  LibXR::MillisecondTimestamp last_heat_time_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;
  LibXR::MillisecondTimestamp shoot_time_ = 0;
  LibXR::MillisecondTimestamp receive_fire_time_ = 0;
  LibXR::MillisecondTimestamp shot_start_time_ = 0;

  LibXR::Topic shoot_waiting_ = LibXR::Topic::CreateTopic<float>("shoot_dt");
  LibXR::Topic shoot_number_ = LibXR::Topic::CreateTopic<float>("shoot_number");
  LibXR::Topic shoot_freq_ = LibXR::Topic::CreateTopic<float>("trig_freq");

  LauncherEvent launcher_event_ = LauncherEvent::SET_FRICMODE_RELAX;
  LauncherState launcher_state_ = LauncherState::RELAX;
  TrigMode trig_mode_ = TrigMode::RELAX;
  TrigMode last_trig_mode_ = TrigMode::RELAX;

  RefereeData referee_data_{.heat_limit = 0.0f, .heat_cooling = 0.0f};
  HeatLimit heat_limit_{
      .single_heat = 0.0f,
      .launched_num = 0.0f,
      .current_heat = 0.0f,
      .heat_threshold = 0.0f,
      .allow_fire = true,
  };
  RMMotor* motor_fric_0_;
  RMMotor* motor_fric_1_;
  RMMotor* motor_trig_;

  TRIGMODE last_trig_mod_ = TRIGMODE::RELAX;
  TRIGMODE trig_mod_ = TRIGMODE::RELAX;
  float trig_angle_ = 0.0f;
  float target_trig_angle_ = 0.0f;
  float number_ = 0;
  float last_trig_angle_ = 0.0f;
  float trig_freq_ = 0.0f;
  LibXR::MillisecondTimestamp last_trig_time_ = 0;
  Motor::Feedback param_fric_0_;
  Motor::Feedback param_fric_1_;
  Motor::Feedback param_trig_;
  LibXR::PID<float> pid_trig_angle_;
  LibXR::PID<float> pid_trig_sp_;
  LibXR::PID<float> pid_fric_0_;
  LibXR::PID<float> pid_fric_1_;
  LauncherParam param_;
  CMD::LauncherCMD launcher_cmd_;
  float dt_ = 0;
  float shoot_dt_ = 0.0f;
  LibXR::MillisecondTimestamp shoot_time_ = 0.0f;
  LibXR::MillisecondTimestamp receive_fire_time_ = 0.0f;
  LibXR::Topic shoot_wating_ = LibXR::Topic::CreateTopic<float>("shoot_dt");
  LibXR::Topic shoot_number_ = LibXR::Topic::CreateTopic<float>("shoot_number");
  LibXR::Topic shoot_freq_ = LibXR::Topic::CreateTopic<float>("trig_freq");
  bool press_continue_ = false;
  bool last_fire_notify_ = false;
  LibXR::MillisecondTimestamp fire_press_time_ = 0;
  LibXR::MillisecondTimestamp last_heat_time_ = 0.0f;
  LibXR::Thread thread_;
  LibXR::Mutex mutex_;
  LibXR::MillisecondTimestamp last_jam_time_ = 0.0f;
  LibXR::MillisecondTimestamp jam_keep_time_ = 0.0f;
  LibXR::MillisecondTimestamp last_online_time_ = 0.0f;
  CMD* cmd_;
  bool is_reverse_ = false;
  bool shoot_active_ = false;
  float target_rpm_ = 0;
  void TrigControl(float& out_trig, float target_trig_angle_, float dt_) {
    float plate_omega_ref = pid_trig_angle_.Calculate(
        target_trig_angle, trig_angle_,
        param_trig_.omega / param_.trig_gear_ratio, dt);
    float omega_limit =
        static_cast<float>(1.5f * M_2PI * trig_freq_ / param_.num_trig_tooth);
    float motor_omega_ref =
        std::clamp(plate_omega_ref, -omega_limit, omega_limit);
    out_trig = pid_trig_sp_.Calculate(
        motor_omega_ref, param_trig_.omega / param_.trig_gear_ratio, dt);
  }
  void FricControl(float& out_fric_0, float& out_fric_1, float target_rpm,
                   float dt_) {
    out_fric_0 = pid_fric_0_.Calculate(target_rpm, param_fric_0_.velocity, dt_);
    out_fric_1 = pid_fric_1_.Calculate(target_rpm, param_fric_1_.velocity, dt_);
    /*缓停*/
    if (launcher_event_ == LauncherEvent::SET_FRICMODE_SAFE) {
      pid_fric_0_.SetOutLimit(1.5f);
      pid_fric_1_.SetOutLimit(1.5f);
    }
  }
};

#ifdef DEBUG
#define INFANTRY_LAUNCHER_DEBUG_IMPL
#include "InfantryLauncherDebug.inl"
#undef INFANTRY_LAUNCHER_DEBUG_IMPL
#endif
