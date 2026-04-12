#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - motor_fric_front_left: '@&motor_fric_0'
  - motor_fric_front_right: '@&motor_fric_1'
  - motor_trig: '@&motor_trig'
  - task_stack_depth: 4096
  - pid_trig_angle:
      k: 1.0
      p: 4000.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 4000.0
      cycle: false
  - pid_trig_speed:
      k: 1.0
      p: 0.0012
      i: 0.0005
      d: 0.0
      i_limit: 1.0
      out_limit: 1.0
      cycle: false
  - pid_fric_speed_0:
      k: 1.0
      p: 0.002
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  - pid_fric_speed_1:
      k: 1.0
      p: 0.002
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 1.0
      cycle: false
  - launcher_param:
      fric1_setpoint_speed: 6500.0
      trig_gear_ratio: 36
      num_trig_tooth: 10
      trig_freq_: 10.0
  - cmd: '@&cmd'
  - thread_priority: LibXR::Thread::Priority::HIGH
required_hardware:
  - dr16
  - can
depends:
  - qdu-future/CMD
  - qdu-future/RMMotor
=== END MANIFEST === */
// clang-format on
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "CMD.hpp"
#include "Motor.hpp"
#include "RMMotor.hpp"
#include "Referee.hpp"
#include "app_framework.hpp"
#include "event.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"

#ifdef DEBUG
#include "DebugCore.hpp"
#include "ramfs.hpp"
#endif

namespace launcher::param {
constexpr float TRIG_STEP = static_cast<float>(M_2PI) / 10.0f;
constexpr float JAM_TORQUE = 0.025f;
constexpr float JAM_TOGGLE_INTERVAL_SEC = 0.02f;
constexpr float LONG_PRESS_THRESHOLD_SEC = 0.5f;
constexpr float HEAT_TICK_SEC = 0.05f;
constexpr float SHOT_PROGRESS_EPSILON = 1e-4f;
constexpr float TRIGGER_SETTLE_ANGLE = 0.2f * TRIG_STEP;
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
    float cooling_rate = 0.0f;
    float heat_limit = 0.0f;
    float current_heat_17 = 0.0f;
  };

  struct LauncherParam {
    float fric1_setpoint_speed;
    float trig_gear_ratio;
    uint8_t num_trig_tooth;
  };

  struct HeatLimit {
    float single_heat;
    float launched_num;
    float current_heat;
    float heat_threshold;
    bool allow_fire;
    float merge;
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
  InfantryLauncher(
      LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
      RMMotor* motor_fric_0, RMMotor* motor_fric_1, RMMotor* motor_trig,
      uint32_t task_stack_depth, LibXR::PID<float>::Param pid_param_trig_angle,
      LibXR::PID<float>::Param pid_param_trig_speed,
      LibXR::PID<float>::Param pid_param_fric_0,
      LibXR::PID<float>::Param pid_param_fric_1, LauncherParam launch_param,
      CMD* cmd,
      LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::HIGH)
      : motor_fric_0_(motor_fric_0),
        motor_fric_1_(motor_fric_1),
        motor_trig_(motor_trig),
        pid_trig_angle_(pid_param_trig_angle),
        pid_trig_sp_(pid_param_trig_speed),
        pid_fric_0_(pid_param_fric_0),
        pid_fric_1_(pid_param_fric_1),
        param_(launch_param)
#ifdef DEBUG
        ,
        cmd_file_(LibXR::RamFS::CreateFile(
            "launcher",
            debug_core::command_thunk<InfantryLauncher,
                                      &InfantryLauncher::DebugCommand>,
            this))
#endif
  {
    UNUSED(app);

#ifdef DEBUG
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);
#endif
    thread_.Create(this, ThreadFunc, "LauncherThread", task_stack_depth,
                   thread_priority);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, InfantryLauncher* self, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          self->mutex_.Lock();
          self->LostCtrl();
          self->mutex_.Unlock();
        },
        this);

    auto start_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, InfantryLauncher* self, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          self->mutex_.Lock();
          self->SetMode(
              static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_RELAX));
          self->mutex_.Unlock();
        },
        this);

    cmd->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
    cmd->GetEvent().Register(CMD::CMD_EVENT_START_CTRL, start_ctrl_callback);

    auto event_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, InfantryLauncher* self, uint32_t event_id) {
          UNUSED(in_isr);
          self->mutex_.Lock();
          self->SetMode(event_id);
          self->mutex_.Unlock();
        },
        this);
    launcher_event.Register(
        static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_RELAX),
        event_callback);
    launcher_event.Register(
        static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_SAFE),
        event_callback);
    launcher_event.Register(
        static_cast<uint32_t>(LauncherEvent::SET_FRICMODE_READY),
        event_callback);
  }

  static void ThreadFunc(InfantryLauncher* self) {
    LibXR::Topic::ASyncSubscriber<CMD::LauncherCMD> cmd_sub("launcher_cmd");
    LibXR::Topic::ASyncSubscriber<Referee::LauncherPack> launcher_ref(
        "launcher_ref");
    cmd_sub.StartWaiting();
    launcher_ref.StartWaiting();
    self->last_wakeup_time_ = LibXR::Timebase::GetMilliseconds();
    self->last_online_time_ = LibXR::Timebase::GetMicroseconds();
    while (true) {
      auto now = LibXR::Timebase::GetMicroseconds();
      self->SetControlDt((now - self->last_online_time_).ToSecondf());
      self->last_online_time_ = now;

      if (cmd_sub.Available()) {
        self->launcher_cmd_ = cmd_sub.GetData();
        cmd_sub.StartWaiting();
      }
      if (launcher_ref.Available()) {
        self->ref_data_.heat_limit =
            launcher_ref.GetData().rs.shooter_heat_limit;
        self->ref_data_.cooling_rate =
            launcher_ref.GetData().rs.shooter_cooling_value;
        self->ref_data_.current_heat_17 =
            launcher_ref.GetData().launcher_id1_17_heat;
        self->robot_level = launcher_ref.GetData().rs.robot_level;
        launcher_ref.StartWaiting();
      }
      self->mutex_.Lock();
      self->Update();
      self->RunStateMachine();
      self->mutex_.Unlock();
      self->Control();
      LibXR::Thread::Sleep(2);
    }
  }

  /**
   * @brief 数据处理主入口
   * @details 更新周期时间、电机反馈、拨弹角度，并刷新发射器总状态。
   */
  void Update() {
    last_online_time_ = LibXR::Timebase::GetMicroseconds();
    heat_limit_.single_heat = 10.0f;
    robot_level = 5;
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

    UpdateLauncherState();
  }

  /**
   * @brief 控制输出
   * @details 计算拨盘与摩擦轮控制量并下发到电机，包含电机状态检查和错误恢复。
   */
  void Control() {
    // float out_trig = 0.0f;
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
        TrigControl(out_trig_, target_trig_angle_, dt_);
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
                                    .velocity = out_trig_};
    auto cmd_fric_0 = Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                                      .reduction_ratio = 1.0f,
                                      .velocity = out_fric_0};
    auto cmd_fric_1 = Motor::MotorCmd{.mode = Motor::ControlMode::MODE_CURRENT,
                                      .reduction_ratio = 1.0f,
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
    press_continue_ = false;
    launcher_cmd_.isfire = false;

    motor_trig_->Disable();
    motor_fric_0_->Relax();
    motor_fric_1_->Relax();
  }

  LibXR::Event& GetEvent() { return launcher_event; }

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
  RefereeData ref_data_;

 private:
  RMMotor* motor_fric_0_;
  RMMotor* motor_fric_1_;
  RMMotor* motor_trig_;
  float last_trig_angle;
  Motor::Feedback param_fric_0_{};
  Motor::Feedback param_fric_1_{};
  Motor::Feedback param_trig_{};

  LibXR::PID<float> pid_trig_angle_;
  LibXR::PID<float> pid_trig_sp_;
  LibXR::PID<float> pid_fric_0_;
  LibXR::PID<float> pid_fric_1_;

  LauncherParam param_;
  LibXR::Event launcher_event;
  LibXR::Thread thread_;
  uint8_t robot_level; /* 机器人等级 */

  float out_trig_ = 0.0f;

  float expect_trig_freq_;
  float dt_ = 0.0f;
  float target_rpm_ = 0.0f;
  float trig_freq_ = 0.0f;
  float last_trig_freq_ = 0.0f;
  float trig_angle_ = 0.0f;
  float target_trig_angle_ = 0.0f;
  float last_motor_angle_ = 0.0f;

  float number_ = 0.0f;
  float shoot_dt_ = 0.0f;

  bool last_fire_notify_ = false;
  bool press_continue_ = false;
  bool is_reverse_ = false;
  bool shoot_active_ = false;
  bool heat_initialized_ = false;
  bool trigger_step_active_ = false;

  float jam_keep_time_s_ = 0.0f;
  float shot_progress_ = 0.0f;

  LibXR::MillisecondTimestamp fire_press_time_ = 0;
  LibXR::MillisecondTimestamp last_trig_time_ = 0;
  LibXR::MillisecondTimestamp last_jam_time_ = 0;
  LibXR::MillisecondTimestamp last_heat_time_ = 0;
  LibXR::MillisecondTimestamp last_check_time_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;
  LibXR::MillisecondTimestamp shot_start_time_ = 0;
  LibXR::MillisecondTimestamp last_add_time_ = 0;
  LibXR::MillisecondTimestamp last_wakeup_time_ = 0;

  LauncherEvent launcher_event_ = LauncherEvent::SET_FRICMODE_RELAX;
  LauncherState launcher_state_ = LauncherState::RELAX;
  TrigMode trig_mode_ = TrigMode::RELAX;
  TrigMode last_trig_mode_ = TrigMode::RELAX;

  HeatLimit heat_limit_{
      .single_heat = 0.0f,
      .launched_num = 0.0f,
      .current_heat = 0.0f,
      .heat_threshold = 0.0f,
      .allow_fire = true,
      .merge = 0.0f,
  };
  LibXR::Mutex mutex_;
#ifdef DEBUG
  LibXR::RamFS::File cmd_file_;
#endif

  /*-----------------工具函数---------------------------------------------------*/

  /**
   * @brief 更新发射器总状态
   * @details 基于卡弹判据、摩擦轮模式与热量许可，计算
   * RELAX/STOP/NORMAL/JAMMED。
   */
  void UpdateLauncherState() {
    if (fabsf(param_trig_.torque) > launcher::param::JAM_TORQUE) {
      launcher_state_ = LauncherState::JAMMED;
      return;
    }

    if (launcher_event_ != LauncherEvent::SET_FRICMODE_READY) {
      launcher_state_ = LauncherState::RELAX;
      return;
    }

    if (!heat_limit_.allow_fire) {
      launcher_state_ = LauncherState::STOP;
      return;
    }

    launcher_state_ =
        launcher_cmd_.isfire ? LauncherState::NORMAL : LauncherState::STOP;
  }

  /**
   * @brief 执行拨弹状态机
   * @details 依次更新拨盘模式、拨盘目标角度和发射判定，并刷新按键边沿状态。
   */
  void RunStateMachine() {
    auto now = LibXR::Timebase::GetMilliseconds();
    UpdateTriggerMode(now);
    UpdateTriggerSetpoint(now);
    CurrentHeat(now);
    UpdateHeatControl(now);

    last_fire_notify_ = launcher_cmd_.isfire;
  }

  /**
   * @brief 根据发射器状态切换拨盘模式
   * @param now 当前时间戳
   * @details 处理单发/连发/卡弹模式切换，其中连发由按键长按时长触发。
   */
  void UpdateTriggerMode(LibXR::MillisecondTimestamp now) {
    switch (launcher_state_) {
      case LauncherState::RELAX:
        trig_mode_ = TrigMode::RELAX;
        press_continue_ = false;
        break;

      case LauncherState::STOP:
        trig_mode_ = TrigMode::SAFE;
        press_continue_ = false;
        break;

      case LauncherState::NORMAL:
        if (!last_fire_notify_) {
          fire_press_time_ = now;
          press_continue_ = false;
          trig_mode_ = TrigMode::SINGLE;
        } else {
          if (!press_continue_ &&
              (now - fire_press_time_).ToSecondf() >
                  launcher::param::LONG_PRESS_THRESHOLD_SEC) {
            press_continue_ = true;
          }
          trig_mode_ = press_continue_ ? TrigMode::CONTINUE : TrigMode::SINGLE;
        }
        break;

      case LauncherState::JAMMED:
        trig_mode_ = TrigMode::JAM;
        break;
    }
  }

  /**
   * @brief 更新拨盘目标角度
   * @param now 当前时间戳
   * @details 根据 TrigMode 生成目标角度；卡弹模式下周期切换正反向退弹角度。
   */
  void UpdateTriggerSetpoint(LibXR::MillisecondTimestamp now) {
    if (trigger_step_active_) {
      float angle_error = fabsf(target_trig_angle_ - trig_angle_);
      if (angle_error <= launcher::param::TRIGGER_SETTLE_ANGLE) {
        trigger_step_active_ = false;
      }
    }

    switch (trig_mode_) {
      case TrigMode::RELAX:
      case TrigMode::SAFE:
        target_trig_angle_ = trig_angle_;
        trigger_step_active_ = false;
        shoot_active_ = false;
        break;

      case TrigMode::SINGLE:
        if (last_trig_mode_ == TrigMode::SAFE ||
            last_trig_mode_ == TrigMode::RELAX ||
            last_trig_mode_ == TrigMode::JAM) {
          target_trig_angle_ += launcher::param::TRIG_STEP;
          trigger_step_active_ = true;
          shoot_active_ = true;
        }
        break;

      case TrigMode::CONTINUE: {
        float trig_freq = std::max(trig_freq_, 1e-3f);
        float interval_s = 1.0f / trig_freq;
        float since_last = (now - last_trig_time_).ToSecondf();
        if (!trigger_step_active_ && since_last >= interval_s) {
          target_trig_angle_ += launcher::param::TRIG_STEP;
          trigger_step_active_ = true;
          last_trig_time_ = now;
          shoot_active_ = true;
        }
      } break;

      case TrigMode::JAM: {
        trigger_step_active_ = false;
        shoot_active_ = false;
        jam_keep_time_s_ = (now - last_jam_time_).ToSecondf();
        if (jam_keep_time_s_ >= launcher::param::JAM_TOGGLE_INTERVAL_SEC) {
          if (last_trig_mode_ != TrigMode::JAM) {
            is_reverse_ = true;
          }
          target_trig_angle_ =
              trig_angle_ + (is_reverse_ ? -0.80f * launcher::param::TRIG_STEP
                                         : launcher::param::TRIG_STEP);
          is_reverse_ = !is_reverse_;
        }
        last_jam_time_ = now;
      } break;
    }

    last_trig_mode_ = trig_mode_;
  }

  /**
   * @brief 根据模式设置摩擦轮目标转速
   * @details RELAX/SAFE 下目标转速为0，READY 下使用配置转速。
   */
  void SetFricTargetByEvent() {
    switch (launcher_event_) {
      case LauncherEvent::SET_FRICMODE_RELAX:
      case LauncherEvent::SET_FRICMODE_SAFE:
        target_rpm_ = 0.0f;
        break;
      case LauncherEvent::SET_FRICMODE_READY:
        target_rpm_ = param_.fric1_setpoint_speed;
        break;
      default:
        break;
    }
  }

  /**
   * @brief 热量管理与弹频调度
   * @details 周期更新当前热量，计算是否允许发射，并依据剩余热量调整目标弹频。
   */
  void UpdateHeatControl(LibXR::MillisecondTimestamp now) {
    float delta_time = (now - last_heat_time_).ToSecondf();

    if (delta_time < launcher::param::HEAT_TICK_SEC) {
      return;
    }
    last_heat_time_ = now;

    float residuary_heat =
        ref_data_.heat_limit - heat_limit_.current_heat - heat_limit_.merge;
    /*热量限制阈值阈值，当剩余热量低于阈值时，不允许发射*/
    heat_limit_.allow_fire = (residuary_heat > heat_limit_.merge);

    if (!heat_limit_.allow_fire) {
      trig_freq_ = 0;
      return;
    }

    if (residuary_heat <=
        heat_limit_.single_heat * heat_limit_.heat_threshold) {
      float safe_freq = ref_data_.cooling_rate / heat_limit_.single_heat;
      float ratio = residuary_heat /
                    (heat_limit_.single_heat * heat_limit_.heat_threshold);
      trig_freq_ = ratio * (expect_trig_freq_ - safe_freq) + safe_freq;
      return;
    }

    trig_freq_ = expect_trig_freq_;
  }
  void CurrentHeat(LibXR::MillisecondTimestamp now) {
    float delta_time = (now - last_check_time_).ToSecondf();

    if (!heat_initialized_) {
      heat_initialized_ = true;
      last_check_time_ = now;
      last_trig_angle = trig_angle_;
      return;
    }

    last_check_time_ = now;
    heat_limit_.launched_num = 0.0f;

    if (delta_time > 0.0f) {
      heat_limit_.current_heat -= ref_data_.cooling_rate * delta_time;
    }
    if (heat_limit_.current_heat <= 0) {
      heat_limit_.current_heat = 0;
    }

    float delta_teeth =
        (trig_angle_ - last_trig_angle) / launcher::param::TRIG_STEP;
    last_trig_angle = trig_angle_;

    if (launcher_event_ == LauncherEvent::SET_FRICMODE_READY) {
      shot_progress_ += delta_teeth;
      if (shot_progress_ < 0.0f) {
        shot_progress_ = 0.0f;
      }
    } else {
      shot_progress_ = 0.0f;
    }

    if (shot_progress_ >= 1.0f - launcher::param::SHOT_PROGRESS_EPSILON) {
      heat_limit_.launched_num = floorf(shot_progress_);
      /*不扔掉小数点后的热量，减小误差*/
      shot_progress_ -= heat_limit_.launched_num;
      heat_limit_.current_heat +=
          heat_limit_.single_heat * heat_limit_.launched_num;
      number_ += heat_limit_.launched_num;
    }
    /*根据机器人等级不同，改变他的不同阈值*/
    float performance_system_shooter =
        1; /*后期裁判系统获取：冷却优先(1)or爆发优先(2)*/
    if (performance_system_shooter == 1) {
      switch (robot_level) {
        case 1:
        case 2:
          heat_limit_.heat_threshold = 2.50f;
          heat_limit_.merge = 0;
          expect_trig_freq_ = 12;
          break;
        case 3:
        case 4:
        case 5: {
          heat_limit_.heat_threshold = 3.0f;
          heat_limit_.merge = 0;
          expect_trig_freq_ = 18;
          /*弥补误差*/
          if (!last_fire_notify_ && launcher_cmd_.isfire) {
            heat_limit_.current_heat += 10;
          }
          if (launcher_cmd_.isfire) {
            if ((now - last_add_time_).ToSecondf() > 8.0f) {
              heat_limit_.current_heat += 5;
              last_add_time_ = now;
            }
          }
        } break;
        default:
          break;
      }
    } else if (performance_system_shooter == 2) {
      switch (robot_level) {
        case 1:
        case 2: {
          expect_trig_freq_ = 6;
          heat_limit_.heat_threshold = 2.0f;
          heat_limit_.merge = 0;
          /*冷却优先总热量低，每次开始发射提供双发容错*/
          if (last_fire_notify_ != true && (launcher_cmd_.isfire) == true) {
            heat_limit_.current_heat += 10;
            shot_start_time_ = now;
          }
          if ((now - shot_start_time_).ToSecondf() > 8.0f) {
            shot_start_time_ = now;
            heat_limit_.current_heat += 2;
          }
        } break;
        case 3:
        case 4:
        case 5: {
          expect_trig_freq_ = 16;
          heat_limit_.heat_threshold = 3.0f;
          heat_limit_.merge = 10;
          /*弥补误差*/
        }
        default:

          break;
      }
    }
    last_trig_freq_ = trig_freq_;
  }
  /**
   * @brief 拨盘控制解算
   * @param out_trig 拨盘控制输出
   * @param target_trig_angle 拨盘目标角度
   * @param dt 控制周期
   * @details 角度环生成参考速度，速度环生成最终控制输出，并进行速度限幅。
   */
  void TrigControl(float& out_trig, float target_trig_angle, float dt) {
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

  /**
   * @brief 摩擦轮控制解算
   * @param out_fric_0 摩擦轮0控制输出
   * @param out_fric_1 摩擦轮1控制输出
   * @param target_rpm 摩擦轮目标转速
   * @param dt 控制周期
   * @details 速度环计算摩擦轮输出，SAFE 模式下对输出限幅做缓停处理。
   */
  void FricControl(float& out_fric_0, float& out_fric_1, float target_rpm,
                   float dt) {
    out_fric_0 = pid_fric_0_.Calculate(target_rpm, param_fric_0_.velocity, dt);
    out_fric_1 = pid_fric_1_.Calculate(target_rpm, param_fric_1_.velocity, dt);

    if (launcher_event_ == LauncherEvent::SET_FRICMODE_SAFE) {
      out_fric_0 /= 50;
      out_fric_1 /= 50;
    }
  }
};

#ifdef DEBUG
#define INFANTRY_LAUNCHER_DEBUG_IMPL
#include "InfantryLauncherDebug.inl"
#undef INFANTRY_LAUNCHER_DEBUG_IMPL
#endif
