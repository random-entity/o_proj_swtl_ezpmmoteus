#pragma once

#include <condition_variable>
#include <map>
#include <memory>

#include "moteus.h"
#ifdef __RASPBERRY_PI__
#include "pi3hat.h"
#include "pi3hat_moteus_transport.h"
#endif

#include "slcm_multiplex.hpp"
#include "slcm_parser.hpp"
#include "slcm_printer.hpp"
#include "slcm_utils.hpp"

#ifdef NaN
#undef NaN
#endif
#define NaN (std::numeric_limits<double>::quiet_NaN())

using namespace mjbots;

namespace som {
/// Vocabulary commonly used in this API:
/// * Command     : A PositionMode command represented by a single
///                 moteus::PositionMode::Command object, i.e. data composed of
///                 desired position and other values for elements of enum
///                 CommandItem.
/// * CommandItem : Represents a physical quality, desired value of which can be
///                 commanded to moteus as a PositionMode command.
/// * Reply       : A moteus state data represented by a single
///                 moteus::Query::Result object, i.e. data composed of
///                 reported position and other values for elements of enum
///                 ReplyItem.
/// * ReplyItem   : Represents a physical quality, value of which is
///                 reported by moteus as a Query Result.
///
/// Commands {
///   ID 1 => Command {
///             CommandItem position = 00.00
///             CommandItem velocity = 00.00
///             ...
///           },
///   ID 2 => Command {
///             CommandItem position = 00.00
///             CommandItem velocity = 00.00
///             ...
///           },
///   ...
/// }
///
/// Replies {
///   ID 1 => Reply {
///             ReplyItem mode     = 00
///             ReplyItem position = 00.00
///             ...
///           },
///   ID 2 => Reply {
///             ReplyItem mode     = 00
///             ReplyItem position = 00.00
///             ...
///           },
///   ...
/// }
///
///                       Commands
///     +--------------+------------>+--------------------+
///     |              |             |                    |
///     | Your program |  Transport  | System of moteuses |
///     |              |             |                    |
///     +--------------+<------------+--------------------+
///                        Replies

class ServoSystem {
 private:
  class Servo {
    friend class ServoSystem;

   private:
    Servo(const int id, const int bus,
          const std::shared_ptr<moteus::Transport>& transport,
          const moteus::PositionMode::Format& cmd_fmt,
          const moteus::PositionMode::Command& init_cmd,
          const CommandPositionRelativeTo usr_cmd_pos_rel_to,
          const moteus::Query::Format& rpl_fmt,
          const ReplyPositionRelativeTo usr_rpl_pos_rel_to,  //
          const bool use_aux2,
          const ReplyPositionRelativeTo usr_rpl_aux2_pos_rel_to)
        : controller_{std::make_unique<moteus::Controller>([&]() {
            moteus::Controller::Options options;
            options.id = id;
            options.bus = bus;
            options.transport = transport;
            options.position_format = cmd_fmt;
            options.query_format = rpl_fmt;
            return options;
          }())},
          usr_cmd_{init_cmd},
          usr_cmd_pos_rel_to_{usr_cmd_pos_rel_to},
          usr_rpl_pos_rel_to_{usr_rpl_pos_rel_to},
          use_aux2_{use_aux2},
          usr_rpl_aux2_pos_rel_to_{usr_rpl_aux2_pos_rel_to} {
      /* TODO: moteus firmware-API compatibility check */

      // Execute Stop Command to clears all faults.
      const auto maybe_reply = controller_->SetStop();
      std::cout << (maybe_reply ? "Got" : "Failed to get")
                << " reply from initial Stop Command for Servo ID " << id << "."
                << std::endl;

      // Try setting base position for 1 second.
      SetBasePosition(1.0);
      if (use_aux2) {
        SetBaseAux2Position(1.0);
      }
    }

    const int GetId() { return controller_->options().id; }

    moteus::Query::Result GetReply() { return usr_rpl(); }

    void SetReply(const moteus::Query::Result& new_sys_rpl) {
      const auto delta = new_sys_rpl.abs_position - sys_rpl_.abs_position;
      if (delta > 0.5) {
        aux2_revs_--;
      } else if (delta < -0.5) {
        aux2_revs_++;
      }
      sys_rpl_ = new_sys_rpl;
    }

    const bool InitSucceeded() {
      return use_aux2_
                 ? std::isfinite(base_pos_) && std::isfinite(base_aux2_pos_)
                 : std::isfinite(base_pos_);
    }

    void SetCommandPositionRelativeTo(CommandPositionRelativeTo new_val) {
      usr_cmd_pos_rel_to_ = new_val;
    }

    void SetReplyPositionRelativeTo(ReplyPositionRelativeTo new_val) {
      usr_rpl_pos_rel_to_ = new_val;
    }

    void SetReplyAux2PositionRelativeTo(ReplyPositionRelativeTo new_val) {
      usr_rpl_aux2_pos_rel_to_ = new_val;
    }

    enum class WhichPosition { Internal, Aux2 };

    bool SetBasePosition(const double time_limit) {
      return SetBase(time_limit, WhichPosition::Internal);
    }

    bool SetBaseAux2Position(const double time_limit) {
      return SetBase(time_limit, WhichPosition::Aux2);
    }

    bool SetBase(const double time_limit, WhichPosition which_pos) {
      const std::string which_str =
          (which_pos == WhichPosition::Internal) ? "" : "aux2 ";
      const int id = GetId();
      std::cout << "Attempting to get current " << which_str
                << "position from Servo ID " << id << "..." << std::endl;
      const double time_init = Utils::GetTime();
      while (true) {
        const auto& maybe_reply = controller_->SetQuery();
        if (!maybe_reply) {
          const double time_passed = Utils::GetTime() - time_init;
          const double time_left = time_limit - time_passed;
          if (time_left > 0.0) {
            std::cout << "Not getting reply from Servo ID " << id
                      << ", but will try " << time_left << " more seconds...\r";
            ::usleep(1e6 / 100.0);
            continue;
          } else {
            std::cout << "Failed to get reply from Servo ID " << id << " for "
                      << time_limit << " seconds.  " << "Base " << which_str
                      << "position setting failed." << std::endl;
            return false;
          }
        } else {
          const double cur_val = (which_pos == WhichPosition::Internal)
                                     ? maybe_reply->values.position
                                     : maybe_reply->values.abs_position;
          if (std::isfinite(cur_val)) {
            auto& base = (which_pos == WhichPosition::Internal)
                             ? base_pos_
                             : base_aux2_pos_;
            base = cur_val;
            std::cout << "Successfully set base " << which_str
                      << "position to current " << which_str
                      << "position for Servo ID " << id << ": " << base
                      << std::endl;
            SetReply(maybe_reply->values);
            return true;
          } else {
            const double time_passed = Utils::GetTime() - time_init;
            const double time_left = time_limit - time_passed;
            if (time_left > 0.0) {
              std::cout << "Current " << which_str << "position for Servo ID "
                        << id << " is NaN, but will try " << time_left
                        << " more seconds...\r";
              ::usleep(1e6 / 100.0);
              continue;
            } else {
              std::cout << "Failed to get finite value as the current "
                        << which_str << "position for Servo ID " << id
                        << " for " << time_limit << " seconds.  " << "Base "
                        << which_str << "position setting failed." << std::endl;
              return false;
            }
          }
        }
      }
    }

    const std::unique_ptr<moteus::Controller> controller_;
    CommandPositionRelativeTo usr_cmd_pos_rel_to_;
    moteus::PositionMode::Command usr_cmd_;
    moteus::PositionMode::Command sys_cmd() {
      auto cmd = usr_cmd_;
      switch (usr_cmd_pos_rel_to_) {
        case CommandPositionRelativeTo::Base:
          if (std::isnan(base_pos_) || std::isnan(cmd.position)) break;
          cmd.position += base_pos_;
          break;
        case CommandPositionRelativeTo::Recent:
          if (std::isnan(cmd.position)) break;
          if (updated_last_cycle_) {
            cmd.position += sys_rpl_.position;
          } else {
            cmd.position = NaN;
          }
          break;
        default:
          break;
      }
      return cmd;
    }
    ReplyPositionRelativeTo usr_rpl_pos_rel_to_;
    ReplyPositionRelativeTo usr_rpl_aux2_pos_rel_to_;
    moteus::Query::Result sys_rpl_;  // aux2 position coiled.
                                     // Must update via dedicated setter
                                     // to track aux2 revolutions.
    moteus::Query::Result usr_rpl() {
      auto rpl = sys_rpl_;
      rpl.abs_position += aux2_revs_;  // Uncoil aux2 position.
      if (usr_rpl_pos_rel_to_ == ReplyPositionRelativeTo::Base) {
        rpl.position -= base_pos_;
      }
      if (usr_rpl_aux2_pos_rel_to_ == ReplyPositionRelativeTo::Base) {
        rpl.abs_position -= base_aux2_pos_;
      }
      return rpl;
    }
    bool updated_last_cycle_ = false;
    double base_pos_ = NaN;
    double base_aux2_pos_ = NaN;
    int aux2_revs_ = 0;
    bool use_aux2_;
  };

 protected:
  friend class LoopingThreadManager;
  class LoopingThreadManager {
    friend class ServoSystem;

   public:
    LoopingThreadManager(
        ServoSystem* servosystem,
        std::function<void(ServoSystem*, std::atomic_bool*)> task,
        const std::vector<std::string>& aliases)
        : servosystem_{servosystem},
          task_{task},
          aliases_{aliases},
          thread_{nullptr} {
      if (aliases_.empty()) {
        std::cout << "LoopingThreadManager initialization failed since "
                     "no aliases are given for this thread."
                  << std::endl;
        return;
      }
      for (const auto& alias : aliases_) {
        servosystem_->threads_mgr_[alias] = this;
      }
    }

    void Start() {
      if (!thread_) {
        terminated_ = false;
        thread_ = new std::thread(task_, servosystem_, &terminated_);
        std::cout << aliases_[0]
                  << " thread successfully initialized and detached."
                  << std::endl;
      } else {
        std::cout << aliases_[0] << " thread is already running." << std::endl;
      }
    }

    void Terminate() {
      if (thread_) {
        terminated_ = true;
        std::cout << "Waiting for " << aliases_[0] << " thread to join..."
                  << std::endl;
        thread_->join();
        delete thread_;
        thread_ = nullptr;
        std::cout << aliases_[0] << " thread successfully terminated."
                  << std::endl;
      } else {
        std::cout << aliases_[0] << " thread has not been running."
                  << std::endl;
      }
    }

    bool IsRunning() { return !!thread_; }

   private:
    const std::vector<std::string> aliases_;
    ServoSystem* const servosystem_;
    std::thread* thread_;
    const std::function<void(ServoSystem*, std::atomic_bool*)> task_;
    std::atomic_bool terminated_;
  };

 public:
  ServoSystem(const std::map<int, int>& id_bus_map,
              const std::string& cmd_conf_dir = "../config",
              const CommandPositionRelativeTo cmd_pos_rel_to =
                  CommandPositionRelativeTo::Base,
              const std::string& rpl_conf_dir = "../config",
              const ReplyPositionRelativeTo rpl_pos_rel_to =
                  ReplyPositionRelativeTo::Base,
              const bool use_aux2 = false,
              const ReplyPositionRelativeTo rpl_aux2_pos_rel_to =
                  ReplyPositionRelativeTo::Absolute)
      : executor_{
            this, &ServoSystem::ExecuteCommands, {"Executor", "exec", "x"}} {
    /// Get default transport. Priority order: pi3hat > fdcanusb > socketcan
#ifdef __RASPBERRY_PI__
    pi3hat::Pi3HatMoteusFactory::Register();
#endif
    transport_ = moteus::Controller::MakeSingletonTransport({});
    /* Print transport status and handle failure. */ {
      if (transport_) {
        std::string transport_name = Utils::GetClassName(transport_.get());
        if (!transport_name.empty()) {
          std::cout << "Default transport found: " << transport_name
                    << std::endl;
        } else {
          std::cout << "Default transport found, but failed to get its class "
                       "name."
                    << std::endl;
        }
      } else {
        std::cout << "Default transport not found.  Returning from "
                     "ServoSystem constructor without any initialization."
                  << std::endl;
        return;
      }
    }

    /// Parse the Command config file.
    const auto cmd_conf = Parser::ParseCommandConfig(cmd_conf_dir);
    const auto& cmd_fmt = cmd_conf.first;
    const auto& init_cmd = cmd_conf.second;

    /// Parse the Reply config file.
    const auto rpl_fmt = Parser::ParseReplyConfig(rpl_conf_dir);
    if (use_aux2 && rpl_fmt.abs_position == moteus::Resolution::kIgnore) {
      std::cout << "WARNING: ServoSystem was told the user will use aux2, "
                   "but the Reply resolution for aux2 position "
                   "is set to kIgnore."
                << std::endl;
    } else if (!use_aux2 &&
               rpl_fmt.abs_position != moteus::Resolution::kIgnore) {
      std::cout << "WARNING: ServoSystem was told the user will not use aux2, "
                   "but the Reply resolution for aux2 position "
                   "is not set to kIgnore."
                << std::endl;
    }

    /// Initialize Servos.
    std::set<int> failed_ids;
    for (const auto& id_bus : id_bus_map) {
      const auto& id = id_bus.first;
      const auto& bus = id_bus.second;
      const auto& maybe_servo = std::make_shared<Servo>(
          id, bus, transport_, cmd_fmt, init_cmd, cmd_pos_rel_to, rpl_fmt,
          rpl_pos_rel_to, use_aux2, rpl_aux2_pos_rel_to);
      if (maybe_servo) {
        servos_.insert({id, maybe_servo});
        ids_.insert(id);
        std::cout << "Initialization succeeded for Servo ID " << id << "."
                  << std::endl;
      } else {
        failed_ids.insert(id);
        std::cout << "Initialization failed for Servo ID " << id << "."
                  << std::endl;
      }
    }

    /* Print Servo initialization result */ {
      if (!failed_ids.empty()) {
        std::cout << "Excluded Servos with the following IDs "
                     "from the Servo list: ";
        for (auto id : failed_ids) {
          std::cout << id << " ";
        }
        std::cout << std::endl;
      }

      if (!servos_.empty()) {
        std::cout << "Ready to control Servos of the following IDs: ";
        for (const auto& id_servo : servos_) {
          std::cout << id_servo.first << " ";
        }
        std::cout << std::endl;
      } else {
        std::cout << "No available Servos found." << std::endl;
      }
    }
  }

  ///////////////////////////////////////////////////////////////
  ///           Commands you can send to the Servos.          ///

  std::set<int> GetIds() { return ids_; }

  void ListenInternalCommand(bool listen) { listen_.internal = listen; }

  void ListenExternalCommand(bool listen) { listen_.external = listen; }

  void Command(const std::map<int, std::map<CommandItem, double>> cmds) {
    if (!listen_.internal) {
      std::cout << "Internal listening mode set to false.  Use "
                   "ListenInternalCommand(true) to re-enable internal Command."
                << std::endl;
      return;
    }
    EmplaceCommands(cmds);
  }

  void CommandAll(const std::map<CommandItem, double> cmd) {
    if (!listen_.internal) {
      std::cout << "Internal listening mode set to false.  Use "
                   "ListenInternalCommand(true) to re-enable internal Command."
                << std::endl;
      return;
    }
    std::map<int, std::map<CommandItem, double>> cmds;
    for (const auto id : ids_) {
      cmds[id] = cmd;
    }
    EmplaceCommands(cmds);
  }

  void GetReplyAll(char* const output, size_t size) {
    InternalReplySender(output, size);
  }

  void Fix(const std::set<int> ids) {
    /* Ignores whether internal listening mode is true or false */
    std::map<int, std::map<CommandItem, double>> cmds;
    for (const auto id : ids) {
      const auto& servo = Utils::SafeAt(servos_, id);
      if (servo) {
        cmds[id][CommandItem::position] = NaN;
        cmds[id][CommandItem::velocity] = 0.0;
        cmds[id][CommandItem::feedforward_torque] = 0.0;
      }
    }
    EmplaceCommands(cmds);
  }

  void FixAll() {
    /* Ignores whether internal listening mode is true or false */
    Fix(ids_);
  }

  void Stop(const std::set<int> ids) {
    /* Ignores whether internal listening mode is true or false */
    for (const auto id : ids) {
      const auto& servo = Utils::SafeAt(servos_, id);
      if (servo) {
        servo.value()->controller_->SetStop();
      }
    }
  }

  void StopAll() {
    /* Ignores whether internal listening mode is true or false */
    Stop(ids_);
  }

  void SetBasePosition(const std::set<int> ids) {
    for (const auto id : ids) {
      const auto& servo = Utils::SafeAt(servos_, id);
      if (servo) {
        if (!servo.value()->SetBasePosition(0.0)) {
          std::cout << "Base position setting failed for ID " << id << "."
                    << std::endl;
        } else {
          std::cout << "Base position setting succeeded for ID " << id << "."
                    << std::endl;
        }
      } else {
        std::cout << "No Servo of ID " << id
                  << " found.  Base position setting failed." << std::endl;
      }
    }
  }

  void SetBasePositionAll() { SetBasePosition(ids_); }

  void SetBaseAux2Position(const std::set<int> ids) {
    for (const auto id : ids) {
      const auto& servo = Utils::SafeAt(servos_, id);
      if (servo) {
        if (!servo.value()->SetBaseAux2Position(0.0)) {
          std::cout << "Base aux2 position setting failed for ID " << id << "."
                    << std::endl;
        } else {
          std::cout << "Base aux2 position setting succeeded for ID " << id
                    << "." << std::endl;
        }
      } else {
        std::cout << "No Servo of ID " << id
                  << " found.  Base aux2 position setting failed." << std::endl;
      }
    }
  }

  void SetBaseAux2PositionAll() { SetBaseAux2Position(ids_); }

  void SetCommandPositionRelativeTo(const std::set<int> ids,
                                    CommandPositionRelativeTo new_val) {
    for (const auto id : ids) {
      const auto& servo = Utils::SafeAt(servos_, id);
      if (servo) {
        servo.value()->SetCommandPositionRelativeTo(new_val);
      }
    }
  }

  void SetCmdPosRelToAll(CommandPositionRelativeTo new_val) {
    SetCommandPositionRelativeTo(ids_, new_val);
  }

  void SetReplyPositionRelativeTo(const std::set<int> ids,
                                  ReplyPositionRelativeTo new_val) {
    for (const auto id : ids) {
      const auto& servo = Utils::SafeAt(servos_, id);
      if (servo) {
        servo.value()->SetReplyPositionRelativeTo(new_val);
      }
    }
  }

  void SetReplyPositionRelativeToAll(ReplyPositionRelativeTo new_val) {
    SetReplyPositionRelativeTo(ids_, new_val);
  }

  void SetReplyAux2PositionRelativeTo(const std::set<int> ids,
                                      ReplyPositionRelativeTo new_val) {
    for (const auto id : ids) {
      const auto& servo = Utils::SafeAt(servos_, id);
      if (servo) {
        servo.value()->SetReplyAux2PositionRelativeTo(new_val);
      }
    }
  }

  void SetReplyAux2PositionRelativeToAll(ReplyPositionRelativeTo new_val) {
    SetReplyAux2PositionRelativeTo(ids_, new_val);
  }

  void RegisterThread(std::function<void(ServoSystem*, std::atomic_bool)> task,
                      const std::vector<std::string>& aliases) {
    LoopingThreadManager{this, task, aliases};
  }

  void StartThread(const std::string& alias) {
    const auto& maybe_thread_manager = Utils::SafeAt(threads_mgr_, alias);
    if (maybe_thread_manager) {
      maybe_thread_manager.value()->Start();
    } else {
      std::cout << "Cannot find thread with alias " << alias << "."
                << std::endl;
    }
  }

  void StartThreadAll() {
    std::set<LoopingThreadManager*> threads;
    for (const auto& pair : threads_mgr_) {
      threads.insert(pair.second);
    }
    for (const auto& thread : threads) {
      StartThread(thread);
    }
  }

  void TerminateThread(const std::string& alias) {
    const auto& maybe_thread_manager = Utils::SafeAt(threads_mgr_, alias);
    if (maybe_thread_manager) {
      maybe_thread_manager.value()->Terminate();
    } else {
      std::cout << "Cannot find thread with alias " << alias << "."
                << std::endl;
    }
  }

  void TerminateThreadAll() {
    std::set<LoopingThreadManager*> threads;
    for (const auto& pair : threads_mgr_) {
      threads.insert(pair.second);
    }
    for (const auto& thread : threads) {
      TerminateThread(thread);
    }
  }

  void PrintThreadAll() {
    std::set<LoopingThreadManager*> threads;
    for (const auto& pair : threads_mgr_) {
      threads.insert(pair.second);
    }
    int i = 1;
    for (const auto& thread : threads) {
      std::cout << "Thread #" << i++ << ":" << std::endl;
      std::cout << "    Aliases: ";
      for (const auto& alias : thread->aliases_) {
        std::cout << alias << " || ";
      }
      std::cout << std::endl
                << "    Is Running?: "
                << (thread->IsRunning() ? "true" : "false") << std::endl;
    }
  }

  ///           Commands you can send to the Servos.          ///
  ///////////////////////////////////////////////////////////////

 protected:
  std::set<int> ids_;
  std::map<int, std::shared_ptr<Servo>> servos_;  // ID -> Servo
  std::shared_ptr<moteus::Transport> transport_;
  struct {
    bool internal = true;
    bool external = true;
  } listen_;
  const unsigned int cycle_period_us_ =
      static_cast<unsigned int>(1.0 / 100.0 * 1e6);
  std::mutex servo_access_mutex_;
  std::map<std::string, LoopingThreadManager*> threads_mgr_;
  LoopingThreadManager executor_;
  // LoopingThreadManager ext_cmd_mgr_;
  // LoopingThreadManager ext_rpl_mgr_;

  virtual void InternalReplySender(char* const output, size_t size) {
    char* current = output;
    int written;
    for (const auto& id_servo : servos_) {
      const auto id = id_servo.first;
      const auto& servo = id_servo.second;
      moteus::Query::Result usr_rpl;
      bool updated;

      {
        std::lock_guard<std::mutex> lock(servo_access_mutex_);
        usr_rpl = servo->usr_rpl();
        updated = servo->updated_last_cycle_;
      }

      if (updated) {
        written =
            ::snprintf(current, size,
                       "id=%2d | mode=%2d | "
                       "pos=%3.3f | aux2_pos=%3.3f | vel=%3.3f | trq=%3.3f | "
                       "q=%3.3f | d=%3.3f | volt=%3.2f | power=%3.3f | "
                       "motor_temp=%3.3f | temp=%2.1f | "
                       "trj=%s | home=%2d | "
                       "fault=%2d\n",
                       id, static_cast<int>(usr_rpl.mode),  //
                       usr_rpl.position, usr_rpl.abs_position, usr_rpl.velocity,
                       usr_rpl.torque,  //
                       usr_rpl.q_current, usr_rpl.d_current, usr_rpl.voltage,
                       usr_rpl.power,                                   //
                       usr_rpl.motor_temperature, usr_rpl.temperature,  //
                       usr_rpl.trajectory_complete ? "true" : "false",
                       static_cast<int>(usr_rpl.home_state),  //
                       usr_rpl.fault);
      } else {
        written =
            ::snprintf(current, size, "id=%2d | NOT UPDATED LAST CYCLE\n", id);
      }

      if (written < 0) {
        std::cout << "Error occured during printing to designated pointer."
                  << std::endl;
        return;
      } else if (written >= size) {
        std::cout << "Insufficient size to accommodate Replies." << std::endl;
        return;
      }

      current += written;
      size -= written;
    }
  }

  void EmplaceCommands(
      const std::map<int, std::map<CommandItem, double>>& cmds) {
    std::cout << "Parsing Commands..." << std::endl;
    for (const auto& id_cmd : cmds) {
      const auto id = id_cmd.first;
      const auto& cmd = id_cmd.second;
      const auto& maybe_servo = Utils::SafeAt(servos_, id);
      if (!maybe_servo) {
        std::cout << "No Servo of ID " << id
                  << " found on the Servo list.  Ignoring Command for this ID."
                  << std::endl;
        continue;
      }
      const auto& servo = maybe_servo.value();

      std::cout << "Parsing command for Servo ID " << id << "..." << std::endl;
      for (const auto& item_val : cmd) {
        const auto& item = item_val.first;
        auto val = item_val.second;
        std::cout << "Parsing command item " << item << "..." << std::endl;

        {
          std::lock_guard<std::mutex> lock(servo_access_mutex_);

          if (*CommandItemsManager::ItemToPtr(
                  item, servo->controller_->options().position_format) ==
              moteus::Resolution::kIgnore) {
            std::cout << "ServoSystem command resolution is set to " << item
                      << " = " << "kIgnore.  Ignoring command." << std::endl;
            continue;
          }

          *CommandItemsManager::ItemToPtr(item, servo->usr_cmd_) = val;
        }
        std::cout << "Successfully set command " << item << " = " << val
                  << " for Servo ID " << id << "." << std::endl;
      }
    }
  }

 private:
  void ExecuteCommands(std::atomic_bool* terminated) {
    std::cout << "Executor thread is running..." << std::endl;

    if (servos_.empty()) {
      std::cout << "No Servos found.  Executor thread terminating."
                << std::endl;
      return;
    }

    while (!terminated->load()) {
      ::usleep(cycle_period_us_);

      std::vector<moteus::CanFdFrame> cmd_frames;
      {
        std::lock_guard<std::mutex> lock(servo_access_mutex_);
        for (auto& id_servo : servos_) {
          auto id = id_servo.first;
          auto& servo = id_servo.second;
          const auto sys_cmd = servo->sys_cmd();
          cmd_frames.push_back(servo->controller_->MakePosition(sys_cmd));
          servo->updated_last_cycle_ = false;
        }

        std::vector<moteus::CanFdFrame> reply_frames;
        transport_->BlockingCycle(&cmd_frames[0], cmd_frames.size(),
                                  &reply_frames);

        for (const auto& frame : reply_frames) {
          int id = frame.source;
          const auto& maybe_servo = Utils::SafeAt(servos_, id);
          if (maybe_servo) {
            auto& servo = maybe_servo.value();
            servo->SetReply(moteus::Query::Parse(frame.data, frame.size));
            servo->updated_last_cycle_ = true;
          }
        }
      }
    }
  }

  void StartThread(LoopingThreadManager* thread) {
    if (thread) thread->Start();
  }

  void TerminateThread(LoopingThreadManager* thread) {
    if (thread) thread->Terminate();
  }
};

}  // namespace som
