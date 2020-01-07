#include "esphome.h"

class RolloTraits {
  public:
    RolloTraits() = default;
};

class Rollo;

class OneSecondTick: public PollingComponent {
  public:
    typedef std::function<void()> UpdateAction;
  private:
    UpdateAction updateAction;
  public:
    OneSecondTick(UpdateAction updateAction): PollingComponent(1000), updateAction(updateAction) {
    }
    void update() {
      (this->updateAction)();
    }
};

class Rollo : public MQTTComponent, public Nameable {
 private:
   OneSecondTick oneSecondTick;
   Switch *openSwitch;
   Switch *closeSwitch;
   std::string action;
   std::string running;
   uint8_t percent;
   uint8_t driveTime;
   optional<uint8_t> driveActualTime;
 public:
  /// Construct this LightState using the provided traits and name.
  Rollo(): 
    oneSecondTick(std::bind(&Rollo::update, this)),
    percent(0), 
    driveTime(20) {
  }

  void set_open_switch(Switch *sc) {
    openSwitch = sc;
  }
  void set_close_switch(Switch *sc) {
    closeSwitch = sc;
  }

  void send_discovery(JsonObject &root, SendDiscoveryConfig &config) override {
  };

  bool send_initial_state() override {
    return true;
  }
  bool is_internal() override {
    return false;
  }
  std::string component_type() const override {
    return "rollo";
  }
  
  std::string friendly_name() const override {
    return get_name();
  }

  void update() {
    if (driveActualTime > 0) {
      --(*driveActualTime);
      publish(action, driveActualTime);
    }
    if (driveActualTime == 0) {
      driveActualTime.reset();
      publish("IDLE", driveActualTime);
    }
  }

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  /// Load state from preferences
  void setup() {
    oneSecondTick.call_setup();
    subscribe_json(get_command_topic_(), [=](const std::string &str, JsonObject &state) {
        auto odriveTime = state["driveTime"];
        if (odriveTime.success()) {
          driveTime = odriveTime.as<int>();
        }
        auto oaction = state["action"];
        if (oaction.success()) {
          if (driveActualTime.has_value()) {
            action = "STOP";
          } else {
            action = oaction.as<const char *>();
          }
          if (action == "UP") {
            driveActualTime = driveTime; 
          } else if (action == "DOWN") {
            driveActualTime = driveTime; 
          } else if (action == "STOP" || action == "IDLE") {
            action = "IDLE";
            driveActualTime.reset();
          }
        }
        publish(action, driveActualTime);
    });
    publish("IDLE", driveActualTime);
  }

  void publish(const std::string &action, optional<int> driveActualTime) {
    if (action == "UP") {
      closeSwitch->turn_off(); 
      openSwitch->turn_on(); 
    } else if (action == "DOWN") {
      openSwitch->turn_off(); 
      closeSwitch->turn_on(); 
    } else if (action == "IDLE") {
      openSwitch->turn_off(); 
      closeSwitch->turn_off(); 
    }
    publish_json(get_state_topic_(), [=](JsonObject &state) {
      state["action"] = action;
      if (driveActualTime.has_value()) {
        state["driveActualTime"] = *driveActualTime; 
      }
      state["driveTime"] = driveTime;
    });
  }

  void dump_config() {
  }
  void loop() {
    // oneSecondTick.call_loop();
  }
  
  /// Publish the currently active state to the frontend.
  void publish_state() {
  }


  /*
   *
   * This is different from add_new_current_values_callback in that it only sends events for start
   * and end values. For example, with transitions it will only send a single callback whereas
   * the callback passed in add_new_current_values_callback will be called every loop() cycle when
   * a transition is active
   *
   * Note the callback should get the output values through get_remote_values().
   *
   * @param send_callback The callback.
  void add_new_remote_values_callback(std::function<void()> &&send_callback) {
  }
  */

#ifdef USE_JSON
  /// Dump the state of this light as JSON.
  void dump_json(JsonObject &root) {
  }
#endif

 protected:
  uint32_t hash_base() override {
    return (uint32_t)this;
  }

  /** Callback to call when new values for the frontend are available.
   *
   * "Remote values" are light color values that are reported to the frontend and have a lower
   * publish frequency than the "real" color values. For example, during transitions the current
   * color value may change continuously, but the remote values will be reported as the target values
   * starting with the beginning of the transition.
   */
  // CallbackManager<void()> remote_values_callback_{};
};

