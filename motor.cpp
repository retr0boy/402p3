#include <pigpio.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

// ---------------- Pin mapping for Pololu MC33926 ----------------
static constexpr int GPIO_PWM = 12; // M1D2 (Active-High Disable)
static constexpr int GPIO_IN1 = 24; // M1IN1 (Direction 1)
static constexpr int GPIO_IN2 = 23; // M1IN2 (Direction 2)
static constexpr int GPIO_EN  = 22; // EN (Enable Chip)
static constexpr int GPIO_SF  = 5;  // M1SF (Status Flag, active-low)

// ---------------- Utilities ----------------
static double clamp01(double x) { return std::max(0.0, std::min(1.0, x)); }

// ---------------- Motor driver wrapper ----------------
class MotorMC33926 {
public:
  explicit MotorMC33926(unsigned pwm_freq_hz) : pwm_freq_hz_(pwm_freq_hz) {}

  bool init() {
    if (gpioInitialise() < 0) {
      std::cerr << "ERROR: pigpio init failed (run with sudo).\n";
      return false;
    }

    // Set Outputs
    gpioSetMode(GPIO_IN1, PI_OUTPUT);
    gpioSetMode(GPIO_IN2, PI_OUTPUT);
    gpioSetMode(GPIO_EN,  PI_OUTPUT);
    gpioSetMode(GPIO_PWM, PI_OUTPUT);

    // Set SF input with pull-up (SF is open-drain)
    gpioSetMode(GPIO_SF, PI_INPUT);
    gpioSetPullUpDown(GPIO_SF, PI_PUD_UP);

    // Safe startup defaults
    disable();
    setDuty(0.0);
    gpioWrite(GPIO_IN1, 0);
    gpioWrite(GPIO_IN2, 0);

    if (faultActive()) {
      std::cerr << "WARNING: SF is LOW at startup. Check wiring and battery power.\n";
    }

    return true;
  }

  void shutdown() {
    setDuty(0.0);
    disable();
    gpioTerminate();
  }

  void enable()  { gpioWrite(GPIO_EN, 1); }
  void disable() { gpioWrite(GPIO_EN, 0); }

  void setDirection(bool forward) {
    if (forward) {
      gpioWrite(GPIO_IN1, 1);
      gpioWrite(GPIO_IN2, 0);
    } else {
      gpioWrite(GPIO_IN1, 0);
      gpioWrite(GPIO_IN2, 1);
    }
  }

  // duty in [0..1]
  void setDuty(double duty) {
    duty = clamp01(duty);

    // Because M1D2 is an "Active-High Disable" pin, applying 3.3V stops the motor.
    // To make 100% duty cycle equal full speed, we mathematically invert the signal.
    double inverted_duty = 1.0 - duty; 
    
    unsigned duty_u = static_cast<unsigned>(std::lround(inverted_duty * 1'000'000.0));
    gpioHardwarePWM(GPIO_PWM, pwm_freq_hz_, duty_u);
  }

  bool faultActive() const {
    return gpioRead(GPIO_SF) == 0;
  }

private:
  unsigned pwm_freq_hz_;
};

// ---------------- CLI helpers ----------------
static bool hasArg(int argc, char** argv, const std::string& k) {
  for (int i = 1; i < argc; ++i) if (k == argv[i]) return true;
  return false;
}

static std::string getArg(int argc, char** argv, const std::string& k, const std::string& def) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (k == argv[i]) return argv[i + 1];
  }
  return def;
}

int main(int argc, char** argv) {
  const unsigned freq = static_cast<unsigned>(std::stoul(getArg(argc, argv, "--freq", "12000")));
  const double duty   = std::stod(getArg(argc, argv, "--duty", "0.0"));
  const bool forward  = hasArg(argc, argv, "--forward");
  const bool reverse  = hasArg(argc, argv, "--reverse");
  const bool ramp     = hasArg(argc, argv, "--ramp");
  const double ramp_from = std::stod(getArg(argc, argv, "--from", "0.0"));
  const double ramp_to   = std::stod(getArg(argc, argv, "--to",   "0.6"));
  const double ramp_time = std::stod(getArg(argc, argv, "--time", "2.0"));

  MotorMC33926 motor(freq);
  if (!motor.init()) return 1;

  std::cout << "MC33926 Motor Control Initialized\n";

  if (motor.faultActive()) {
    std::cerr << "ERROR: Driver reports FAULT. Is the motor battery turned on?\n";
    motor.shutdown();
    return 2;
  }

  const bool dir_fwd = reverse ? false : true; 
  motor.setDirection(dir_fwd);
  motor.enable();

  auto stopSafe = [&motor]() {
    motor.setDuty(0.0);
    motor.disable();
  };

  try {
    if (ramp) {
      std::cout << "Ramping duty " << ramp_from << " -> " << ramp_to
                << " over " << ramp_time << " s, dir=" << (dir_fwd ? "FWD" : "REV") << "\n";

      const int steps = 120;
      for (int i = 0; i <= steps; ++i) {
        if (motor.faultActive()) {
          std::cerr << "\nFAULT detected! Stopping.\n";
          stopSafe(); motor.shutdown(); return 3;
        }
        double d = ramp_from + (static_cast<double>(i) / steps) * (ramp_to - ramp_from);
        motor.setDuty(d);
        std::this_thread::sleep_for(std::chrono::duration<double>(ramp_time / steps));
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } else {
      std::cout << "Setting duty=" << duty << " dir=" << (dir_fwd ? "FWD" : "REV") << " for 3s...\n";
      motor.setDuty(duty);
      for (int i = 0; i < 300; ++i) {
        if (motor.faultActive()) {
          std::cerr << "\nFAULT detected! Stopping.\n";
          stopSafe(); motor.shutdown(); return 3;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }

    std::cout << "Stopping...\n";
    stopSafe();
    motor.shutdown();
    return 0;

  } catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << "\n";
    stopSafe();
    motor.shutdown();
    return 4;
  }
}
