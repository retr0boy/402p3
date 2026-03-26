#include <pigpio.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <atomic>

// ---------------- Pin mapping (BCM) ----------------
static constexpr int GPIO_PWM = 12; // M1D2 (Active-High Disable)
static constexpr int GPIO_IN1 = 24; // M1IN1 (Direction 1)
static constexpr int GPIO_IN2 = 23; // M1IN2 (Direction 2)
static constexpr int GPIO_EN  = 22; // EN (Enable Chip)
static constexpr int GPIO_SF  = 5;  // Status Flag (Active-Low)

static constexpr int ENC_A    = 17; // Encoder Channel A
static constexpr int ENC_B    = 27; // Encoder Channel B

// ---------------- Quadrature Encoder Class ----------------
class QuadratureEncoder {
public:
    QuadratureEncoder(int gpioA, int gpioB, double cpr = 3916.8)
        : gpioA_(gpioA), gpioB_(gpioB), cpr_eff_(cpr) {}

    void init() {
        gpioSetMode(gpioA_, PI_INPUT);
        gpioSetMode(gpioB_, PI_INPUT);
        gpioSetPullUpDown(gpioA_, PI_PUD_UP);
        gpioSetPullUpDown(gpioB_, PI_PUD_UP);
        gpioGlitchFilter(gpioA_, 100); // Increased filter for PWM noise
        gpioGlitchFilter(gpioB_, 100);
        
        last_state_ = (gpioRead(gpioA_) << 1) | gpioRead(gpioB_);
        gpioSetAlertFuncEx(gpioA_, alertTrampoline, this);
        gpioSetAlertFuncEx(gpioB_, alertTrampoline, this);
    }

    int32_t getCount() const { return count_.load(); }
    double getRadians() const { 
        return (2.0 * M_PI * static_cast<double>(getCount())) / cpr_eff_; 
    }

private:
    static void alertTrampoline(int gpio, int level, uint32_t tick, void* user) {
        auto* self = static_cast<QuadratureEncoder*>(user);
        int a = gpioRead(self->gpioA_);
        int b = gpioRead(self->gpioB_);
        int new_state = (a << 1) | b;
        int old_state = self->last_state_.exchange(new_state);
        static const int8_t delta[16] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
        self->count_.fetch_add(delta[(old_state << 2) | new_state]);
    }
    int gpioA_, gpioB_;
    double cpr_eff_;
    std::atomic<int32_t> count_{0};
    std::atomic<int> last_state_{0};
};

// ---------------- Motor driver wrapper ----------------
class MotorMC33926 {
public:
    explicit MotorMC33926(unsigned pwm_freq_hz) : pwm_freq_hz_(pwm_freq_hz) {}

    bool init() {
        if (gpioInitialise() < 0) return false;
        gpioSetMode(GPIO_IN1, PI_OUTPUT);
        gpioSetMode(GPIO_IN2, PI_OUTPUT);
        gpioSetMode(GPIO_EN,  PI_OUTPUT);
        gpioSetMode(GPIO_PWM, PI_OUTPUT);
        gpioSetMode(GPIO_SF, PI_INPUT);
        gpioSetPullUpDown(GPIO_SF, PI_PUD_UP);
        setDuty(0.0);
        return true;
    }

    void enable()  { gpioWrite(GPIO_EN, 1); }
    void disable() { gpioWrite(GPIO_EN, 0); }

    void setDirection(bool forward) {
        gpioWrite(GPIO_IN1, forward ? 1 : 0);
        gpioWrite(GPIO_IN2, forward ? 0 : 1);
    }

    void setDuty(double duty) {
        duty = std::max(0.0, std::min(1.0, duty));
        double inverted_duty = 1.0 - duty; // M1D2 Active-High Disable logic
        unsigned duty_u = static_cast<unsigned>(std::lround(inverted_duty * 1'000'000.0));
        gpioHardwarePWM(GPIO_PWM, pwm_freq_hz_, duty_u);
    }

    bool faultActive() const { return gpioRead(GPIO_SF) == 0; }
    void shutdown() { setDuty(0.0); disable(); gpioTerminate(); }

private:
    unsigned pwm_freq_hz_;
};

// ---------------- CLI helpers ----------------
static bool hasArg(int argc, char** argv, const std::string& k) {
    for (int i = 1; i < argc; ++i) if (k == argv[i]) return true;
    return false;
}

static std::string getArg(int argc, char** argv, const std::string& k, const std::string& def) {
    for (int i = 1; i + 1 < argc; ++i) if (k == argv[i]) return argv[i + 1];
    return def;
}

// ---------------- Main ----------------
int main(int argc, char** argv) {
    // Parse arguments
    const unsigned freq   = static_cast<unsigned>(std::stoul(getArg(argc, argv, "--freq", "10000")));
    const double duty     = std::stod(getArg(argc, argv, "--duty", "0.2"));
    const double run_time = std::stod(getArg(argc, argv, "--time", "3.0"));
    const bool reverse    = hasArg(argc, argv, "--reverse");
    const bool ramp       = hasArg(argc, argv, "--ramp");
    const double ramp_to  = std::stod(getArg(argc, argv, "--to", "0.6"));

    MotorMC33926 motor(freq);
    QuadratureEncoder enc(ENC_A, ENC_B);

    if (!motor.init()) return 1;
    enc.init();
    
    if (motor.faultActive()) {
        std::cerr << "FAULT detected! Check power.\n";
        motor.shutdown(); return 2;
    }

    motor.setDirection(!reverse);
    motor.enable();

    std::cout << "Starting Test: " << (ramp ? "RAMP" : "STEADY") 
              << " | Freq: " << freq << "Hz | Dir: " << (reverse ? "REV" : "FWD") << "\n";

    try {
        if (ramp) {
            const int steps = 100;
            for (int i = 0; i <= steps; ++i) {
                if (motor.faultActive()) break;
                double d = (static_cast<double>(i) / steps) * ramp_to;
                motor.setDuty(d);
                std::cout << "Duty: " << d << " | Counts: " << enc.getCount() << "\r" << std::flush;
                std::this_thread::sleep_for(std::chrono::duration<double>(run_time / steps));
            }
        } else {
            motor.setDuty(duty);
            auto start = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - start < std::chrono::duration<double>(run_time)) {
                if (motor.faultActive()) break;
                std::cout << "Duty: " << duty << " | Counts: " << enc.getCount() << " | Rad: " << enc.getRadians() << "\r" << std::flush;
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
    } catch (...) {}

    std::cout << "\nTest Complete. Counts: " << enc.getCount() << "\n";
    motor.shutdown();
    return 0;
}
