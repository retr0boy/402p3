#include <pigpio.h>
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <algorithm>
#include <iomanip>

// --- Настройки пинов мотора ---
static constexpr int GPIO_PWM = 12; // M1D2 (Active-Low: 0 = 100% мощности, 1 = 0% мощности)
static constexpr int GPIO_IN1 = 24; // M1IN1
static constexpr int GPIO_IN2 = 23; // M1IN2
static constexpr int GPIO_EN  = 22; // EN
static constexpr int ENC_A    = 17; // Encoder A
static constexpr int ENC_B    = 27; // Encoder B

// --- Настройки I2C (AltIMU-10 v5 / LSM6DS33) ---
static constexpr int I2C_BUS = 1;
static constexpr int LSM6DS33_ADDR = 0x6B; // Адрес по умолчанию. Если ошибка, поменяй на 0x6A
static constexpr int CTRL1_XL = 0x10;      // Регистр настройки акселерометра
static constexpr int OUTX_L_XL = 0x28;     // Начальный регистр данных (X, Y, Z)

// --- Константы системы ---
static constexpr double GEAR_RATIO = 20.4; 
static constexpr double CPR = 48.0;        
static constexpr double TOTAL_CPR = GEAR_RATIO * CPR; 
static constexpr double RAD_TO_DEG = 180.0 / M_PI;
static constexpr double DT = 1.0 / 200.0; // 200 Hz (5 мс)

// --- Настройки фильтра EMA ---
static constexpr double ALPHA = 0.1; // Коэффициент сглаживания данных сенсора

volatile long encoder_counts = 0;

// Обработчик прерываний энкодера
void encoder_isr(int gpio, int level, uint32_t tick) {
    int a = gpioRead(ENC_A);
    int b = gpioRead(ENC_B);
    if (gpio == ENC_A) {
        (a != b) ? encoder_counts++ : encoder_counts--;
    } else {
        (a == b) ? encoder_counts++ : encoder_counts--;
    }
}

// Расчет угла вокруг оси Y (Pitch/Тангаж) по акселерометру
double get_pitch_from_accel(double ax, double az) {
    return atan2(ax, az); 
}

int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "Ошибка инициализации pigpio! (Запусти через sudo?)" << std::endl;
        return 1;
    }

    // --- Инициализация I2C ---
    int i2c_handle = i2cOpen(I2C_BUS, LSM6DS33_ADDR, 0);
    if (i2c_handle < 0) {
        std::cerr << "Ошибка открытия I2C. Проверь подключение и адрес!" << std::endl;
        gpioTerminate();
        return 1;
    }

    // Включаем акселерометр: 208 Гц (0x50), ±2g
    i2cWriteByteData(i2c_handle, CTRL1_XL, 0x50);

    // --- Настройка GPIO ---
    gpioSetMode(GPIO_IN1, PI_OUTPUT);
    gpioSetMode(GPIO_IN2, PI_OUTPUT);
    gpioSetMode(GPIO_EN,  PI_OUTPUT);
    gpioSetMode(GPIO_PWM, PI_OUTPUT);
    gpioSetMode(ENC_A, PI_INPUT);
    gpioSetMode(ENC_B, PI_INPUT);

    gpioSetAlertFunc(ENC_A, encoder_isr);
    gpioSetAlertFunc(ENC_B, encoder_isr);

    gpioWrite(GPIO_EN, 1); // Включаем драйвер

    // --- Переменные ПИД-регулятора ---
    double Kp = 0.30;   // П-коэффициент для зоны < 7 градусов
    double Ki = 0.00;   // И-коэффициент (интегратор) для точной доводки
    double Kd = 0.7;  // Д-коэффициент для торможения
    
    double prev_error = 0.0;
    double integral = 0.0;
    
    // Переменные трекинга
    double cumulative_ref = 0.0; // Стартуем ровно с нуля
    double prev_raw_angle = 0.0;
    
    double filtered_ax = 0;
    double filtered_az = 0;

    // ==========================================
    // ЭТАП КАЛИБРОВКИ И ПРОГРЕВА ФИЛЬТРА
    // ==========================================
    std::cout << "Calibrating and zeroing IMU... Do not move!" << std::endl;
    char buf[6];
    
    // Первое чтение для инициализации фильтра
    if (i2cReadI2CBlockData(i2c_handle, OUTX_L_XL, buf, 6) == 6) {
        filtered_ax = (double)((int16_t)(((uint8_t)buf[1] << 8) | (uint8_t)buf[0]));
        filtered_az = (double)((int16_t)(((uint8_t)buf[5] << 8) | (uint8_t)buf[4]));
    }

    // Читаем 100 раз для стабилизации EMA-фильтра (~0.5 сек)
    for (int i = 0; i < 100; ++i) {
        if (i2cReadI2CBlockData(i2c_handle, OUTX_L_XL, buf, 6) == 6) {
            double ax_raw = (double)((int16_t)(((uint8_t)buf[1] << 8) | (uint8_t)buf[0]));
            double az_raw = (double)((int16_t)(((uint8_t)buf[5] << 8) | (uint8_t)buf[4]));
            
            filtered_ax = ALPHA * ax_raw + (1.0 - ALPHA) * filtered_ax;
            filtered_az = ALPHA * az_raw + (1.0 - ALPHA) * filtered_az;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Фиксируем нулевую точку
    prev_raw_angle = get_pitch_from_accel(filtered_ax, filtered_az);
    std::cout << "Zeroed successfully! Starting control loop." << std::endl;
    // ==========================================

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Ref_Deg\t\tMotor_Deg\tError_Deg\tU_Cmd" << std::endl;

    while (true) {
        auto start_loop = std::chrono::steady_clock::now();

        // 1. Чтение сырых данных IMU
        double ax_raw = 0, az_raw = 0;
        if (i2cReadI2CBlockData(i2c_handle, OUTX_L_XL, buf, 6) == 6) {
            ax_raw = (double)((int16_t)(((uint8_t)buf[1] << 8) | (uint8_t)buf[0]));
            az_raw = (double)((int16_t)(((uint8_t)buf[5] << 8) | (uint8_t)buf[4]));
        }

        // 2. Применяем EMA фильтр
        filtered_ax = ALPHA * ax_raw + (1.0 - ALPHA) * filtered_ax;
        filtered_az = ALPHA * az_raw + (1.0 - ALPHA) * filtered_az;

        // 3. Вычисляем текущий угол
        double current_raw_angle = get_pitch_from_accel(filtered_ax, filtered_az);

        // 4. Логика развертки с шумоподавлением
        double delta = current_raw_angle - prev_raw_angle;
        if (delta > M_PI) delta -= 2.0 * M_PI; 
        else if (delta < -M_PI) delta += 2.0 * M_PI; 
        
        // Noise Gate: игнорируем изменения меньше ~0.05 градуса (0.001 рад)
        if (std::abs(delta) > 0.001) { 
            cumulative_ref += delta;
            prev_raw_angle = current_raw_angle; 
        }

        // 5. Позиция мотора
        double theta_motor = 2.0 * M_PI * ((double)encoder_counts / TOTAL_CPR);

        // 6. Вычисление ошибки
        double error = cumulative_ref - theta_motor; 
        double error_deg = error * RAD_TO_DEG;       
        double error_dot = (error - prev_error) / DT;
        
        double u = 0.0;

        // 7. Гибридная логика управления
        if (std::abs(error_deg) > 10.0) {
            // Ошибка больше ±7 градусов: 100% мощности в нужную сторону
            u = (error > 0) ? 0.95 : -0.95; 
            integral = 0.0; // Сбрасываем интегратор
        } else {
            // Ошибка внутри ±7 градусов: ПИД-регулятор
            integral += error * DT;
            
            // Anti-windup: ограничение влияния интегратора (макс 30% мощности)
            double max_i_power = 0.3;
            if ((Ki * integral) > max_i_power) integral = max_i_power / Ki;
            else if ((Ki * integral) < -max_i_power) integral = -max_i_power / Ki;

            u = (Kp * error) + (Ki * integral) + (Kd * error_dot);
            u = -(std::max(-1.0, std::min(1.0, u))); // Общее ограничение сигнала
        }
        
        prev_error = error;

        // 8. Управление направлением
        if (u >= 0) {
            gpioWrite(GPIO_IN1, 1); gpioWrite(GPIO_IN2, 0);
        } else {
            gpioWrite(GPIO_IN1, 0); gpioWrite(GPIO_IN2, 1);
        }

        // 9. Управление ШИМ с компенсацией трения покоя (Minimum Duty)
        double duty = std::abs(u);
        
        // ПОДБЕРИ ЭТО ЗНАЧЕНИЕ: Минимальная мощность, при которой мотор страгивается с места. 
        // 0.15 означает 15% мощности.
        double MIN_POWER = 0.8; 

        if (duty < 0.01) { 
            // Deadband (Мертвая зона): Если ПИД просит меньше 1% мощности, 
            // считаем, что мы уже на месте, и полностью отключаем мотор, чтобы он не гудел.
            duty = 0.0; 
        } else {
            // Масштабируем сигнал: 0.01 -> MIN_POWER, а 1.0 -> 1.0
            duty = MIN_POWER + duty * (1.0 - MIN_POWER); 
        }
        
        // Защита от превышения (на всякий случай)
        duty = std::min(1.0, duty);

        // Инверсия для Active-Low: Если duty = 1.0 -> inverted_duty = 0.0 (100% ток на мотор)
        double inverted_duty = 1.0 - duty; 
        gpioHardwarePWM(GPIO_PWM, 12000, (unsigned)(duty * 1000000));

        // 10. Вывод в консоль
        std::cout << (cumulative_ref * RAD_TO_DEG) << "\t\t" 
                  << (theta_motor * RAD_TO_DEG) << "\t\t"
                  << error_deg << "\t\t" 
                  << u << "   \r" << std::flush;

        // 11. Ожидание до конца цикла (5000 мкс = 200 Гц)
        std::this_thread::sleep_until(start_loop + std::chrono::microseconds(5000));
    }

    // Очистка при выходе
    i2cClose(i2c_handle);
    gpioWrite(GPIO_EN, 0);
    gpioTerminate();
    return 0;
}
