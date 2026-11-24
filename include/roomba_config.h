#pragma once
// ============================================================================
//  Roomba cuadrícula — Propuesta C (sin Raspberry Pi)
//  ESP32 DevKit V1 + L298N + 2 motorreductores con encoder de cuadratura + BNO055.
//  Pines, constantes físicas y valores por defecto.
//  Lo marcado con (NVS) se puede cambiar desde la interfaz web sin recompilar.
//  Sin monitor de batería ni zumbador (GPIO 32 queda libre).
// ============================================================================
#include <stdint.h>

// ---------------------------------------------------------------- Motores (L298N)
// Quita los puentes (jumpers) ENA y ENB del módulo: el PWM entra por esos pines.
constexpr int PIN_L_EN = 25;   // ENA, PWM motor izquierdo
constexpr int PIN_L_IN1 = 26;  // IN1
constexpr int PIN_L_IN2 = 27;  // IN2
constexpr int PIN_R_EN = 23;   // ENB, PWM motor derecho
constexpr int PIN_R_IN1 = 16;  // IN3 (libre en módulos WROOM; en WROVER lo usa la PSRAM)
constexpr int PIN_R_IN2 = 17;  // IN4

constexpr uint32_t PWM_FREQ_HZ = 2000;  // 2 kHz: el L298N (bipolar) conmuta lento
constexpr uint8_t PWM_BITS = 10;        // 0..1023
constexpr uint8_t LEDC_CH_L = 0, LEDC_CH_R = 1;

// Límite de voltaje en el motor (motores de 6 V). Sin medir la batería se supone la peor
// condición: 2S recién cargada (8.4 V) menos la caída típica del L298N (~2 V).
//   duty máx = MOTOR_V_MAX / (BATT_V_FULL − L298N_DROP_V) = 6 / 6.4 ≈ 0.94
constexpr float MOTOR_V_MAX = 6.0f;
constexpr float BATT_V_FULL = 8.4f;
constexpr float L298N_DROP_V = 2.0f;
constexpr float MOTOR_MAX_DUTY = MOTOR_V_MAX / (BATT_V_FULL - L298N_DROP_V);

// Si un motor avanza al revés, cambia su INVERT en vez de recablear.
constexpr bool LEFT_MOTOR_INVERT = false;
constexpr bool RIGHT_MOTOR_INVERT = false;

// ---------------------------------------------------------------- Encoders de cuadratura
constexpr int PIN_ENC_L_A = 34;  // solo entrada, sin pull-up interno: 4.7 kΩ a 3.3 V por fuera
constexpr int PIN_ENC_L_B = 35;
constexpr int PIN_ENC_R_A = 18;  // pull-up externo de 4.7 kΩ a 3.3 V
constexpr int PIN_ENC_R_B = 19;
// Si al avanzar una cuenta baja (se ve en Telemetría), cambia su INVERT o intercambia A y B.
constexpr bool LEFT_ENC_INVERT = false;
constexpr bool RIGHT_ENC_INVERT = false;
// Ejemplo de la propuesta: 11 pulsos/vuelta del motor × reducción 34 × 4 flancos = 1496.
// Confírmalo con la hoja de datos de tu motor y con 10 vueltas marcadas a mano.
constexpr float CUENTAS_POR_VUELTA = 1496.0f;  // (NVS)
constexpr uint16_t ENC_FILTER_APB = 500;       // filtro PCNT: 500 ciclos de 80 MHz = 6.25 µs
constexpr uint32_t ENC_STILL_MS = 100;         // sin cuentas en 100 ms → rueda quieta

// ---------------------------------------------------------------- Geometría
constexpr float WHEEL_RADIUS_M = 0.0325f;  // (NVS) rueda de 65 mm
constexpr float WHEEL_BASE_M = 0.200f;     // (NVS) ¡MÍDELA! distancia entre los centros de las llantas
constexpr float CELL_SIZE_M = 0.50f;

// ---------------------------------------------------------------- BNO055 (de la Propuesta B)
constexpr int PIN_SDA = 21;
constexpr int PIN_SCL = 22;
// Reinicio por hardware si el bus se bloquea (activo en bajo). En la B era el 23, que en
// la C es ENB del L298N; aquí va al 33 (el pin INT de la MPU6050 en la A). −1 = sin cable.
constexpr int PIN_BNO_RST = 33;
constexpr uint32_t I2C_FREQ = 100000;     // 100 kHz por el clock stretching del BNO055
constexpr uint16_t I2C_TIMEOUT_MS = 50;
// Tu módulo responde en 0x29 (ADR en alto). Si el tuyo dice 0x28, cámbialo aquí.
constexpr uint8_t IMU_I2C_ADDRESS = 0x29;
constexpr bool IMU_USE_EXT_CRYSTAL = true;
constexpr uint8_t IMU_BEGIN_RETRIES = 5;      // begin() falla al primer intento: reintentar
constexpr uint16_t IMU_BOOT_WAIT_MS = 800;    // el BNO055 tarda ~650 ms en arrancar
// El BNO055 da el rumbo en sentido horario; el robot usa antihorario positivo.
// Si al girar a la izquierda θ baja (chip de cabeza), pon +1.
constexpr float IMU_YAW_SIGN = -1.0f;
constexpr float GYRO_BETA = 0.95f;  // (NVS) peso del BNO055 en el filtro complementario (1 = solo IMU)

// ---------------------------------------------------------------- Interfaz física
constexpr int PIN_BUTTON = 4;  // paro / inicio, INPUT_PULLUP
constexpr int PIN_LED = 2;     // LED de la placa

// ---------------------------------------------------------------- Valores por defecto (NVS)
struct DefaultGains {
  // Velocidad por rueda: duty = signo·(PWM_MIN + KFF·|v|) + Kp·e + Ki·∫e
  // Punto de partida para un JGA25-370 de 6 V; ajústalo con la prueba de PWM de la web.
  static constexpr float WHEEL_KP = 1.0f;    // duty por (m/s)
  static constexpr float WHEEL_KI = 5.0f;    // duty por (m/s·s)
  static constexpr float WHEEL_KFF = 1.5f;   // duty por (m/s)
  static constexpr float WHEEL_PWM_MIN = 0.25f;  // el L298N pierde ~2 V: hace falta más PWM para arrancar
  // Avance: ω_ref = Kθ·eθ + Ky·ey
  static constexpr float HEADING_K = 4.0f;   // (rad/s)/rad
  static constexpr float LATERAL_K = 6.0f;   // (rad/s)/m
  // Giro en sitio: PD sobre el rumbo
  static constexpr float TURN_KP = 5.0f;     // (rad/s)/rad
  static constexpr float TURN_KD = 0.25f;
};

constexpr float V_MAX = 0.25f;            // m/s (NVS)
constexpr float ACCEL = 0.50f;            // m/s² (NVS)
constexpr float V_APPROACH = 0.08f;       // m/s, últimos 10 cm (NVS)
constexpr float APPROACH_DIST = 0.10f;    // m
constexpr float TURN_W_MAX_DEG = 90.0f;   // °/s (NVS)
constexpr float TURN_ALPHA_DEG = 180.0f;  // °/s² (NVS)
constexpr float TURN_W_MIN_DEG = 12.0f;   // °/s mínimos mientras falte giro
constexpr float TURN_TOL_DEG = 1.5f;      // |eθ| < 1.5°
constexpr float TURN_STILL_DEG_S = 5.0f;  // |ω| < 5 °/s
constexpr uint16_t TURN_SETTLE_MS = 100;  // ...durante 100 ms
constexpr float FORWARD_STOP_M = 0.005f;  // distancia restante < 5 mm
constexpr uint16_t PRIMITIVE_PAUSE_MS = 150;  // pausa con freno entre primitivas
constexpr float SLIP_LIMIT_DEG = 3.0f;    // encoder vs BNO055 durante un avance
constexpr float TURN_COST = 0.5f;         // c_g del planificador (NVS)

// Atasco: PWM alto y sin cuentas del encoder durante STALL_TIME_MS
constexpr float STALL_MIN_DUTY = 0.35f;
constexpr uint16_t STALL_TIME_MS = 500;
constexpr uint16_t HEARTBEAT_TIMEOUT_MS = 2000;  // latido del navegador (NVS: activar/desactivar)

// ---------------------------------------------------------------- Wi-Fi
constexpr const char *AP_SSID = "Roomba-UAMM-C";
constexpr const char *AP_PASSWORD = "roomba-grid";  // WPA2: mínimo 8 caracteres. ¡Cámbiala!
constexpr const char *MDNS_NAME = "roomba-c";       // roomba-c.local en modo estación
constexpr uint32_t STA_CONNECT_TIMEOUT_MS = 15000;

// ---------------------------------------------------------------- Tareas (FreeRTOS)
constexpr uint32_t CONTROL_PERIOD_MS = 10;    // 100 Hz, núcleo 1
constexpr uint32_t SAFETY_PERIOD_MS = 20;     // núcleo 1
constexpr uint32_t MISSION_PERIOD_MS = 50;    // núcleo 1
constexpr uint32_t TELEMETRY_PERIOD_MS = 100; // núcleo 0
