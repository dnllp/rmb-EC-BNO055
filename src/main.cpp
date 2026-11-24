// Roomba cuadrícula — Propuesta C
// ESP32 DevKit V1 + L298N + encoders de cuadratura (PCNT) + BNO055 + interfaz web.
// Sin Raspberry Pi, sin monitor de batería y sin zumbador.
//
// Núcleo 1: control (100 Hz), seguridad (50 Hz), misión (20 Hz).
// Núcleo 0: Wi-Fi y servidor (AsyncTCP), planificador (bajo demanda), telemetría (10 Hz).
#include <Arduino.h>

#include "roomba_config.h"
#include "roomba_settings.h"
#include "L298n.h"
#include "QuadEncoder.h"
#include "Bno055.h"
#include "Pose.h"
#include "Motion.h"
#include "Safety.h"
#include "Mission.h"
#include "WebApi.h"

Settings gSettings;

static L298n motorL, motorR;
static QuadEncoder encL, encR;
static Bno055 bno;
static Pose pose;
static Motion motion;
static Safety safety;
static Mission mission;

static volatile bool planReady = false;

// ---------------------------------------------------------------- utilidades
static bool missionRunning() {
  Mission::State s = mission.state();
  Motion::State m = motion.state();
  return s == Mission::State::EJECUTANDO ||
         m == Motion::State::FORWARD || m == Motion::State::TURN || m == Motion::State::OPEN_LOOP;
}

// ---------------------------------------------------------------- tareas
static void controlTask(void *) {
  TickType_t last = xTaskGetTickCount();
  const float dt = CONTROL_PERIOD_MS / 1000.0f;
  for (;;) {
    motion.step(dt);
    vTaskDelayUntil(&last, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}

static void safetyTask(void *) {
  TickType_t last = xTaskGetTickCount();
  Safety::Event prev = Safety::Event::NONE;
  for (;;) {
    Safety::Event ev = safety.step(missionRunning());
    if (ev != Safety::Event::NONE && ev != prev) {
      const char *msg = Safety::eventMessage(ev);
      Serial.printf("[seguridad] %s\n", msg);
      if (ev == Safety::Event::HEARTBEAT_LOST) {
        // Se frena (freno activo y luego rueda libre); hay que volver a planear y ejecutar
        mission.emergencyStop();
      } else {
        motion.emergency();
        mission.fail(msg);
      }
      webapi::sendEvent(msg);
    }
    prev = ev;
    vTaskDelayUntil(&last, pdMS_TO_TICKS(SAFETY_PERIOD_MS));
  }
}

static void missionTask(void *) {
  TickType_t last = xTaskGetTickCount();
  bool btnPrev = true;
  uint32_t btnT = 0;
  Mission::State prevState = mission.state();
  for (;;) {
    mission.step();

    // Botón físico (activo en bajo): paro si está en marcha, ejecutar si hay plan, reanudar si está pausado
    bool btn = digitalRead(PIN_BUTTON);
    if (!btn && btnPrev && millis() - btnT > 300) {
      btnT = millis();
      const char *err = "";
      switch (mission.state()) {
        case Mission::State::EJECUTANDO: mission.emergencyStop(); break;
        case Mission::State::LISTO: mission.execute(true, &err); break;
        case Mission::State::PAUSADO: mission.togglePause(&err); break;
        default: motion.emergency(); break;
      }
    }
    btnPrev = btn;

    // LED: fijo = ejecutando, parpadeo lento = listo/pausado, rápido = falla
    Mission::State s = mission.state();
    uint32_t t = millis();
    bool led = false;
    switch (s) {
      case Mission::State::EJECUTANDO: led = true; break;
      case Mission::State::LISTO:
      case Mission::State::PAUSADO: led = (t / 500) & 1; break;
      case Mission::State::FALLA: led = (t / 100) & 1; break;
      default: led = (t % 2000) < 50; break;
    }
    digitalWrite(PIN_LED, led);

    if (s != prevState) {
      Serial.printf("[mision] %s -> %s (%s)\n", Mission::stateName(prevState), Mission::stateName(s),
                    mission.message());
      prevState = s;
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(MISSION_PERIOD_MS));
  }
}

static void plannerTask(void *) { mission.plannerLoop(); }

static void telemetryTask(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    if (planReady) {
      planReady = false;
      webapi::broadcastPlan();
    }
    webapi::sendTelemetry();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
  }
}

// ---------------------------------------------------------------- arranque
void setup() {
  // Primero, motores en rueda libre: EN (25 y 23) e IN1–IN4 en bajo
  for (int pin : {PIN_L_EN, PIN_L_IN1, PIN_L_IN2, PIN_R_EN, PIN_R_IN1, PIN_R_IN2}) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  Serial.begin(115200);
  delay(100);
  Serial.println("\n[roomba-grid-c] arrancando");

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  gSettings.load();

  motorL.begin({PIN_L_EN, PIN_L_IN1, PIN_L_IN2, LEDC_CH_L}, PWM_FREQ_HZ, PWM_BITS, LEFT_MOTOR_INVERT,
               MOTOR_MAX_DUTY);
  motorR.begin({PIN_R_EN, PIN_R_IN1, PIN_R_IN2, LEDC_CH_R}, PWM_FREQ_HZ, PWM_BITS, RIGHT_MOTOR_INVERT,
               MOTOR_MAX_DUTY);
  Serial.printf("[motores] L298N a %u Hz, duty maximo %.2f (%.1f V en el motor)\n", (unsigned)PWM_FREQ_HZ,
                MOTOR_MAX_DUTY, MOTOR_V_MAX);

  if (!encL.begin(PIN_ENC_L_A, PIN_ENC_L_B, LEFT_ENC_INVERT, ENC_FILTER_APB))
    Serial.println("[enc] fallo PCNT izquierdo");
  if (!encR.begin(PIN_ENC_R_A, PIN_ENC_R_B, RIGHT_ENC_INVERT, ENC_FILTER_APB))
    Serial.println("[enc] fallo PCNT derecho");

  // BNO055: espera su arranque (~800 ms) y reintenta begin(), que falla al primer intento
  Bno055::Config ic{PIN_SDA,         PIN_SCL,           PIN_BNO_RST,       I2C_FREQ,         I2C_TIMEOUT_MS,
                    IMU_I2C_ADDRESS, IMU_USE_EXT_CRYSTAL, IMU_BEGIN_RETRIES, IMU_BOOT_WAIT_MS, IMU_YAW_SIGN};
  if (bno.begin(ic)) {
    Serial.printf("[imu] BNO055 en 0x%02X, IMUPLUS, %s\n", IMU_I2C_ADDRESS,
                  bno.offsetsRestored() ? "offsets restaurados de NVS" : "sin offsets guardados (calibra desde la web)");
  } else {
    Serial.printf("[imu] BNO055 NO responde en 0x%02X (se reintenta cada 2 s con los motores parados)\n",
                  IMU_I2C_ADDRESS);
  }

  motion.begin({&motorL, &motorR, &encL, &encR, &bno, &pose});
  safety.begin(&motorL, &motorR, &encL, &encR);
  mission.begin(&motion, &pose);
  mission.onPlanReady([] { planReady = true; });

  // Tareas de tiempo real en el núcleo 1
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(safetyTask, "safety", 4096, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(missionTask, "mission", 4096, nullptr, 3, nullptr, 1);
  // Red y planificación en el núcleo 0
  xTaskCreatePinnedToCore(plannerTask, "planner", 8192, nullptr, 3, nullptr, 0);
  delay(10);  // que la tarea planner registre su handle antes de recibir pedidos

  webapi::begin({&mission, &motion, &safety, &bno, &pose});
  xTaskCreatePinnedToCore(telemetryTask, "telemetry", 6144, nullptr, 1, nullptr, 0);

  Serial.printf("[roomba-grid-c] listo: %s\n", webapi::networkInfo());
}

void loop() {
  // Todo corre en tareas; loop() solo cede el procesador.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
