#pragma once
// Pose (x, y, θ) por odometría diferencial fusionada con el rumbo del BNO055.
//
//   Δs = (Δs_R + Δs_L)/2      Δθ_enc = (Δs_R − Δs_L)/B
//   Δθ = β·Δθ_IMU + (1 − β)·Δθ_enc          (filtro complementario, β ≈ 0.95)
// Δθ_IMU es el cambio del rumbo fusionado del BNO055 en el ciclo (ya sin sesgo: el
// BNO055 calibra su giroscopio solo). β = 1 equivale a la Propuesta B (solo IMU).
//   x += Δs·cos(θ + Δθ/2)     y += Δs·sin(θ + Δθ/2)     θ += Δθ
//
// El giroscopio domina a corto plazo (no le afecta el deslizamiento); el encoder aporta
// una referencia. Con las ruedas quietas no se integra nada (evita la deriva en reposo).
// La diferencia acumulada encoder − giroscopio se usa para detectar deslizamiento.
#include <math.h>
#include <stdint.h>

class Pose {
 public:
  float x = 0, y = 0, theta = 0;  // m, m, rad (continuo, sin envolver)

  void reset(float x0, float y0, float th0) {
    x = x0;
    y = y0;
    theta = th0;
    resetSlip();
  }

  // dnL, dnR: cuentas con signo de este ciclo. dThImu: Δθ del BNO055 en el ciclo [rad].
  // gyroOk = false → solo encoders (β = 0). still = ruedas quietas → Δθ = 0.
  void update(int32_t dnL, int32_t dnR, float dThImu, bool gyroOk, bool still, float dt,
              float metersPerTick, float wheelBase, float beta) {
    float dsL = dnL * metersPerTick;
    float dsR = dnR * metersPerTick;
    float ds = 0.5f * (dsL + dsR);
    float dThEnc = (dsR - dsL) / wheelBase;
    float dThGyro = dThImu;
    float dTh = gyroOk ? beta * dThGyro + (1.0f - beta) * dThEnc : dThEnc;
    if (still) dTh = 0;

    x += ds * cosf(theta + 0.5f * dTh);
    y += ds * sinf(theta + 0.5f * dTh);
    theta += dTh;
    distance += fabsf(ds);
    lastRate = dt > 0 ? dTh / dt : 0;

    if (slipWatch && gyroOk) {
      slipEnc += dThEnc;
      slipImu += dThGyro;
    }
  }

  // Diferencia acumulada encoder − IMU desde resetSlip() [rad]
  float slipError() const { return slipEnc - slipImu; }
  void resetSlip() { slipEnc = slipImu = 0; }

  bool slipWatch = false;
  uint32_t slipEvents = 0;
  float distance = 0;  // odómetro total [m]
  float lastRate = 0;  // ω fusionada del último ciclo [rad/s]

 private:
  float slipEnc = 0, slipImu = 0;
};

// Utilidades de ángulos
inline float wrapPi(float a) {
  while (a > (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}
