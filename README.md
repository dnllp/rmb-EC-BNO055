# roomba-grid-c — Propuesta C

Robot diferencial tipo Roomba que recorre metas en una cuadrícula de 6×6 celdas de 50 cm.
ESP32 DevKit V1 + PlatformIO, puente H L298N, los mismos dos motorreductores con encoder de cuadratura
de la Propuesta A, IMU **BNO055** (la de la Propuesta B) y una página web con **tema claro** servida por la
propia ESP32 para marcar inicio, obstáculos y metas, y elegir A\*, BFS, DFS o UCS.

**Sin Raspberry Pi** (no hay PiLink ni ROS 2), sin monitor de batería y sin zumbador.
Es la Propuesta A con la MPU6050 cambiada por el BNO055: L298N, encoders, control de movimiento,
planificador, misión y API son los de la A; el controlador del BNO055 es el de la B.

## Qué hay en la carpeta

```text
roomba-grid-c/
├── platformio.ini            # entornos esp32dev (robot) y native (pruebas en la PC)
├── include/roomba_config.h   # pines, medidas del robot, ganancias y límites
├── include/roomba_settings.h # ajustes que se cambian desde la web y se guardan en NVS
├── src/main.cpp              # arranque y tareas de FreeRTOS
├── lib/
│   ├── L298n/                # PWM 2 kHz en ENA/ENB, sentido con IN1–IN4, freno activo, límite de 6 V
│   ├── QuadEncoder/          # cuadratura ×4 con PCNT (ESP32Encoder) y velocidad por ventana de 50 ms
│   ├── Bno055/               # de la Propuesta B: IMUPLUS, 0x29, espera de arranque + 5 reintentos, RST, offsets en NVS
│   ├── Pose/                 # odometría + rumbo del BNO055 con filtro complementario (β = 0.95)
│   ├── Motion/               # PI de ruedas, avance, giro, frenado
│   ├── Safety/               # atasco (por encoder) y latido del navegador
│   ├── Planner/              # Grid, BFS, DFS, UCS, A*, Held–Karp, 2-opt, primitivas (sin Arduino)
│   ├── Mission/              # máquina de estados y cola de primitivas
│   └── WebApi/               # Wi-Fi, REST y WebSocket
├── data/                     # index.html, app.js, style.css (se suben a LittleFS)
└── test/test_planner/        # 18 pruebas Unity del planificador
```

## Conexiones

| Señal | GPIO | Notas |
| --- | --- | --- |
| ENA (PWM motor izq.) | 25 | Quita el puente ENA del L298N |
| IN1 / IN2 (motor izq.) | 26 / 27 | |
| ENB (PWM motor der.) | 23 | Quita el puente ENB (en la B este pin era el RST del BNO055) |
| IN3 / IN4 (motor der.) | 16 / 17 | Solo en módulos WROOM |
| Encoder izq. A / B | 34 / 35 | Pull-up de 4.7 kΩ a 3.3 V (estos pines no tienen pull-up interno) |
| Encoder der. A / B | 18 / 19 | Pull-up de 4.7 kΩ a 3.3 V |
| SDA / SCL (BNO055) | 21 / 22 | I²C a 100 kHz (el BNO055 estira el reloj), dirección **0x29** (ADR en alto) |
| RST (BNO055) | 33 | Reinicio si el bus se bloquea. Opcional: sin cable, pon `PIN_BNO_RST = -1` |
| Botón paro / inicio | 4 | A GND, INPUT_PULLUP |
| LED de estado | 2 | LED de la placa |

Encoders y BNO055 se alimentan con **3.3 V** de la ESP32 (o el módulo BNO055 a su VIN si trae regulador),
para que ninguna salida pase de 3.3 V. Si tu módulo responde en 0x28, cambia `IMU_I2C_ADDRESS`.
Une todas las tierras (batería, L298N, buck, ESP32) en un solo punto.

## Paso a paso

Necesitas VS Code con la extensión PlatformIO, o PlatformIO por consola. Todos los comandos se escriben
**dentro de la carpeta `roomba-grid-c`**. En VS Code: menú *Terminal → New Terminal*.

**1. Pruebas del planificador en tu laptop** (no necesita la ESP32):

```bash
pio test -e native
```

Debe terminar con `18 Tests 0 Failures`. En Windows, `native` necesita un compilador `gcc`;
si no lo tienes, córrelo desde WSL (Ubuntu) con `pip install platformio` y el mismo comando.

**2. Compilar el firmware:**

```bash
pio run -e esp32dev
```

**3. Subir el firmware** (ESP32 conectada por USB):

```bash
pio run -e esp32dev -t upload
```

**4. Subir la página web** a la memoria de la ESP32 (una vez, y cada vez que cambies algo en `data/`):

```bash
pio run -e esp32dev -t uploadfs
```

**5. Ver los mensajes de arranque** (deja el robot quieto unos segundos: el BNO055 calibra su giroscopio solo):

```bash
pio device monitor
```

Debes ver algo como:

```text
[roomba-grid-c] arrancando
[motores] L298N a 2000 Hz, duty maximo 0.94 (6.0 V en el motor)
[imu] BNO055 en 0x29, IMUPLUS, sin offsets guardados (calibra desde la web)
[web] AP Roomba-UAMM-C 192.168.4.1
[roomba-grid-c] listo: AP Roomba-UAMM-C 192.168.4.1
```

Si dice `BNO055 NO responde`, revisa SDA/SCL y la dirección (0x29 o 0x28). El arranque tarda ~1 s más
que en la A porque el firmware espera al BNO055 y reintenta `begin()`, que en tu módulo falla al primer intento.

**6. Abrir la interfaz:** conecta el teléfono o la laptop a la red Wi-Fi `Roomba-UAMM-C`
(clave `roomba-grid`, cámbiala en `roomba_config.h`) y abre `http://192.168.4.1`.

## Primer arranque con motores (con las ruedas en el aire)

1. En la web, sección *Pruebas*: PWM izq `0.4`, der `0.4`, 1000 ms → **Probar PWM**. Las dos ruedas deben
   girar hacia adelante. Si una gira al revés, cambia `LEFT_MOTOR_INVERT` o `RIGHT_MOTOR_INVERT`.
2. Mira *Cuentas izq / der* en Telemetría: las dos deben **subir** al avanzar. Si una baja, cambia
   `LEFT_ENC_INVERT` o `RIGHT_ENC_INVERT` (o intercambia A y B). Si se queda en 0, revisa el cable y los pull-up.
3. **Cuentas por vuelta**: marca la rueda, gírala 10 vueltas a mano y divide las cuentas entre 10.
   Ponlo en *Ajustes → Cuentas por vuelta* (por defecto 1496 = 11 × 34 × 4, el ejemplo de la propuesta).
4. **Separación entre ruedas**: mide de centro a centro de las llantas y ponla en *Ajustes* (por defecto 0.20 m,
   un valor provisional). El UMBmark de la propuesta la afina.
5. **BNO055**: pulsa **Calibrar BNO055**, deja el robot quieto 3 s y luego inclínalo despacio en 6 posiciones
   (cada cara hacia arriba unos segundos). En Telemetría verás `sys/giro/acel de 3`; cuando giro y acel llegan
   a 3 los offsets se guardan en NVS y se restauran en cada arranque (basta hacerlo una vez).
   Luego, con *Colocar robot en el inicio marcado*, gira el robot a mano a la izquierda: θ debe subir.
   Si baja (BNO055 montado de cabeza), pon `IMU_YAW_SIGN = 1.0f`.
6. Robot en el piso, en una celda: marca esa celda como Inicio con la orientación real y pulsa
   **Colocar robot en el inicio marcado**. Prueba `AVANZAR(1)` y `GIRAR(+90°)`.
7. Con eso bien, marca metas, elige algoritmo, **Planear** y **Ejecutar**.

El botón físico (GPIO 4) hace: paro si está en marcha, ejecutar si hay un plan listo, reanudar si está pausado.
La barra espaciadora en la página también manda PARO.

## De dónde sale cada parte

| Tema | Viene de | En la Propuesta C |
| --- | --- | --- |
| Puente H, encoders, control PI/PD de ruedas, avance y giro | A | Igual que la A (L298N, cuadratura ×4, PD de giro en los dos sentidos) |
| IMU | B | BNO055 en modo IMUPLUS, 0x29, espera 800 ms + 5 reintentos, reinicio por RST (ahora GPIO 33), offsets en NVS |
| Rumbo | A + B | Filtro complementario de la A, pero con el Δθ del rumbo fusionado del BNO055 en vez de ω_z·Δt de la MPU6050: Δθ = β·Δθ_BNO + (1 − β)·Δθ_enc, β = 0.95 (`beta_giro` en la web; 1 = como la B, 0 = solo encoders). No hay sesgo b_z que calibrar: el BNO055 lo corrige solo |
| Reinicio del sensor | B | Si el BNO055 se pierde, el robot se detiene con falla y el reintento (bloquea ~2 s) solo se hace con los motores apagados |
| Calibración desde la web | B | `/api/calibrar` `{"tipo":"imu"}` o `/api/imu/calibrar`: quieto 3 s + 6 posiciones, se guarda sola |
| Raspberry Pi, lidar, PiLink, ROS 2 | B | **Quitados** |
| Planificador, misión, API, web | A = B | Iguales; la web usa tema claro fijo (aunque el teléfono esté en modo oscuro) |
| Wi-Fi | — | `Roomba-UAMM-C`, `roomba-c.local`, para que no choque con la A ni la B |
| Batería y zumbador | — | Quitados, como en la A: vigila tú el voltaje de la 2S (no bajes de ~6.4 V) |

Lo que sigue es la comparación original de la A contra la B; en la C vale todo salvo las filas de IMU,
rumbo, calibración y Wi-Fi, que cambiaron como dice la tabla de arriba.

| Tema | Propuesta B | Propuesta A |
| --- | --- | --- |
| Puente H | 2× BTS7960, PWM 20 kHz en RPWM/LPWM, EN compartido | L298N: PWM 2 kHz en ENA/ENB, sentido con IN1–IN4. Freno activo = IN1 = IN2 = alto con EN al 100 %; rueda libre = EN en bajo |
| Límite de voltaje | — | Duty máximo 0.94 = 6 V / (8.4 V − 2 V de caída del L298N), fijo porque no se mide la batería |
| Encoder | Un canal, el signo se suponía por el comando | Cuadratura ×4 con PCNT (biblioteca ESP32Encoder). El signo es real, así que cada rueda puede invertir el sentido dentro de una primitiva |
| Velocidad de rueda | Periodo entre pulsos | Diferencia de cuentas en una ventana de 50 ms |
| IMU | BNO055 (rumbo absoluto del sensor) | MPU6050, solo giroscopio Z. Sesgo b_z con 500 muestras en reposo al arrancar (si se movió, usa el último guardado en NVS) y se reajusta despacio con las ruedas quietas |
| Rumbo | El del BNO055 | Filtro complementario de la propuesta: Δθ = β·(ω_z − b_z)·Δt + (1 − β)·Δθ_enc, con β = 0.95 ajustable en la web (`beta_giro`; 0 = solo encoders). Con las ruedas quietas no se integra |
| Giro en sitio | Frenar y corregir si se pasa (hasta 3 veces) | PD continuo en los dos sentidos. Termina con \|eθ\| < 1.5° y \|ω\| < 5 °/s durante 100 ms, como dice la propuesta |
| Corte del avance | < 2 cuentas + frenado aprendido | < 5 mm + frenado aprendido |
| Botón | GPIO 18 | GPIO 4 (el 18 ahora es del encoder derecho) |
| Calibración desde la web | `/api/imu/calibrar` (6 posiciones del BNO055) | `/api/calibrar` con `{"tipo":"imu"}` (también responde en `/api/imu/calibrar`): robot quieto 5 s |
| Wi-Fi | `Roomba-UAMM-B`, `roomba-b.local` | `Roomba-UAMM`, `roomba.local` (los de la propuesta) |
| Batería y zumbador | Quitados | Quitados: GPIO 32 libre, sin paro por bajo voltaje. Vigila tú el voltaje de la 2S (no bajes de ~6.4 V) |

Valores que **tienes que confirmar** con tu hardware: cuentas por vuelta, radio de rueda (32.5 mm),
separación entre ruedas (0.20 m provisional), `PWM_MIN` y `KFF` de las ruedas (se ajustan con la prueba de PWM).

## API (para probar con `curl` desde la laptop conectada al robot)

Todas las rutas POST esperan JSON. Si no llevan datos, manda `{}`.

```bash
curl http://192.168.4.1/api/estado
curl -X POST -H "Content-Type: application/json" -d '{"obstaculos":[[2,2],[2,3]]}' http://192.168.4.1/api/mapa
curl -X POST -H "Content-Type: application/json" -d '{"algoritmo":"astar","inicio":[0,0],"orientacion":"E","metas":[[4,4],[0,5]],"regresar":false}' http://192.168.4.1/api/plan
curl -X POST -H "Content-Type: application/json" -d '{}' http://192.168.4.1/api/ejecutar
curl -X POST -H "Content-Type: application/json" -d '{"tipo":"imu"}' http://192.168.4.1/api/calibrar
curl -X POST http://192.168.4.1/api/paro
```

| Ruta | Uso |
| --- | --- |
| GET /api/estado | Pose, celda, estado, velocidades, cuentas, ω y calibración del BNO055 (`calib.sys/gyro/accel`, 0–3) |
| GET, POST /api/mapa | Obstáculos `{obstaculos:[[f,c],…]}` (se guardan en NVS) |
| GET, POST /api/plan | `{algoritmo, heuristica?, inicio, orientacion, metas, regresar, obstaculos?}`; el resultado llega por `/ws` o con GET |
| POST /api/ejecutar | `{confirmar?: true}` |
| POST /api/pausa | Pausa al terminar la primitiva en curso, o reanuda |
| POST /api/paro | Freno activo y luego rueda libre |
| POST /api/calibrar | `{tipo:"imu"}`: calibración del BNO055 (quieto + 6 posiciones), los offsets se guardan en NVS |
| POST /api/imu/cero | `{celda:[f,c], orientacion:"E"}`: coloca el robot en esa celda y rumbo |
| GET, POST /api/config | Ganancias, velocidades, c_g, β; `{restablecer:true}` vuelve a `roomba_config.h` |
| POST /api/prueba | `{tipo:"avanzar",celdas:1}`, `{tipo:"girar",grados:90}` o `{tipo:"pwm",izq:0.4,der:0.4,ms:1000}` |
| POST /api/wifi | `{modo:"sta",ssid:"…",clave:"…"}` o `{modo:"ap"}`; reinicia. En estación: `roomba-c.local` |
| WS /ws | Telemetría cada 100 ms, plan, avisos. El navegador manda `{"tipo":"latido"}` cada segundo |

## Qué está probado y qué no

- **Planificador**: las 18 pruebas pasan (compiladas con g++; es el mismo código de la A y la B).
- **Firmware**: no se pudo compilar para ESP32 donde se escribió (el registro de PlatformIO está bloqueado en
  esa red). `main.cpp`, `Motion.cpp` y `Mission.cpp` se revisaron con g++ contra cabeceras de prueba
  (Arduino, ESP32Encoder, Adafruit BNO055) sin errores ni avisos; `WebApi.cpp` solo cambió en los campos
  del BNO055, copiados de la B. **El primer paso es correr `pio run -e esp32dev`** y, si sale algún error,
  mandarlo tal cual.
- **En el robot real**: nada aún. Las ganancias de `roomba_config.h` son un punto de partida; se ajustan desde la web.
