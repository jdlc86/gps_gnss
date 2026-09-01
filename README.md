# GPS_GNSS

POC Android para estudiar posicionamiento preciso de vehículos usando únicamente sensores del teléfono.

## Objetivo de esta primera fase

La primera versión **no intenta resolver todavía el parking completo**. Se centra en adquirir datos reproducibles para medir qué precisión real puede obtenerse con un smartphone antes de diseñar el filtro definitivo.

Se registran, cuando el dispositivo y Android lo permiten:

- `Location`: latitud, longitud, altitud, accuracy, speed y bearing.
- Estado GNSS: satélites por constelación, C/N0 y frecuencia de portadora.
- `GnssMeasurementsEvent`: reloj GNSS y medidas raw por señal.
- IMU: acelerómetro, giroscopio, magnetómetro y rotation vector.
- Timestamps monotónicos (`elapsedRealtimeNanos`) y UTC cuando están disponibles.

## No reinventar el ecosistema GNSS

La POC adopta el esquema de logging de **Google GnssLogger / gps-measurement-tools** en lugar de crear un formato raw propio. Cada sesión genera un `gnss_log.txt` con filas compatibles del tipo:

- `Fix`
- `Raw`
- `Status`
- `UncalAccel` / `Accel`
- `UncalGyro` / `Gyro`
- `UncalMag` / `Mag`
- `OrientationDeg`

Cuando existe una variante IMU no calibrada, se prioriza porque conserva la estimación de bias del sensor y resulta más útil para investigación GNSS/INS.

La compatibilidad actual es deliberadamente un **subconjunto** del formato GnssLogger. Todavía no registramos `Nav`, `NMEA`, `Agc` separado ni `GnssAntennaInfo`. Se añadirán solo si aportan valor al procesamiento que seleccionemos.

## Arquitectura

```text
Android phone
  ├─ Location / GNSS status
  ├─ Raw GNSS measurements
  └─ calibrated/uncalibrated IMU
        ↓
 GnssLogger-compatible acquisition
        ↓
 gnss_log.txt
        ↓
 ┌──────────────────────────────────────┐
 │ Existing GNSS tools / WLS baselines │
 │ Our Python experiments              │
 │ Existing EKF/INS implementations    │
 └──────────────────────────────────────┘
        ↓
 select best baseline
        ↓
 vehicle model + parking map matching
```

La idea es reutilizar algoritmos existentes primero. Solo desarrollaremos componentes propios cuando exista una razón medible para hacerlo, especialmente el modelo cinemático del vehículo, las restricciones geométricas del parking y el map matching.

## Lo que no debe confundirse

Un plano georreferenciado puede tener geometría muy precisa y, aun así, la posición estimada del vehículo tener varios metros de error. La POC existe precisamente para medir y reducir ese segundo error.

Un P95 pequeño alrededor de la mediana tampoco demuestra exactitud absoluta. Sin una coordenada de referencia independiente solo estamos midiendo dispersión/repetibilidad.

## Análisis inicial

```bash
pip install -r analysis/requirements.txt
python analysis/analyze_static.py path/to/gnss_log.txt
```

Con una posición de referencia conocida:

```bash
python analysis/analyze_static.py path/to/gnss_log.txt \
  --truth-lat 40.0000000 --truth-lon -3.0000000
```

## Pruebas mínimas

1. Estático durante 5–10 min.
2. Dos puntos con separación conocida.
3. Misma trayectoria repetida varias veces.
4. Trayectoria en vehículo con rectas, giros y aparcamiento.

Las métricas prioritarias serán RMSE, P50/P95, máximo, desviación estándar, error longitudinal/lateral y repetibilidad.

## Próximo criterio de decisión

Antes de escribir un EKF propio debemos obtener un dataset real del teléfono y comparar, como mínimo:

1. posición Android `Fix`;
2. solución raw/WLS disponible en herramientas existentes;
3. calidad y continuidad de Doppler/ADR;
4. ruido y bias de IMU;
5. disponibilidad real de señales multifrecuencia.

Solo después decidiremos si integrar un EKF existente, modificarlo o desarrollar un modelo específico para el vehículo.
