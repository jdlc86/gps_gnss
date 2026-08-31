# GPS_GNSS

POC Android para estudiar posicionamiento preciso de vehículos usando únicamente sensores del teléfono.

## Objetivo de esta primera fase

La primera versión **no intenta resolver todavía el parking completo**. Se centra en adquirir datos reproducibles para medir qué precisión real puede obtenerse con un smartphone antes de diseñar el filtro definitivo.

Se registran, cuando el dispositivo y Android lo permiten:

- `Location`: latitud, longitud, altitud, accuracy, speed y bearing.
- Estado GNSS: satélites por constelación, C/N0 y frecuencia de portadora.
- `GnssMeasurementsEvent`: reloj GNSS y medidas raw por satélite.
- IMU: acelerómetro, giroscopio, magnetómetro y rotation vector.
- Timestamps monotónicos (`elapsedRealtimeNanos`) y UTC cuando están disponibles.

Los registros se guardan en CSV separados por sesión para analizarlos posteriormente en Python.

## Principio de arquitectura

```text
Android phone
  ├─ Location / GNSS status
  ├─ Raw GNSS measurements
  └─ IMU
        ↓
 timestamped acquisition
        ↓
 CSV session dataset
        ↓
 Python research
        ↓
 GNSS baseline → EKF → GNSS/IMU → map matching
```

## Lo que no debe confundirse

Un plano georreferenciado puede tener geometría muy precisa y, aun así, la posición estimada del vehículo tener varios metros de error. La POC existe precisamente para medir y reducir ese segundo error.

## Estructura prevista

- `app/`: aplicación Android de adquisición.
- `docs/`: arquitectura, protocolo experimental y decisiones técnicas.
- `analysis/`: scripts Python para métricas y comparación de algoritmos (siguiente etapa).

## Pruebas mínimas

1. Estático durante 5–10 min.
2. Dos puntos con separación conocida.
3. Misma trayectoria repetida varias veces.
4. Trayectoria en vehículo con rectas, giros y aparcamiento.

Las métricas prioritarias serán RMSE, CEP50, CEP95, máximo, desviación estándar, error longitudinal/lateral y repetibilidad.

## Alcance actual

La rama inicial implementa adquisición y logging. No se debe interpretar `accuracy` de Android como error real ni prometer precisión submétrica sin ground truth y ensayos repetibles.
