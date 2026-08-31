# Protocolo experimental de posicionamiento

## 1. Qué estamos midiendo

Hay que separar dos conceptos:

- **Precisión/repetibilidad**: cuánto se agrupan las soluciones entre sí.
- **Exactitud (accuracy real)**: cuánto se acercan a una posición de referencia conocida.

Un móvil puede producir una nube de 30 cm muy estable desplazada 3 m respecto a la posición real. Ese caso tiene buena repetibilidad y mala exactitud.

El campo `Location.accuracy` de Android es una estimación del sistema, no debe usarse como ground truth.

## 2. Test estático

1. Colocar el móvil inmóvil, con orientación y posición física constantes.
2. Registrar al menos 10 min al aire libre.
3. No tocar el teléfono durante el ensayo.
4. Anotar dispositivo, versión Android, condiciones meteorológicas, obstrucciones y posición del móvil.
5. Repetir el ensayo en distintos momentos del día.

Si existe un punto topográfico conocido, registrar sus coordenadas WGS84 como referencia. Si no existe, el ensayo solo permite estudiar dispersión/repetibilidad, no error absoluto verdadero.

## 3. Dos puntos conocidos

Usar dos posiciones A y B cuya separación física se haya medido independientemente. Repetir A → B → A varias veces. No usar la propia solución GNSS del móvil para definir la distancia de referencia.

Métricas:

- error de distancia A-B;
- dispersión en A y B;
- sesgo sistemático;
- repetibilidad entre visitas.

## 4. Trayectoria repetida

Recorrer la misma línea o circuito al menos 5 veces. Conviene disponer de puntos de control físicos existentes en el terreno, aunque esos puntos no formen parte del sistema final de posicionamiento.

Medir especialmente el error transversal a la trayectoria: para navegación entre filas puede ser más importante que el error longitudinal.

## 5. Vehículo

Fijar el móvil mecánicamente en la misma posición del vehículo. No cambiarlo de bolsillo, mano o soporte entre repeticiones.

Registrar:

- rectas;
- giros de 90°;
- parada completa;
- maniobra de aparcamiento;
- recorridos repetidos.

La IMU del teléfono describe el movimiento del **teléfono**. Para inferir la pose del vehículo será necesario conocer o calibrar la transformación teléfono→vehículo.

## 6. Métricas mínimas

Con ground truth:

- error horizontal por muestra;
- bias Este/Norte;
- RMSE horizontal;
- percentil 50 (CEP50 empírico);
- percentil 95 (CEP95 empírico);
- máximo;
- error longitudinal y lateral respecto al heading/ruta.

Sin ground truth:

- dispersión alrededor de una referencia robusta (mediana);
- percentiles 50/95 de esa dispersión;
- desviación estándar Este/Norte;
- deriva temporal.

No llamar a estas últimas métricas “accuracy absoluta”.

## 7. Criterio de decisión para la POC

No se debe seleccionar un algoritmo por una captura visual. Comparar exactamente los mismos datasets y reportar al menos P50/P95 y outliers.

Una mejora útil para el parking debe reducir especialmente el error lateral y las colas de la distribución. Una solución que alcanza 0.3 m ocasionalmente pero presenta saltos de 4 m no es equivalente a una solución estable alrededor de 1 m.

## 8. Etapas de algoritmo

Orden recomendado:

1. Baseline `Location` Android.
2. Estadística y detección de outliers.
3. Velocidad/bearing GNSS y Doppler raw.
4. Orientación/giros mediante IMU.
5. EKF loosely coupled.
6. Restricciones cinemáticas del vehículo.
7. Map matching sobre calles/filas del parking.
8. Solo después: tightly coupled, factor graphs o técnicas avanzadas de Raw GNSS.

La POC debe conservar siempre el dataset raw para poder reprocesarlo sin repetir el ensayo.
