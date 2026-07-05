# Simulación de Sistema de Control de Crucero Adaptativo (ACC) - Modo Velocidad Fija
## UTN FRBA - Teoría de Control (Curso K-4011)

Este proyecto contiene una simulación interactiva en tiempo real del lazo de control de velocidad para un **Sistema de Control de Crucero Adaptativo (ACC)** operando en modo fijo. Permite sintonizar un controlador PID digital, inyectar perturbaciones (cargas del camino como pendientes o viento) y analizar de manera visual el comportamiento dinámico del vehículo.

---

## 🛠️ Requisitos Previos

Para ejecutar la simulación, es necesario contar con **Python 3.x** y las siguientes librerías de cálculo y graficación:

* **NumPy**: Para el modelado numérico y procesamiento de las señales.
* **Matplotlib**: Para renderizar el tablero interactivo y los gráficos dinámicos.

---

## 🚀 Cómo Correr la Simulación

Sigue estos pasos en tu terminal para clonar, preparar y ejecutar el entorno:

1. **Clonar o acceder al repositorio:**
   Asegúrate de estar ubicado en la raíz del proyecto:
   ```bash
   cd tfi-tpla-acc
   ```

2. **Instalar las dependencias:**
   Puedes instalar los requisitos necesarios a través de `pip`:
   ```bash
   pip install numpy matplotlib
   ```

3. **Ejecutar el script principal:**
   Inicia la simulación corriendo el archivo `simulacion_acc.py`:
   ```bash
   python simulacion_acc.py
   ```

---

## 📊 Descripción del Tablero Interactiva

Al ejecutar la simulación, se abrirá una ventana gráfica interactiva con cuatro curvas y un panel de control a la derecha:

### 1. Variables del Lazo de Control (Gráficos)
* **Velocidad del Vehículo:** Grafica la referencia deseada fijada por el conductor ($\theta_i(t) = 100\text{ km/h}$ en verde) versus la velocidad real medida del auto ($\theta_o(t)$ en azul).
* **Señal de Error $e(t)$:** Muestra la señal de error en voltios generada por la diferencia entre la referencia y la realimentación: $e(t) = \theta_i(t) - f(t)$.
* **Señal de Control $\theta_{oc}(t)$:** Es la salida directa del controlador PID (tensión de control) inyectada al actuador del acelerador. Está físicamente saturada en $\pm 5\text{ V}$.
* **Perturbación $P(t)$:** Carga de oposición equivalente en km/h que simula el ingreso a una pendiente o resistencia del viento.

### 2. Panel de Sintonización y Control (Lado Derecho)
* **Ganancias del PID (Kp, Ki, Kd):** Deslizadores para cambiar los parámetros del controlador digital en caliente para observar cambios en estabilidad, sobrepico y tiempo de establecimiento.
* **Perturbación (Amplitud y Duración):** Ajusta las características de la perturbación que se inyectará.
* **Condición Inicial (Velocidad Inicial):** Deslizador para ajustar la velocidad inicial ($V_0$) con la que arranca el vehículo al reiniciar la simulación (configurable de $0$ a $150\text{ km/h}$).
* **Velocidad de Simulación:** Permite acelerar el paso del tiempo en el gráfico acelerando el procesamiento de ciclos de escaneo por cuadro visual.
* **Botones:**
  * `▶ Iniciar / Pausar`: Arranca o detiene temporalmente la simulación.
  * `Inyectar perturbación`: Inyecta instantáneamente el escalón de perturbación configurado.
  * `Reiniciar simulación`: Limpia el historial de gráficos y resetea las variables a la velocidad inicial configurada.

---

## ⚙️ Parámetros Técnicos y Modelo de la Planta

* **Planta $G(s)$:** Modelada como un sistema de primer orden:
  $$G(s) = \frac{K}{\tau s + 1}$$
  Donde la ganancia estática $K = 1.0$ y la constante de tiempo mecánica del vehículo es $\tau = 8.0\text{ s}$.
* **Período de Muestreo ($DT$):** Ciclo de scan fijado en $0.1\text{ s}$ ($10\text{ Hz}$).
* **Relación de Solidaridad (Conversión de magnitud):** 
  $100\text{ km/h} \leftrightarrow 683\text{ Hz}\text{ (Sensor/Encoder)} \leftrightarrow 2.5\text{ V}\text{ (Convertidor de Frecuencia a Tensión LM2907)}$.
* **Anti-Windup:** Implementa un acumulador integral acotado (clamping) en $\pm 50$ para evitar la saturación excesiva de la acción integral.