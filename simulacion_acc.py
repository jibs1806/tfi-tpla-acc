"""
================================================================================
 SIMULACIÓN DE SISTEMA DE CONTROL DE CRUCERO ADAPTATIVO (ACC) - MODO FIJO
 UTN FRBA - Teoría de Control K-4011
================================================================================

Este programa simula el lazo de control de velocidad del ACC en modo fijo,
mostrando en tiempo real las variables fundamentales del sistema:

    Theta_i (t) -> Referencia de velocidad fijada por el conductor      [V]
    e(t)        -> Señal de error = Theta_i(t) - f(t)                   [V]
    Theta_oc(t) -> Salida del controlador PID (señal de control)        [V]
    Theta_o(t)  -> Salida real de la planta = velocidad del vehículo    [V] (mostrada en km/h)
    f(t)        -> Señal de realimentación (elemento de medición)       [V]
    P(t)        -> Perturbación (carga del recorrido: pendiente/viento) [km/h equivalente]

El tablero permite, en tiempo real:
    - Modificar las ganancias Kp, Ki, Kd del controlador PID
    - Modificar la amplitud y duración de la perturbación
    - Inyectar la perturbación en el instante que se desee
    - Pausar y reanudar la simulación (para analizar cambio y recuperación)
    - Reiniciar la simulación completa

Requisitos: numpy, matplotlib
Ejecutar con: python simulacion_acc.py
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

# ==============================================================================
# 1. RELACIÓN DE SOLIDARIDAD: velocidad [km/h] <-> voltaje [V]
#    100 km/h equivalen a 683 Hz en el encoder equivalen a 2.5V en el LM2907
# ==============================================================================
FACTOR_V_KMH = 2.5 / 100.0  # [V / (km/h)]

def kmh_to_v(v_kmh):
    return v_kmh * FACTOR_V_KMH

def v_to_kmh(v_volts):
    return v_volts / FACTOR_V_KMH


# ==============================================================================
# 2. PARÁMETROS DE LA PLANTA  G(s) = K / (tau*s + 1)
# ==============================================================================
K_PLANT = 1.0     # ganancia estática de la planta
TAU = 8.0         # constante de tiempo mecánica del vehículo [s]
DT = 0.1          # paso de integración = ciclo de scan del controlador [s] (10 Hz)
U_MAX = 5.0       # saturación del actuador (esfuerzo máximo de tracción/frenado) [V]
INTEGRAL_CLAMP = 50.0  # anti-windup: límite del acumulador integral

T_MAX = 300.0                 # duración máxima simulable [s]
N_MAX = int(T_MAX / DT)        # cantidad máxima de pasos

V_REF_KMH = 100.0             # velocidad de referencia fijada por el conductor
THETA_I = kmh_to_v(V_REF_KMH)  # referencia en Volts (constante, modo fijo)


# ==============================================================================
# 3. ESTADO DE LA SIMULACIÓN
# ==============================================================================
class EstadoSimulacion:
    def __init__(self):
        self.reset()

    def reset(self, v0_kmh=0.0):
        self.k = 0                      # índice de paso actual
        self.theta_o = kmh_to_v(v0_kmh)  # salida de la planta [V] (velocidad inicial configurable)
        self.integral = 0.0
        self.e_prev = 0.0
        self.pert_activa_desde = -1.0   # instante en que se inyectó la última perturbación
        self.running = False

        # arrays para graficar (rellenos con NaN hasta que se calculan)
        self.t = np.full(N_MAX, np.nan)
        self.theta_i_arr = np.full(N_MAX, np.nan)
        self.e_arr = np.full(N_MAX, np.nan)
        self.theta_oc_arr = np.full(N_MAX, np.nan)
        self.theta_o_kmh_arr = np.full(N_MAX, np.nan)
        self.f_arr = np.full(N_MAX, np.nan)
        self.P_arr = np.full(N_MAX, np.nan)


estado = EstadoSimulacion()


def paso_simulacion(Kp, Ki, Kd, pert_amp_kmh, pert_dur):
    """Ejecuta un paso (un ciclo de scan) del lazo de control completo."""
    if estado.k >= N_MAX:
        estado.running = False
        return

    t = estado.k * DT

    # --- Señal de realimentación f(t): el sensor mide la salida real ---
    f = estado.theta_o

    # --- Punto suma: señal de error e(t) ---
    e = THETA_I - f

    # --- Perturbación P(t): rectángulo de amplitud y duración configurables ---
    if estado.pert_activa_desde >= 0 and estado.pert_activa_desde <= t < estado.pert_activa_desde + pert_dur:
        P_kmh = pert_amp_kmh
    else:
        P_kmh = 0.0
    P_v = kmh_to_v(P_kmh)

    # --- Controlador PID digital (con anti-windup por clamping) ---
    estado.integral += e * DT
    estado.integral = np.clip(estado.integral, -INTEGRAL_CLAMP, INTEGRAL_CLAMP)
    derivada = (e - estado.e_prev) / DT
    theta_oc = Kp * e + Ki * estado.integral + Kd * derivada
    theta_oc = np.clip(theta_oc, -U_MAX, U_MAX)  # saturación del actuador
    estado.e_prev = e

    # --- Planta: primer orden, la perturbación resta esfuerzo efectivo ---
    entrada_efectiva = theta_oc - P_v
    estado.theta_o = estado.theta_o + (DT / TAU) * (-estado.theta_o + K_PLANT * entrada_efectiva)

    # --- Guardar resultados ---
    k = estado.k
    estado.t[k] = t
    estado.theta_i_arr[k] = THETA_I
    estado.e_arr[k] = e
    estado.theta_oc_arr[k] = theta_oc
    estado.theta_o_kmh_arr[k] = v_to_kmh(f)  # graficamos la velocidad ya medida (post-planta anterior)
    estado.f_arr[k] = f
    estado.P_arr[k] = P_kmh

    estado.k += 1


# ==============================================================================
# 4. INTERFAZ GRÁFICA — TABLERO DE CONTROL
# ==============================================================================
fig = plt.figure(figsize=(13, 9))
fig.suptitle("Simulación ACC — Modo Velocidad Fija — Lazo de Control PID", fontsize=13, fontweight="bold")

gs = fig.add_gridspec(4, 2, width_ratios=[3, 1], hspace=0.5, wspace=0.35,
                       left=0.08, right=0.93, top=0.92, bottom=0.06)

ax_v = fig.add_subplot(gs[0, 0])
ax_e = fig.add_subplot(gs[1, 0], sharex=ax_v)
ax_u = fig.add_subplot(gs[2, 0], sharex=ax_v)
ax_p = fig.add_subplot(gs[3, 0], sharex=ax_v)

# --- Subplot 1: velocidad referencia vs real ---
line_vref, = ax_v.plot([], [], color="#3B6D11", lw=1.8, label=r"$\theta_i(t)$ — referencia")
line_vout, = ax_v.plot([], [], color="#185FA5", lw=1.8, label=r"$\theta_o(t)$ — velocidad real")
ax_v.set_ylabel("Velocidad [km/h]")
ax_v.set_ylim(-10, 160)
ax_v.legend(loc="lower right", fontsize=8)
ax_v.set_title("Variable controlada: velocidad del vehículo", fontsize=10)
ax_v.grid(alpha=0.3)

# --- Subplot 2: error ---
line_e, = ax_e.plot([], [], color="#854F0B", lw=1.4)
ax_e.axhline(0, color="gray", lw=0.6)
ax_e.set_ylabel("e(t) [V]")
ax_e.set_ylim(-1.5, 3.0)
ax_e.set_title(r"Señal de error  $e(t) = \theta_i(t) - f(t)$", fontsize=10)
ax_e.grid(alpha=0.3)

# --- Subplot 3: señal de control ---
line_u, = ax_u.plot([], [], color="#534AB7", lw=1.4)
ax_u.axhline(0, color="gray", lw=0.6)
ax_u.set_ylabel(r"$\theta_{oc}(t)$ [V]")
ax_u.set_ylim(-U_MAX - 0.5, U_MAX + 0.5)
ax_u.set_title(r"Salida del controlador PID  $\theta_{oc}(t)$ (saturada en actuador)", fontsize=10)
ax_u.grid(alpha=0.3)

# --- Subplot 4: perturbación ---
line_p, = ax_p.plot([], [], color="#C0392B", lw=1.4)
ax_p.fill_between([], [], color="#C0392B", alpha=0.15)
ax_p.set_ylabel("P(t) [km/h eq.]")
ax_p.set_xlabel("Tiempo [s]")
ax_p.set_ylim(-45, 45)
ax_p.set_title("Perturbación — carga del recorrido (pendiente / viento / asfalto)", fontsize=10)
ax_p.grid(alpha=0.3)

for ax in (ax_v, ax_e, ax_u):
    plt.setp(ax.get_xticklabels(), visible=False)

# ==============================================================================
# 5. PANEL DE CONTROLES (columna derecha)
# ==============================================================================
ax_slider_area = fig.add_axes([0.78, 0.58, 0.15, 0.32])
ax_slider_area.axis("off")

# Sliders de ganancias PID
s_kp_ax = fig.add_axes([0.78, 0.86, 0.15, 0.025])
s_ki_ax = fig.add_axes([0.78, 0.82, 0.15, 0.025])
s_kd_ax = fig.add_axes([0.78, 0.78, 0.15, 0.025])
s_kp = Slider(s_kp_ax, "Kp", 0.0, 3.0, valinit=0.8, valstep=0.05, color="#185FA5")
s_ki = Slider(s_ki_ax, "Ki", 0.0, 1.0, valinit=0.15, valstep=0.01, color="#3B6D11")
s_kd = Slider(s_kd_ax, "Kd", 0.0, 2.0, valinit=0.3, valstep=0.05, color="#854F0B")

fig.text(0.78, 0.895, "Ganancias del controlador PID", fontsize=9, fontweight="bold")

# Sliders de perturbación
s_amp_ax = fig.add_axes([0.78, 0.68, 0.15, 0.025])
s_dur_ax = fig.add_axes([0.78, 0.64, 0.15, 0.025])
s_amp = Slider(s_amp_ax, "Amplitud", -40, 40, valinit=25, valstep=1, color="#C0392B")
s_dur = Slider(s_dur_ax, "Duración [s]", 1, 30, valinit=5, valstep=1, color="#C0392B")

fig.text(0.78, 0.715, "Perturbación (carga del recorrido)", fontsize=9, fontweight="bold")

# Slider de velocidad inicial (condición inicial de la planta)
s_v0_ax = fig.add_axes([0.78, 0.595, 0.15, 0.025])
s_v0 = Slider(s_v0_ax, "V. inicial [km/h]", 0, 150, valinit=0, valstep=5, color="#5F5E5A")
fig.text(0.78, 0.625, "Condición inicial", fontsize=9, fontweight="bold")

# Slider de velocidad de reproducción
s_speed_ax = fig.add_axes([0.78, 0.52, 0.15, 0.025])
s_speed = Slider(s_speed_ax, "Pasos/frame", 1, 10, valinit=2, valstep=1, color="#5F5E5A")
fig.text(0.78, 0.555, "Velocidad de simulación", fontsize=9, fontweight="bold")

# Botones
btn_pert_ax = fig.add_axes([0.78, 0.43, 0.15, 0.045])
btn_pausa_ax = fig.add_axes([0.78, 0.37, 0.15, 0.045])
btn_reset_ax = fig.add_axes([0.78, 0.31, 0.15, 0.045])

btn_pert = Button(btn_pert_ax, "Inyectar perturbación", color="#FADBD8", hovercolor="#F1948A")
btn_pausa = Button(btn_pausa_ax, "▶ Iniciar / Pausar", color="#D5F5E3", hovercolor="#7DCEA0")
btn_reset = Button(btn_reset_ax, "Reiniciar simulación", color="#EAECEE", hovercolor="#BFC9CA")

# Texto de estado (tiempo actual, estado de perturbación)
txt_estado = fig.text(0.78, 0.24, "", fontsize=9, va="top", family="monospace")


# ==============================================================================
# 6. CALLBACKS DE LOS BOTONES
# ==============================================================================
def on_click_pert(event):
    """Inyecta la perturbación en el instante actual de la simulación."""
    estado.pert_activa_desde = estado.k * DT


def on_click_pausa(event):
    estado.running = not estado.running


def on_click_reset(event):
    v0 = s_v0.val
    estado.reset(v0_kmh=v0)
    # Precarga del acumulador integral (bumpless initialization):
    # sin esto, el controlador arranca "sin saber" cuánto esfuerzo hace
    # falta para sostener la velocidad inicial, y la velocidad cae antes
    # de empezar a subir hacia la referencia.
    if s_ki.val > 0:
        estado.integral = kmh_to_v(v0) / (K_PLANT * s_ki.val)
    for ln in (line_vref, line_vout, line_e, line_u, line_p):
        ln.set_data([], [])
    fig.canvas.draw_idle()


btn_pert.on_clicked(on_click_pert)
btn_pausa.on_clicked(on_click_pausa)
btn_reset.on_clicked(on_click_reset)


def on_change_v0(val):
    """Si la simulación todavía no avanzó ningún paso (k==0), aplicar
    la velocidad inicial de inmediato, sin necesidad de tocar 'Reiniciar'.
    Si ya avanzó, el cambio se aplicará recién al presionar 'Reiniciar'."""
    if estado.k == 0:
        estado.theta_o = kmh_to_v(val)
        if s_ki.val > 0:
            estado.integral = kmh_to_v(val) / (K_PLANT * s_ki.val)


s_v0.on_changed(on_change_v0)


# ==============================================================================
# 7. LOOP DE ANIMACIÓN
# ==============================================================================
def actualizar(frame):
    if estado.running:
        pasos = int(s_speed.val)
        for _ in range(pasos):
            paso_simulacion(s_kp.val, s_ki.val, s_kd.val, s_amp.val, s_dur.val)

    k = estado.k
    if k > 0:
        t_actual = estado.t[:k]
        line_vref.set_data(t_actual, v_to_kmh(estado.theta_i_arr[:k]))
        line_vout.set_data(t_actual, estado.theta_o_kmh_arr[:k])
        line_e.set_data(t_actual, estado.e_arr[:k])
        line_u.set_data(t_actual, estado.theta_oc_arr[:k])
        line_p.set_data(t_actual, estado.P_arr[:k])

        # ventana deslizante de 60s para simular "tiempo real"
        t_now = estado.t[k - 1]
        t_ini = max(0, t_now - 60)
        for ax in (ax_v, ax_e, ax_u, ax_p):
            ax.set_xlim(t_ini, max(t_ini + 60, 10))

        pert_on = estado.pert_activa_desde >= 0 and estado.pert_activa_desde <= t_now < estado.pert_activa_desde + s_dur.val
        estado_txt = (
            f"t = {t_now:6.1f} s\n"
            f"v real = {estado.theta_o_kmh_arr[k-1]:6.1f} km/h\n"
            f"e(t)   = {estado.e_arr[k-1]:6.3f} V\n"
            f"Toc(t) = {estado.theta_oc_arr[k-1]:6.3f} V\n"
            f"Perturbación: {'ACTIVA' if pert_on else 'inactiva'}\n"
            f"Estado sim.: {'corriendo' if estado.running else 'pausada'}"
        )
        txt_estado.set_text(estado_txt)

    return line_vref, line_vout, line_e, line_u, line_p


ani = None  # se crea al final para mantener referencia viva

def iniciar_animacion():
    global ani
    from matplotlib.animation import FuncAnimation
    ani = FuncAnimation(fig, actualizar, interval=100, blit=False, cache_frame_data=False)


if __name__ == "__main__":
    iniciar_animacion()
    plt.show()
