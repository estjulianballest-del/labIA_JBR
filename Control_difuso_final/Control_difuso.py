import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ========================
# PARÁMETROS DEL SISTEMA
# ========================
g = 9.81
M = 0.5
m = 0.2
l = 0.3
dt = 0.02

# ========================
# FUNCIONES DIFUSAS
# ========================
def trapmf(x, a, b, c, d):
    return np.maximum(
        np.minimum(np.minimum((x-a)/(b-a+1e-6), 1), (d-x)/(d-c+1e-6)), 0
    )

def angulo_mf(theta):
    return {
        "amn": trapmf(theta, -180, -180, -60, -30),
        "an":  trapmf(theta, -60, -30, -5, 0),
        "ap":  trapmf(theta, 0, 5, 30, 60),
        "amp": trapmf(theta, 30, 60, 180, 180),
    }

def vel_mf(omega):
    return {
        "vn": trapmf(omega, -10, -10, -0.5, 0),
        "vp": trapmf(omega, 0, 0.5, 10, 10),
    }

x_vals = np.linspace(-100, 100, 200)
def fuerza_mf():
    return {
        "xmn": trapmf(x_vals, -100, -100, -80, -60),
        "xn":  trapmf(x_vals, -60, -40, -20, 0),
        "x0":  trapmf(x_vals, -10, 0, 0, 10),
        "xp":  trapmf(x_vals, 0, 20, 40, 60),
        "xmp": trapmf(x_vals, 60, 80, 100, 100),
    }

def fuzzy_force(theta_deg, omega):
    mu_theta = angulo_mf(theta_deg)
    mu_omega = vel_mf(omega)
    mu_F = fuerza_mf()

    rules = []
    rules.append(np.minimum(mu_theta["ap"], mu_omega["vp"]) * mu_F["xp"])
    rules.append(np.minimum(mu_theta["ap"], mu_omega["vn"]) * mu_F["x0"])
    rules.append(np.minimum(mu_theta["an"], mu_omega["vp"]) * mu_F["x0"])
    rules.append(np.minimum(mu_theta["an"], mu_omega["vn"]) * mu_F["xn"])
    rules.append(np.minimum(mu_theta["amp"], mu_omega["vp"]) * mu_F["xmp"])
    rules.append(np.minimum(mu_theta["amp"], mu_omega["vn"]) * mu_F["xp"])
    rules.append(np.minimum(mu_theta["amn"], mu_omega["vp"]) * mu_F["xn"])
    rules.append(np.minimum(mu_theta["amn"], mu_omega["vn"]) * mu_F["xmn"])

    aggregated = np.maximum.reduce(rules)
    if np.sum(aggregated) == 0:
        return 0.0
    return np.sum(aggregated * x_vals) / np.sum(aggregated)

# ========================
# DINÁMICA DEL PÉNDULO
# ========================
def cartpole_dynamics(state, F):
    x, xdot, th, thdot = state
    sin_th, cos_th = math.sin(th), math.cos(th)
    total_mass = M + m
    temp = (F + m * l * thdot**2 * sin_th) / total_mass
    denom = l * (4.0/3.0 - (m * cos_th**2) / total_mass)
    thddot = (g * sin_th - cos_th * temp) / denom
    xddot = temp - (m * l * thddot * cos_th) / total_mass
    return np.array([xdot, xddot, thdot, thddot])

def rk4_step(state, F, dt):
    k1 = cartpole_dynamics(state, F)
    k2 = cartpole_dynamics(state + 0.5 * dt * k1, F)
    k3 = cartpole_dynamics(state + 0.5 * dt * k2, F)
    k4 = cartpole_dynamics(state + dt * k3, F)
    return state + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)

# ========================
# ANIMACIÓN
# ========================
state = np.array([0.0, 0.0, np.deg2rad(10), 0.0])  # estado inicial

fig, ax = plt.subplots()
ax.set_xlim(-2, 2)
ax.set_ylim(-1, 1.2)

# QUITAR EJES
ax.axis('off')

# PISO
ax.plot([-3, 3], [-0.1, -0.1], 'k-', lw=3, color="gray")

# Elementos gráficos
car_width, car_height = 0.3, 0.2
cart = plt.Rectangle((-car_width/2, -car_height/2), car_width, car_height, fc='blue')
ax.add_patch(cart)
line, = ax.plot([], [], lw=3, c='red')
time_text = ax.text(0.02, 0.9, '', transform=ax.transAxes, fontsize=10, color="black")

def init():
    cart.set_xy((-car_width/2, -car_height/2))
    line.set_data([], [])
    time_text.set_text('')
    return cart, line, time_text

def animate(i):
    global state
    theta_deg = np.rad2deg(state[2])
    omega = state[3]
    F = fuzzy_force(theta_deg, omega)

    state[:] = rk4_step(state, F, dt)

    x, _, th, _ = state
    cart.set_x(x - car_width/2)

    # Coordenadas del péndulo (vertical hacia arriba en 0 rad)
    px = x + l * np.sin(th)
    py = 0.0 + l * np.cos(th)
    line.set_data([x, px], [0, py])

    time_text.set_text(f't={i*dt:.1f}s | θ={theta_deg:.1f}° | F={F:.1f}N')
    return cart, line, time_text

ani = animation.FuncAnimation(fig, animate, frames=500, init_func=init,
                              interval=dt*1000, blit=True)
plt.show()
