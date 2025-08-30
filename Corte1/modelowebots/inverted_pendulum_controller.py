"""
Controlador PID para equilibrar un péndulo invertido en Webots R2025a.
Incluye perturbaciones manuales con las teclas A y D (versión re-sintonizada).
"""

from controller import Robot, Keyboard

# --- Parámetros de Simulación y PID ---
TIME_STEP = 16

# --- Constantes del PID (VALORES AJUSTADOS) ---
# Kp se aumenta para una reacción mucho más fuerte.
Kp = 300
# Ki se mantiene muy bajo para evitar inestabilidad.
Ki = 0.1
# Kd se aumenta para amortiguar las oscilaciones de un Kp alto.
Kd = 30.0

# --- Parámetros de Perturbación ---
PERTURBATION_FORCE = 1

# Inicialización de variables del PID
integral = 0.0
previous_error = 0.0

# --- Creación y Configuración del Robot ---
robot = Robot()

# Obtener dispositivos
pole_angle_sensor = robot.getDevice("pole position sensor")
cart_position_sensor = robot.getDevice("cart position sensor")
cart_motor = robot.getDevice("cart motor")

# Habilitar el teclado
keyboard = robot.getKeyboard()
keyboard.enable(TIME_STEP)

# Habilitar los sensores
pole_angle_sensor.enable(TIME_STEP)
cart_position_sensor.enable(TIME_STEP)

# Configurar el motor
cart_motor.setPosition(float('inf'))
cart_motor.setVelocity(0.0)

print("Controlador PID iniciado para Webots R2025a.")
print("Presiona la ventana de simulación y usa 'A' y 'D' para empujar el carro.")
print("Sintonización actual: Kp={}, Ki={}, Kd={}".format(Kp, Ki, Kd))

# --- Bucle de Control Principal ---
while robot.step(TIME_STEP) != -1:
    key = keyboard.getKey()

    # Si se presiona una tecla, aplicamos SOLO la fuerza manual.
    if key == ord('A') or key == ord('a'):
        total_force = -PERTURBATION_FORCE
    elif key == ord('D') or key == ord('d'):
        total_force = PERTURBATION_FORCE
    else:
        # Si no se presiona ninguna tecla, dejamos que el PID trabaje.
        pole_angle = pole_angle_sensor.getValue()
        error = pole_angle

        # Calcular términos del PID
        P = Kp * error
        integral += error * (TIME_STEP / 1000.0)
        I = Ki * integral
        derivative = (error - previous_error) / (TIME_STEP / 1000.0)
        D = Kd * derivative

        pid_force = P + I + D
        total_force = -pid_force
        
        previous_error = error

    # Aplicar la fuerza decidida al motor
    cart_motor.setForce(total_force)