import pygame
import math
import sys

# --------------------------------------------------------------------------
# PARTE 1: MODELO FÍSICO DEL PÉNDULO INVERTIDO (Sin cambios aquí)
# --------------------------------------------------------------------------

def inverted_pendulum_model(state, force, M, m, L, g, dt):
    """
    Calcula el siguiente estado del péndulo invertido usando el método de Euler.
    """
    x, x_dot, theta, theta_dot = state
    # Asegurarse de que theta esté en el rango [-pi, pi] para un mejor comportamiento
    theta = (theta + math.pi) % (2 * math.pi) - math.pi
    
    sin_theta = math.sin(theta)
    cos_theta = math.cos(theta)
    
    # Ecuaciones del movimiento
    temp = (force + m * L * theta_dot**2 * sin_theta) / (M + m)
    
    theta_dot_dot_num = g * sin_theta - cos_theta * temp
    theta_dot_dot_den = L * (4/3 - (m * cos_theta**2) / (M + m))
    theta_dot_dot = theta_dot_dot_num / theta_dot_dot_den
    
    x_dot_dot = temp - (m * L * theta_dot_dot * cos_theta) / (M + m)
    
    # Integración de Euler
    x_dot += x_dot_dot * dt
    x += x_dot * dt
    theta_dot += theta_dot_dot * dt
    theta += theta_dot * dt
    
    return (x, x_dot, theta, theta_dot)

# --------------------------------------------------------------------------
# PARTE 2: CONTROLADOR PID
# --------------------------------------------------------------------------

class PIDController:
    """
    Clase para implementar un controlador PID.
    """
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self._prev_error = 0
        self._integral = 0

    def update(self, current_value, dt):
        """
        Calcula la salida del PID (la fuerza) para un valor de entrada dado.
        """
        # Calcular error
        error = self.setpoint - current_value
        
        # Término Proporcional
        P_out = self.Kp * error
        
        # Término Integral
        self._integral += error * dt
        I_out = self.Ki * self._integral
        
        # Término Derivativo
        derivative = (error - self._prev_error) / dt
        D_out = self.Kd * derivative
        
        # Salida total del PID
        output = P_out + I_out + D_out
        
        # Guardar error para la siguiente iteración
        self._prev_error = error
        
        return output

# --------------------------------------------------------------------------
# PARTE 3: VISUALIZACIÓN Y CONTROL CON PYGAME
# --------------------------------------------------------------------------

def main():
    pygame.init()

    # --- Configuración de la simulación ---
    WIDTH, HEIGHT = 1000, 600
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Péndulo Invertido con PID | Perturbación: A/D | Reiniciar: R")
    font = pygame.font.SysFont("Arial", 20)

    # Colores
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    RED = (217, 30, 24)
    GREEN_PID = (39, 174, 96)
    GRAY_WALL = (150, 150, 150)

    # --- Parámetros físicos configurables ---
    M = 100    # Masa del carro (kg) - Reducida para un control más ágil
    m = 0.1  # Masa del péndulo (kg)
    L = 0.8     # Longitud del péndulo (m)
    g = 9.81    # Gravedad (m/s^2)
    
    # --- Variables de estado iniciales ---
    x_init = 0.0
    x_dot_init = 0.0
    theta_init = 0.1  # Angulo inicial en radianes (ligeramente inclinado)
    theta_dot_init = 0.0
    state = (x_init, x_dot_init, theta_init, theta_dot_init)
    
    # --- Parámetros de control y simulación ---
    force_manual = 0
    force_magnitude_manual = 250.0 # Fuerza para la perturbación manual
    dt = 1/60

    # --- Configuración del Controlador PID ---
    # Estos valores pueden requerir ajuste para diferentes parámetros físicos
    Kp = 1000  #150
    Ki = 100   #0.5
    Kd = 270  #30
    pid = PIDController(Kp, Ki, Kd, setpoint=0)
    max_force = 100 # Limitar la fuerza máxima que el PID puede aplicar

    # --- Definición de los límites del mundo ---
    scale = 200
    cart_width_pixels = 100
    world_boundary = (WIDTH - cart_width_pixels) / 2 / scale

    # Bucle principal
    running = True
    clock = pygame.time.Clock()

    while running:
        # Manejo de eventos
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                if event.key == pygame.K_r: # Tecla de reinicio
                    state = (x_init, x_dot_init, theta_init, theta_dot_init)
                    pid._integral = 0 # Reiniciar el término integral del PID
                    pid._prev_error = 0

        # Control manual (perturbación)
        keys = pygame.key.get_pressed()
        force_manual = 0
        if keys[pygame.K_a]:
            force_manual = -force_magnitude_manual
        elif keys[pygame.K_d]:
            force_manual = force_magnitude_manual

        # --- Lógica del Controlador PID ---
        # El estado del péndulo a controlar es theta (state[2])
        force_pid = pid.update(state[2], dt)

        # Limitar la fuerza del PID para evitar inestabilidad
        force_pid = max(-max_force, min(max_force, force_pid))
        
        # La fuerza total es la suma de la del PID y la perturbación manual
        total_force = force_pid + force_manual
        
        # Actualizar el estado del péndulo
        state = inverted_pendulum_model(state, total_force, M, m, L, g, dt)

        # Lógica para limitar el carro a la pantalla
        x, x_dot, theta, theta_dot = state
        if x > world_boundary:
            x = world_boundary
            x_dot = 0
        elif x < -world_boundary:
            x = -world_boundary
            x_dot = 0
        state = (x, x_dot, theta, theta_dot)
        
        # --- Dibujado en pantalla ---
        screen.fill(WHITE)
        
        floor_y = HEIGHT - 100
        pygame.draw.line(screen, BLACK, (0, floor_y), (WIDTH, floor_y), 2)
        pygame.draw.line(screen, GRAY_WALL, (0, 0), (0, floor_y), 10)
        pygame.draw.line(screen, GRAY_WALL, (WIDTH - 5, 0), (WIDTH - 5, floor_y), 10)
        
        cart_height = 50
        cart_x = state[0] * scale + WIDTH / 2 - cart_width_pixels / 2
        cart_y = floor_y - cart_height
        pygame.draw.rect(screen, BLACK, (cart_x, cart_y, cart_width_pixels, cart_height))

        pendulum_x_start = cart_x + cart_width_pixels / 2
        pendulum_y_start = cart_y
        pendulum_x_end = pendulum_x_start + L * scale * math.sin(state[2])
        pendulum_y_end = pendulum_y_start - L * scale * math.cos(state[2])
        pygame.draw.line(screen, RED, (pendulum_x_start, pendulum_y_start), (pendulum_x_end, pendulum_y_end), 8)
        pygame.draw.circle(screen, RED, (int(pendulum_x_end), int(pendulum_y_end)), 15)
        
        # Mostrar información
        info_texts = [
            f"Posición Carro (x): {state[0]:.2f} m",
            f"Ángulo Péndulo (θ): {math.degrees(state[2]):.2f}°",
            f"Fuerza PID: {force_pid:.1f} N",
            f"Fuerza Manual (Perturbación): {force_manual:.1f} N",
            f"Presiona 'R' para reiniciar"
        ]
        for i, text in enumerate(info_texts):
            color = GREEN_PID if "PID" in text else BLACK
            text_surface = font.render(text, True, color)
            screen.blit(text_surface, (10, 10 + i * 25))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()