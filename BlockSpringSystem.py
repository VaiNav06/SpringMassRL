import numpy as np
import torch
import pyglet

# system parameters
mass = 1.0            # mass of the object
spring_const = 5.0    # stiffness of the spring
damping = 0.5         # damping coefficient
scale_factor = 200.0  # scale for visualization
initial_pos = 3.0     # starting position of the mass
force_scale = 20.0    # force scaling for PID control

# pid controller parameters
kp = 10.0
ki = 2.0
kd = 5.0

# system dynamics (state-space representation)
a_matrix = np.array([[0, 1], [-spring_const / mass, -damping / mass]])
b_matrix = np.array([[0], [1 / mass]])

# create a pyglet window
window = pyglet.window.Window(1280, 720, "mass-spring-damper with pid control")
batch = pyglet.graphics.Batch()

# draw ground and objects
ground_y = 100
ground = pyglet.shapes.Line(100, ground_y, 1180, ground_y, thickness=2, batch=batch)
mass_block = pyglet.shapes.Rectangle(615, ground_y, 50, 50, color=(255, 0, 0), batch=batch)
spring = pyglet.shapes.Line(640, ground_y, 640, ground_y, thickness=2, color=(0, 255, 0), batch=batch)
damper_body = pyglet.shapes.Rectangle(690, ground_y, 20, 100, color=(0, 0, 255), batch=batch)
damper_piston = pyglet.shapes.Rectangle(685, ground_y, 30, 10, color=(100, 100, 255), batch=batch)

# initial state (position and velocity)
state = np.array([[initial_pos], [0.0]])
target_pos = initial_pos

# target line to visualize the desired position
target_line = pyglet.shapes.Line(100, ground_y + initial_pos * scale_factor,
                                 1180, ground_y + initial_pos * scale_factor,
                                 color=(255, 255, 0), batch=batch)

# pid variables
error_sum = 0.0
prev_error = 0.0

# label to show pid values
label = pyglet.text.Label('', font_size=12, x=10, y=700, anchor_x='left', anchor_y='center', color=(255, 255, 255, 255))

@window.event
def on_draw():
    window.clear()
    batch.draw()
    update_label()
    label.draw()

@window.event
def on_key_press(symbol, modifiers):
    """ adjust target position and pid gains using keyboard input """
    global target_pos, kp, ki, kd
    
    if symbol == pyglet.window.key.UP:
        target_pos += 0.1
    elif symbol == pyglet.window.key.DOWN:
        target_pos -= 0.1
    elif symbol == pyglet.window.key.W:
        kp += 1.0
    elif symbol == pyglet.window.key.S:
        kp = max(0, kp - 1.0)
    elif symbol == pyglet.window.key.A:
        ki += 0.5
    elif symbol == pyglet.window.key.D:
        ki = max(0, ki - 0.5)
    elif symbol == pyglet.window.key.Q:
        kd += 1.0
    elif symbol == pyglet.window.key.E:
        kd = max(0, kd - 1.0)
    
    target_y = (target_pos * scale_factor) + ground_y
    target_line.y = target_y
    target_line.y2 = target_y

def update_label():
    """ update the label text with current pid values and target position """
    label.text = f"kp: {kp:.2f}, ki: {ki:.2f}, kd: {kd:.2f}, target: {target_pos:.2f} m"

def pid_control(dt):
    """ calculate the control force using a pid controller """
    global error_sum, prev_error
    
    current_pos = state[0, 0]
    error = target_pos - current_pos
    
    proportional = kp * error
    error_sum += error * dt
    integral = ki * error_sum
    derivative = kd * (error - prev_error) / dt
    
    prev_error = error
    
    return proportional + integral + derivative

def update(dt):
    """ update the state of the system and redraw objects """
    global state
    
    # compute control force
    control_force = pid_control(dt)
    
    # update state using the system dynamics
    state = state + dt * (a_matrix @ state + b_matrix @ np.array([[control_force]]))
    
    # prevent the mass from going below ground
    if state[0, 0] < 0:
        state[0, 0] = 0
        state[1, 0] *= -0.8  # simulate bounce with energy loss
    
    # update object positions
    y_pos = (state[0, 0] * scale_factor) + ground_y
    mass_block.y = y_pos
    spring.y2 = y_pos + 25
    damper_body.height = abs(state[0, 0] * scale_factor)
    damper_body.y = min(y_pos, ground_y)
    damper_piston.y = y_pos + 20

# run the simulation
pyglet.clock.schedule_interval(update, 1/60.0)
pyglet.app.run()
