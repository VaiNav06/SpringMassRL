import pyglet
import numpy as np

m = 1.0
k = 5.0
c = 0.5
scale_factor = 200.0
initial_displacement = 3.0
force_scale = 20.0

kp = 10.0
ki = 2.0
kd = 5.0

A = np.array([[0, 1], [-k/m, -c/m]])
B = np.array([[0], [1/m]])

window = pyglet.window.Window(1280, 720, "Mass-Spring-Damper System with PID Control")
batch = pyglet.graphics.Batch()

ground_y = 100
ground = pyglet.shapes.Line(100, ground_y, 1180, ground_y, thickness=2, batch=batch)
mass_object = pyglet.shapes.Rectangle(615, ground_y, 50, 50, color=(255, 0, 0), batch=batch)
spring = pyglet.shapes.Line(640, ground_y, 640, ground_y + initial_displacement * scale_factor, thickness=2, color=(0, 255, 0), batch=batch)
damper_body = pyglet.shapes.Rectangle(690, ground_y, 20, initial_displacement * scale_factor, color=(0, 0, 255), batch=batch)
damper_piston = pyglet.shapes.Rectangle(685, ground_y + initial_displacement * scale_factor - 10, 30, 10, color=(100, 100, 255), batch=batch)

control_force_line = pyglet.shapes.Line(640, ground_y + initial_displacement * scale_factor, 640, ground_y + initial_displacement * scale_factor, thickness=2, color=(255, 165, 0), batch=batch)
spring_force_line = pyglet.shapes.Line(660, ground_y + initial_displacement * scale_factor, 660, ground_y + initial_displacement * scale_factor, thickness=2, color=(0, 255, 0), batch=batch)
damper_force_line = pyglet.shapes.Line(620, ground_y + initial_displacement * scale_factor, 620, ground_y + initial_displacement * scale_factor, thickness=2, color=(0, 0, 255), batch=batch)

state = np.array([[initial_displacement], [0.0]])
target_position = initial_displacement

target_line = pyglet.shapes.Line(100, ground_y + initial_displacement * scale_factor, 1180, ground_y + initial_displacement * scale_factor, color=(255, 255, 0), batch=batch)

error_sum = 0.0
previous_error = 0.0

label = pyglet.text.Label('', font_size=12, x=10, y=700, anchor_x='left', anchor_y='center', color=(255, 255, 255, 255))

@window.event
def on_draw():
    window.clear()
    batch.draw()
    update_label()
    label.draw()

@window.event
def on_key_press(symbol, modifiers):
    global target_position

    if symbol == pyglet.window.key.UP:
        target_position += 0.1
    elif symbol == pyglet.window.key.DOWN:
        target_position -= 0.1

    target_y = (target_position * scale_factor) + ground_y
    target_line.y = target_y
    target_line.y2 = target_y

def update_label():
    label.text = f"Target: {target_position:.2f} m, Kp: {kp:.2f}, Ki: {ki:.2f}, Kd: {kd:.2f}"

def pid_control(dt):
    global error_sum, previous_error

    current_position = state[0][0]
    error = target_position - current_position

    proportional = kp * error
    error_sum += error * dt
    integral = ki * error_sum
    derivative = kd * (error - previous_error) / dt

    previous_error = error

    return proportional + integral + derivative

def update(dt):
    global state

    control_force = pid_control(dt)
    state += dt * (A @ state + B @ np.array([[control_force]]))

    if state[0][0] < -ground_y / scale_factor:
        state[1][0] *= -1

    y_position = (state[0][0] * scale_factor) + ground_y

    mass_object.y = y_position
    spring.y2 = y_position + mass_object.height // 2
    damper_body.height = abs(state[0][0] * scale_factor)
    damper_piston.y = y_position + mass_object.height // 2

    control_force_line.y = y_position
    control_force_line.y2 = y_position + control_force * force_scale
    spring_force_line.y = y_position
    spring_force_line.y2 = y_position + (-k * state[0][0]) * force_scale
    damper_force_line.y = y_position
    damper_force_line.y2 = y_position + (-c * state[1][0]) * force_scale

pyglet.clock.schedule_interval(update, 1/60.0)
pyglet.app.run()