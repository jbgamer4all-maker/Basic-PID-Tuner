target = 4.0
current = 0.0

class Plant:
    def __init__(self, position):
        self.position = position
    
    def update(self, control_signal, dt):
        self.position += control_signal * dt

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.integral_sum = 0.0
        self.previous_error = 0.0
        
    def compute(self, error, dt):
        p_term = self.kp * error
        
        self.integral_sum += error * dt
        i_term = self.ki * self.integral_sum
        
        derivative = (error - self.previous_error) / dt
        d_term = self.kd * derivative
        
        self.previous_error = error
        
        output = p_term + i_term + d_term
        return output
        
    def auto_tune(self, error):
        if abs(error) > 2.0:
            self.kp += 0.01
        elif abs(error) < 0.2:
            self.kp -= 0.005

plant = Plant(current)
controller = PIDController(kp=1.0, ki=0.1, kd=0.05)
dt = 0.1

while True:
    current_error = target - plant.position
    
    if -0.01 < current_error < 0.01:
        print("Target reached!")
        break
        
    pid_output = controller.compute(current_error, dt)
    controller.auto_tune(current_error)
    plant.update(pid_output, dt)
    
    print(f"Position: {plant.position:.2f} | Error: {current_error:.2f} | Kp: {controller.kp:.3f} | Ki: {controller.ki:.3f} | Kd: {controller.kd:.3f}")