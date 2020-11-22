"""e-puck_touch_obstacles controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from datetime import datetime
from datetime import timedelta
import sys

TIME_STEP = 64
MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())



# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:

# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

leds = []
ledNames = [
    'led0', 'led1', 'led2', 'led3', 'led4',
    'led5', 'led6', 'led7', 'led8', 'led9'
]

for i in range(len(ledNames)):
    leds.append(robot.getLED(ledNames[i]))

for i in range(8):
    ps.append(robot.getDistanceSensor(psNames[i]))
    ps[i].enable(TIME_STEP)
    
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

def lightAllLeds(num):
    diff = qtd_blocos_moveis_encontrados if num == 0 else 0
    for k in range(3):
        for i in range(len(leds) - diff):
            leds[i].set(num)
        


#leftMotor.setVelocity(0.1 * MAX_SPEED)
#rightMotor.setVelocity(0.1 * MAX_SPEED)
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

list_queues = []
for i in range(8):
    list_queues.append([])
    
avgArray = []
for i in range(8):
    avgArray.append([])
time_to_stop_turning = datetime.now()

qtd_blocos_moveis_encontrados = 0
    
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    # Enter here functions to read sensor data, like:
    # detect obstacles
    non_solid_sensor_value = 800
    sensor_value = 700
    min_percentage = 0.05
    right_obstacle = psValues[0] > sensor_value or psValues[1] > sensor_value or psValues[2] > sensor_value
    left_obstacle = psValues[5] > sensor_value or psValues[6] > sensor_value or psValues[7] > sensor_value
    
    for i in range(len(list_queues)):
        list_queues[i].append(psValues[i])
        #print('list_queues[{}] len({}) {}'.format(i, len(list_queues[i]), list_queues[i]))
        if len(list_queues[i]) > 50:
            list_queues[i].pop(0)
        
    #print(list_queues)
    #print('=========================================')
    
    #  val = ds.getValue()
    for i in range(len(psValues)):
        psValue = psValues[i]
        if psValue > non_solid_sensor_value and not ( right_obstacle or left_obstacle) and not (time_to_stop_turning > datetime.now()):
            qtd_blocos_moveis_encontrados += 1;
            lightAllLeds(1)
            print('{} collided with non solid object in sensor {}'.format(datetime.now(), i))
    #print(psValues)
    # Process sensor data here.
    # initialize motor speeds at 50% of MAX_SPEED.
    percentage = 1
    front_sensor = max([psValues[0], psValues[1], psValues[6], psValues[7]])
    if front_sensor >= sensor_value:
        percentage = min_percentage
    elif front_sensor <= 100:
        percentage = 1
    else:
        valor_imaginario = front_sensor - 100
        resultado = (valor_imaginario * 100)/sensor_value
        #print('resultado {}'.format(resultado))
        percentage = min_percentage + (1 - min_percentage) * (resultado/100)
    #print(percentage)
        
        
    # Vetor de médias
    for i in range(len(list_queues)):
        avgArray[i].append(sum(list_queues[i]) / len(list_queues[i]))
        #print('list_queues[{}] len({}) {}'.format(i, len(list_queues[i]), list_queues[i]))
        if len(avgArray[i]) > 100:
            avgArray[i].pop(0)
        
    varArray = [0] * len(list_queues)
    for i in range(len(avgArray)):
        avg = sum(avgArray[i]) / len(avgArray[i])
        #print(avg)
        var = sum((x-avg)**2 for x in avgArray[i]) / len(avgArray[i])
        #print(var)
        varArray[i] = var
        
    #print(varArray)
    maior = 0
    for avg in avgArray:
        maior = max(maior, max(avg))
    #print(maior)
    
    stuck = max(varArray) < 1 and maior > 100
    if stuck:
        time_to_stop_turning = datetime.now() + timedelta(0, 2)
        #print('{} {} collision========================'.format(datetime.now(), time_to_stop_turning))
    
    # Carrin n anda até os calculos das médias terminarem
    if not max(varArray) > 0:
        percentage = 0
        
    leftSpeed  = percentage * MAX_SPEED
    rightSpeed = percentage * MAX_SPEED
    # modify speeds according to obstacles
    #if time_to_stop_turning > datetime.now():
        #print('yup turning')
    if left_obstacle or stuck or time_to_stop_turning > datetime.now():
        # turn right
        leftSpeed  = percentage * MAX_SPEED
        rightSpeed = -percentage * MAX_SPEED
    elif right_obstacle:
        # turn left
        leftSpeed  = -percentage * MAX_SPEED
        rightSpeed = percentage * MAX_SPEED
        #lightAllLeds(1)
    else:
        lightAllLeds(0)
        
    lightAllLeds(0)
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    
    #print(psValues)
    
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
