import asyncio
import math

from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions, dial_direct
from viam.components.base import BaseClient, Base
from viam.components.servo import ServoClient, Servo
from viam.components.sensor import SensorClient, Sensor
from viam.components.camera import CameraClient, Camera
from viam.services.vision import DetectorConfig, DetectorType
from viam.services.types import ServiceType
from PIL import ImageDraw, Image


# Starter function. Prints out the list of resources available to the 
async def client():
  creds = Credentials(
      type='robot-location-secret',
      payload='pem1epjv07fq2cz2z5723gq6ntuyhue5t30boohkiz3iqht4')
  opts = RobotClient.Options(
      refresh_interval=0,
      dial_options=DialOptions(credentials=creds)
  )
  async with await RobotClient.at_address(
    'puppy-rover-main.60758fe0f6.local.viam.cloud:8080',
    opts) as robot:
      print('Resources:')
      print(robot.resource_names)


# Setup the robot client and return it
async def setup():
  creds = Credentials(
      type='robot-location-secret',
      payload='pem1epjv07fq2cz2z5723gq6ntuyhue5t30boohkiz3iqht4')
  opts = RobotClient.Options(
      refresh_interval=0,
      dial_options=DialOptions(credentials=creds)
  )
  robot = await RobotClient.at_address('puppy-rover-main.60758fe0f6.local.viam.cloud:8080', opts)
  return robot


# moveForward makes the base move forward
async def moveForward(robot, dist, velocity, blocking):
  base = Base.from_robot(robot, 'yahboom-base')
  await base.move_straight(dist, velocity, blocking)
    

# spinRobot makes the base spin in place (CCW is positive)
async def spinRobot(robot, angle, velocity, blocking):
  base = Base.from_robot(robot, 'yahboom-base')
  await base.spin(angle, velocity, blocking)


# setServo sets the servo at 'servo_name' to the angle at 'angle'
async def setServo(robot, servo_name, angle):
  servo = Servo.from_robot(robot, servo_name)
  await servo.move(angle)


# getServo reads the current position of the servo at 'servo_name'
async def getServoPos(robot: RobotClient, servo_name):
  servo = Servo.from_robot(robot, servo_name)
  pos = await servo.get_position()
  return pos


# takePic returns a PIL image from the camera
async def takePic(robot):
  camera = Camera.from_robot(robot, "rover_cam")
  frame = await camera.get_frame()
  return frame


# moveForward makes the base move forward
async def getUSReading(robot, tries):
  us = Sensor.from_robot(robot, "us")
  R = 0
  for i in range(tries):
    out = await us.get_readings()
    R+= out[0]
  return R/tries


# Returns the vision service with an added detector
async def getVisService(robot: RobotClient):
  vis = robot.get_service(ServiceType.VISION)
  detectors = await vis.get_detector_names()
  if len(detectors) == 0:
    paramDict = {"detect_color": "#fa7b6a", "tolerance": 0.05, "segment_size": 3000}
    vestD = DetectorConfig(name="vest-detector", type=DetectorType('color'), parameters=paramDict)
    await vis.add_detector(vestD)
    await asyncio.sleep(2)

  print("The detector names are: ", await vis.get_detector_names())
  return vis


# Draws the detection box on the frame
async def drawBox(d, frame):
  rect = [d.x_min, d.y_min, d.x_max, d.y_max]
  img1 = ImageDraw.Draw(frame)
  img1.rectangle(rect, outline="red")
  return frame


# Returns the total area covered by detection boxes
async def getBoxArea(detections):
  areaSum = 0
  for d in detections:
    area = (d.x_max - d.x_min) * (d.y_max-d.y_min)
    areaSum += area
  return areaSum


# Makes the robot face in the approximate direction of the pan servo
async def straighten(robot):
  angle = await getServoPos(robot, "pan")
  await spinRobot(robot, math.floor(2 * (angle-95)), 100, True)
  await setServo(robot, "pan", 95)




# Scan uses the above functions to scan for the vest, detect it, and 
# return the angle the vest was found at
async def scan(robot, tilt=105, panStart=45, panEnd=135):
  vestAngT = tilt
  vestAng = 0
  numPix = 0
  maxBoxSum = 0
  vis = await getVisService(robot)
  
  await setServo(robot,"tilt",tilt)
  for angle in range(panStart,panEnd, 20):
    await setServo(robot, "pan", angle)
    await asyncio.sleep(4)
    frame = await takePic(robot)
    print("got image.")
    numPix = frame.size[0]*frame.size[1]

    detections = await vis.get_detections("rover_cam", "vest-detector")
    boxSum = await getBoxArea(detections)
    if boxSum > maxBoxSum:
      maxBoxSum = boxSum
      vestAng = angle

  coverage = maxBoxSum/numPix  
  if coverage < 0.05:  
    print("Didn't see the vest")
    await setServo(robot, "pan", panEnd)
    await straighten(robot)
    await scan(robot, tilt=tilt+10)
  else: 
    await setServo(robot, "pan", panEnd)
    await straighten(robot)
    print("Found the vest! It was at pan angle ", vestAng, "and covered", coverage, "of the screen")
    return (vestAng, vestAngT, coverage)
      


async def main():

  robot = await setup()
  panAng, tiltAng, coverage = await scan(robot, tilt=105)



'''  robot = await setup()
  panAng, tiltAng, coverage = await scan(robot, tilt=105)
  await moveForward(robot, 800, 200, True)

  usCheck = await getUSReading(robot, 3)
  print("ULTRASONIC", usCheck)

  while usCheck > 0.4 and coverage < 0.4:
    panAng, tiltAng, coverage = await scan(robot, tilt =tiltAng+10)
    await moveForward(robot, 800, 200, True)
    usCheck = await getUSReading(robot, 3)
    print("ULTRASONIC:", usCheck)


  await spinRobot(robot, 500, 20, True)
  '''

if __name__ == '__main__':
  print("Starting up... ")
  #asyncio.run(client())
  asyncio.run(main())
