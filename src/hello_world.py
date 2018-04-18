import cozmo
import time


def fx(evt, **kwargs):
    cozmo.logger.info(evt, kwargs)
    cozmo.logger.info("done!!")

def cozmo_program(robot: cozmo.robot.Robot):
    cozmo.logger.info("start!")
    action = robot.say_text("World Hello ")
    action.on_completed(fx)
    action.wait_for_completed()


cozmo.run_program(cozmo_program)
