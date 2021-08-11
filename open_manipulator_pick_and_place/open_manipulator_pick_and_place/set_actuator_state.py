from rclpy.node import Node

from open_manipulator_msgs.srv import SetActuatorState

class ActuatorController(Node):
    def __init__(self):
        super().__init__("actuator_state")

        self.client = self.create_client(SetActuatorState, 'set_actuator_state')
        self.request = SetActuatorState.Request()

    def send_request(self):
        future = self.client.call_async(self.request)
        # future = self.client.call(self.request)

    def set_request(self, command: bool):
        if command == True:
            print(command)
            self.request.set_actuator_state = True
        else:
            print(command)
            self.request.set_actuator_state = False
        self.send_request()