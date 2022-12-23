import numpy
import rclpy
import rclpy.node
import rclpy.qos
from example_interfaces.msg import String as event_message
import time
import serial

DEFAULT_QOS = rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value

class RewardNode(rclpy.node.Node):
    """
    Node that connects to the arduino bluetooth dispenser and sends reward dispense commands.

    ex: "d1000x1" sent to the NML-NHP Reward device will dispense water for 1000ms
    """
    def __init__(self, *args, node_name='reward', dev_name='Arduino Reward Pump', dev_port='COM6', verbose=False, **kwargs):
        super().__init__(*args, node_name=node_name, **kwargs)
        self.serial_connected = False
        self.dev_name = dev_name
        self.dev_port = dev_port
        self.verbose = verbose
        self.sock = serial.Serial(port=dev_port, baudrate=115200, timeout=1)
        self.current_state = None
        self.last_state = None
        self.reset_dispense = True
        self.state_change_t = time.perf_counter()
        self.initialize_parameters()
        self.initialize_subscribers()
        self.create_timer(0.1, self.timer_callback)
        self.init_client()

    def initialize_parameters(self):
        # Define all time-based dispensing amounts for reward based on task conditions
        # This may change to volume amounts
        self.declare_parameter('/dispense_hold_time/target_a', 0.00)
        self.declare_parameter('/dispense_hold_time/target_b', 0.30)
        self.declare_parameter('/dispense_hold_time/target_c', 0.40)
        self.declare_parameter('/dispense_duration/target_a', 000)
        self.declare_parameter('/dispense_duration/target_b', 300) # outer target, more reward
        self.declare_parameter('/dispense_duration/target_c', 400) # outer target, more reward
        self.declare_parameter('/repeat/target_a', 1)
        self.declare_parameter('/repeat/target_b', 1)
        self.declare_parameter('/repeat/target_c', 1)

    def initialize_subscribers(self):
        # Just need subscription for 'task/center_out/state' topic
        self.task_state_sub = self.create_subscription(event_message,
                                                       '/task/center_out/state',
                                                       self.state_changed,
                                                       DEFAULT_QOS)

    def disconnect(self):
        # Close socket connection to device      
        try:
            self.sock.close()
            print("Device disconnected")
        finally: pass

    def state_changed(self, msg):
        cmd = str(msg.data)
        if self.verbose: print("received data: {}".format(cmd))
        self.last_state = self.current_state
        self.current_state = cmd
        self.reset_dispense = True
        #self.cont_dispense(False)
        self.state_change_t = time.perf_counter()
        
    def timer_callback(self):
        dur_dispense = False        
        cmd = self.current_state

        if cmd == "intertrial":
            pass

        if cmd == "failure":
            pass

        if cmd == "move_a":
            if self.reset_dispense:
                self.reset_dispense = False
                self.cont_dispense(False)

        # Reinforcing the reward when the cursor is in the target
        if cmd == "hold_a":
            hold_time = self.get_parameter('/dispense_hold_time/target_a').value
            if (time.perf_counter() - self.state_change_t) >= hold_time:                
                self.cont_dispense(True)

        if cmd == "move_b":
            if self.reset_dispense:
                self.reset_dispense = False
                self.cont_dispense(False)

        if cmd == "hold_b":
            hold_time = self.get_parameter('/dispense_hold_time/target_b').value
            if (time.perf_counter() - self.state_change_t) >= hold_time:                
                self.cont_dispense(True)
                
        if cmd == "move_c":
            if self.reset_dispense:
                self.reset_dispense = False
                self.cont_dispense(False)

        if cmd == "hold_c":
            hold_time = self.get_parameter('/dispense_hold_time/target_c').value
            if (time.perf_counter() - self.state_change_t) >= hold_time:                
                self.cont_dispense(True)

        if cmd == "success":
            # Turn off the continuous dispense at this point and provide a 
            # duration reward
            self.cont_dispense(False)
            dur_dispense = True
            duration = self.get_parameter('/dispense_duration/target_c').value
            repeat = self.get_parameter('/repeat/target_c').value
            
        if dur_dispense and not self.reset_dispense:
            # send reward command via serial to reward dispenser
            self.reset_dispense = True
            disp_msg = "d{}x{}\n".format(str(duration), str(repeat))
            if self.verbose: print("Sending to {}: {}".format(self.dev_name, disp_msg))
            self.send(disp_msg)
        

        
    def cont_dispense(self, enable = False):
        # Send command to relay to turn on/off
        if enable:            
            self.send("c\n")# Start relay ("continuous")
        else:            
            self.send("q\n") # Stop relay ("quit")

    def send(self, msg, encode_type='utf-8'):
        # Send a string message to the connected serial device and encode it 
        # into default utf-8 format
        if self.sock:
            self.sock.write(bytes(msg, encode_type))
        else:
            print("Error sending '{}' to device".format(msg))
    

    def init_client(self):
        # Device should already be connected, but we will flush the incoming bytes
        #print(self.sock.readline())    
        print("{} set up".format(self.dev_name))
        self.send("q\n")
        

def main(args=None):
    rclpy.init(args=args)
    try:
        node = RewardNode(verbose=False)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.disconnect()
            node.destroy_node()
    finally:
        rclpy.shutdown()
    return


if __name__ == '__main__':
    main()
    
  

