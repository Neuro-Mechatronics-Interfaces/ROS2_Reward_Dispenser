import numpy
import rclpy
import rclpy.node
import rclpy.qos
from example_interfaces.msg import String as event_message
import time
import serial

DEFAULT_QOS = rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value

class RewardNode(rclpy.node.Node):
    def __init__(self, *args, node_name='reward', 
                              dev_name='Arduino Reward Pump', 
                              dev_port='COM6', 
                              verbose=True,
                              **kwargs
                ):
        """
        This object creates a serial connection with an arduino and sends it reward dispense commands while listening to the ROS2 topic '/task/center_out/state'. 
        Reward conditions are based on the task states; as an example if the task state reads 'hold_a' then a dispense command gets sent with the
        amount and number of repetitions.
        
        The microcontroller will also listen to a digital input from a button press which will also enable the dispense relay
    
        Parameters:
        -----------
        node_name : str
            Unique name associated for the ROS2 node
        dev_name : str
            Connected Device assigned name
        dev_port : str
            Locally assigned COM port to connect to (Must be changed to match PC)
        verbose : bool
            Enable/disable verbose output for terminal debugging
    
        Notes:
            Example: "d1000x1" sent to the NML-NHP Reward device will dispense water for 1000ms
            
        """        super().__init__(*args, node_name=node_name, **kwargs)
        self.serial_connected = False
        self.dev_name = dev_name
        self.dev_port = dev_port
        self.verbose = verbose
        self.sock = serial.Serial(port=dev_port, baudrate=115200, timeout=1)
        self.current_state = None
        self.last_state = None
        self.dispensed = False
        self.state_change_t = time.perf_counter()
        self.initialize_parameters()
        self.initialize_subscribers()
        self.initialize_publishers()
        self.create_timer(0.1, self.timer_callback)
        self.init_client()

    def initialize_parameters(self):
        """
        Define all time-based dispensing amounts for reward based on task conditions
        This may change to volume amounts
        """
        self.declare_parameter('/dispense_hold_time/target_a', 0.10)
        self.declare_parameter('/dispense_hold_time/target_b', 0.20)
        self.declare_parameter('/dispense_hold_time/target_c', 0.20)
        self.declare_parameter('/dispense_duration/target_a', 0)
        self.declare_parameter('/dispense_duration/target_b', 300) # outer target, more reward
        self.declare_parameter('/dispense_duration/target_c', 400) # outer target, more reward
        self.declare_parameter('/repeat/target_a', 1)
        self.declare_parameter('/repeat/target_b', 1)
        self.declare_parameter('/repeat/target_c', 1)

    def initialize_subscribers(self):
        """ Just need subscription for 'task/center_out/state' topic
        """
        self.task_state_sub = self.create_subscription(event_message,
                                                       '/task/center_out/state',
                                                       self.state_changed,
                                                       DEFAULT_QOS)
                                                       
    def initialize_publishers(self):
        """ chatter topic for debug
        """
        self.chatter_pub = self.create_publisher(event_message, '/chatter', 1)

    def disconnect(self):
        """ Close socket connection to device  
        """
        try:
            self.sock.close()
            print("Device disconnected")
        finally: pass

    def state_changed(self, msg):
        """ Callback function that gets executed whenever the state topic has new data published to it
        
        Parameters:
        -----------
        msg : String
            ROS2 event_message data type (String) with the new data
        """
        if self.verbose: print("State changed!")
        cmd = str(msg.data)
        if self.verbose: print("received data: {}".format(cmd))
        self.last_state = self.current_state
        self.current_state = cmd
        self.state_change_t = time.perf_counter()
        self.dispensed = False

    def logger(self, msg_str):
        """ Function to publish a message to the '/chatter' topic
        """
        msg = event_message()
        msg.data = msg_str
        self.chatter_pub.publish(msg)
    
    def timer_callback(self):
        """ The callback function which executes whenever the timer timeout is reached (10Hz)
        The state saved to the 'current_state' attribute is evaluated and passed to a case structure
        
        
        """
        #dur_dispense = False        
        cmd = self.current_state

        if cmd == "intertrial":
            pass

        if cmd == "failure":
            pass

        if cmd == "move_a":
            pass

        # Reinforcing the reward when the cursor is in the target
        if cmd == "hold_a":
            if self.verbose: self.logger("entered hold_a")
            hold_time = self.get_parameter('/dispense_hold_time/target_a').value
            if (time.perf_counter() - self.state_change_t) >= hold_time and not self.dispensed: 
                self.dispensed = True
                if self.verbose: self.logger("completed hold_a")
                dur = self.get_parameter('/dispense_duration/target_a').value
                rep = self.get_parameter('/repeat/target_a').value
                self.duration_dispense(dur, rep)

        if cmd == "move_b":
            pass

        if cmd == "hold_b":
            if self.verbose: self.logger("entered hold_b")
            hold_time = self.get_parameter('/dispense_hold_time/target_b').value
            if (time.perf_counter() - self.state_change_t) >= hold_time and not self.dispensed: 
                self.dispensed = True
                if self.verbose: self.logger("completed hold_b")
                dur = self.get_parameter('/dispense_duration/target_b').value
                rep = self.get_parameter('/repeat/target_b').value
                self.duration_dispense(dur, rep)
                
        if cmd == "move_c":
            pass

        if cmd == "hold_c":
            if self.verbose: self.logger("entered hold_c")
            hold_time = self.get_parameter('/dispense_hold_time/target_c').value
            if (time.perf_counter() - self.state_change_t) >= hold_time and not self.dispensed: 
                self.dispensed = True
                if self.verbose: self.logger("completed hold_c")
                dur = self.get_parameter('/dispense_duration/target_c').value
                rep = self.get_parameter('/repeat/target_c').value
                self.duration_dispense(dur, rep)

        if cmd == "success":
            # Turn off the continuous dispense at this point and provide a 
            # duration reward
            pass

    def duration_dispense(self, dur, rep):
        """ Function that creates and handles dispense messages with the duration and number of repetitions
        
        Parameters:
        -----------
        dur : int
            The length in milliseconds to enable the relay dispense
        rep : int
            The number of repetitions for the dispense time to repeat        
        
        """
        self.dispensed = True
        if dur!=0 or rep!=0:
            disp_msg = "d{}x{}\n".format(str(dur), str(rep))
            if self.verbose: print("Sending to {}: {}".format(self.dev_name, disp_msg))
            self.send(disp_msg)    
        
    def cont_dispense(self, enable = False):
        """ Send command to relay to turn on/off
        """
        if enable:            
            self.send("c\n")# Start relay ("continuous")
        else:            
            self.send("q\n") # Stop relay ("quit")

    def send(self, msg, encode_type='utf-8'):
        """ Send a string message to the connected serial device and encode it 
        into default utf-8 format
        
        Parameters:
        -----------
        msg : str
            Message to send to the microcontroller
        encode_type : str
            Encoding type to convert message into bytes, usually 'utf-8'
        """
        if self.sock:
            self.sock.write(bytes(msg, encode_type))
        else:
            print("Error sending '{}' to device".format(msg))
    

    def init_client(self):
        """Helper function for device connection.
        The device should already be connected, but we will flush the incoming bytes
        """
        #print(self.sock.readline())    
        print("{} set up".format(self.dev_name))
        self.send("q\nq\nq\n")
        

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
    
  

