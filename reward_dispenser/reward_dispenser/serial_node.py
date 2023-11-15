import time
import math 
import numpy
import rclpy
import serial
import keyboard
from rclpy.node import Node
from rclpy import qos
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from example_interfaces.msg import String as event_message, Bool
from geometry_msgs.msg import Point, Vector3

DEFAULT_QOS = qos.QoSPresetProfiles.SYSTEM_DEFAULT.value

class RewardNode(Node):
    def __init__(self, *args, node_name='reward', 
                              dev_name='Arduino Reward Pump', 
                              dev_port='/dev/ttyACM0', 
                              dev_baud=115200,
                              verbose=False,
                              **kwargs,
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
            
        """        
        super().__init__(*args, 
                         node_name=node_name, 
                         #allow_undeclared_parameters=True,
                         #automatically_declare_parameters_from_overrides=True,
                         **kwargs)
        self.dev_name = dev_name
        self.dev_port = dev_port
        self.dev_baud = dev_baud
        self.state_cb_rate = 50.0 # Hz
        self.verbose = verbose
                
        self.sock = None
        self.current_state = None
        self.last_state = None
        self.new_state_flag = False
        self.dispensed = False
        self.state_change_t = time.perf_counter()
        self.create_timer(1/self.state_cb_rate, self.timer_callback)
        
        self.initialize_parameters()
        
        self.initialize_subscribers()
        self.initialize_publishers()
        
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.connect()

    def initialize_parameters(self):
        """
        Define all parameters in this function. Time-based dispensing amounts for reward based
        on task conditions This may change to volume amounts
        """
        self.declare_parameters(
            namespace='',
            parameters=[
                ('state_monitor.target_a.hold_time', 100),
                ('state_monitor.target_a.dispense_time', 0),
                ('state_monitor.target_a.repeat', 1),
                ('state_monitor.target_b.hold_time', 200),
                ('state_monitor.target_b.dispense_time', 200),
                ('state_monitor.target_b.repeat', 1),
                ('state_monitor.target_c.hold_time', 200),
                ('state_monitor.target_c.dispense_time', 200),
                ('state_monitor.target_c.repeat', 1),
                ('state_monitor.enable', True),
                ('port', self.dev_port),
                ('baudrate', self.dev_baud),
                ('state_topic', '/task/center_out/state'),
                ('velocity_monitor.enable', False),
                ('velocity_monitor.threshold', 0.03),
                ('velocity_monitor.dispense_time', 300),
                ('manual_dispense.dispense_time', 200),
                ('enable_chatter', True),
            ])

    def initialize_subscribers(self):
        """ Create subscrptions for various topics in which rewards can be triggered from a task state or state space
        condition
        """
        self.task_state_sub = self.create_subscription(event_message,
                                                       self.get_parameter('state_topic').value,
                                                       self.state_change_cb,
                                                       DEFAULT_QOS)
        self.vel_sub = self.create_subscription(Vector3, 'robot/feedback/velocity', self.velocity_cb, DEFAULT_QOS)
        
        self.pos_sub = self.create_subscription(Point, 'robot/feedback/position', self.position_cb, DEFAULT_QOS)
        
        self.dispense_sub = self.create_subscription(Bool, 'manual_dispense', self.manual_dispense_cb, DEFAULT_QOS)
        
                                                                                                      
    def initialize_publishers(self):
        """ chatter topic for debug
        """
        self.chatter_pub = self.create_publisher(event_message, '/chatter', 1)


    def parameters_callback(self, params):
        success = False
        for param in params:
            if param.name == "port":
                if param.type_ == Parameter.Type.STRING:                    
                    success = True
                    self.dev_port = param.value
                    self.connect()
            if param.name == "baud":
                if param.type_ in [Parameter.Type.DOUBLE, Parameter.Type.INTEGER]:
                    if param.value >= 0.0 and param.value < 10000000.0:
                        success = True
                        self.dev_baud = param.value
                        self.connect()
            if param.name == "enable_chatter":
                if para.type_ == Parameter.Type.BOOL:
                    success = True
                    self.verbose = param.value
        return SetParametersResult(successful=success)
        
    
    def disconnect(self):
        """ Safely close socket connection to device  
        """
        if self.sock:
            self.sock.close()
            print("\nDevice disconnected")

    def position_cb(self, msg):
        """
        """
        pass
        
    def manual_dispense_cb(self, msg):
        """ Callback function that runs when a manual dispense request from the ROS2 network occurs.
        
        Parameters:
        -----------
        msg : Bool
            ROS2 event_message data type (Bool) with the new data
        """
        #if msg.data == True:
        self.logger('Manual dispensed command received')
        dur = self.get_parameter('velocity_monitor.enable').value
        self.duration_dispense(200, 1)
        

    def velocity_cb(self, msg):
        """ Dispense if the velocity of the robot crosses a threadholf speed value, otherwise turn off
        """
        if self.get_parameter('velocity_monitor.enable').value:
            if self.magnitude([msg.x, msg.y, msg.z]) >= self.get_parameter('velocity_monitor.threshold').value:
                #print('velocity threshold reached')
                #self.dispensed = True
                #dur = self.get_parameter('velocity_monitor.dispense_time').value
                #self.duration_dispense(dur, 1)
                self.cont_dispense(True)
            else:
                self.cont_dispense(False)
            

    def state_change_cb(self, msg):
        """ Callback function that gets executed whenever the state topic has new data published to it
        
        Parameters:
        -----------
        msg : String
            ROS2 event_message data type (String) with the new data
        """
        cmd = str(msg.data)
        if self.verbose: print("received data: {}".format(cmd))
        self.last_state = self.current_state
        self.current_state = cmd
        self.state_change_t = time.perf_counter()
        self.dispensed = False
        self.new_state_flag = True

    def logger(self, msg_str):
        """ Function to publish a message to the '/chatter' topic
        """
        msg = event_message()
        msg.data = msg_str
        self.chatter_pub.publish(msg)
    
    def timer_callback(self):
        """ The callback function which executes whenever the timer timeout is reached
        The state saved to the 'current_state' attribute is evaluated within a case structure
                
        """
        if keyboard.is_pressed("space"):
            self.manual_dispense_cb(True)
        
        #dur_dispense = False     
        #print(self.current_state)   
        cmd = str(self.current_state)
        if not cmd or cmd=='':
            return
            
        if self.verbose and self.new_state_flag:
            self.logger("entered ".format(cmd)) 

        if cmd == "intertrial":
            pass

        if cmd == "failure":
            pass

        #if cmd == "move_a":
        #    pass

        # Reinforcing the reward when the cursor is in a target
        if cmd[:4] == "hold":            
            target_letter = cmd[-1]
            hold_time = self.get_parameter('state_monitor.target_{}.hold_time'.format(target_letter)).value/1000
            #print("Time elapsed: {}".format((time.perf_counter() - self.state_change_t)))
            if (time.perf_counter() - self.state_change_t) >= hold_time and not self.dispensed:
                self.dispensed = True
                dur = self.get_parameter('state_monitor.target_{}.dispense_time'.format(target_letter)).value
                rep = self.get_parameter('state_monitor.target_{}.repeat'.format(target_letter)).value
                if self.verbose and self.new_state_flag: 
                    self.logger('dispenser triggered for state {}, dispensing {} with repeat of {}'.format(cmd, dur, rep))
                print('dispensing!')
                self.duration_dispense(dur, rep)

        if cmd[:4] == "move":
            pass
        
        """
        if cmd == "hold_a":
            if self.verbose: self.logger("entered hold_a")
            hold_time = self.get_parameter('state_monitor.target_a.dispense_time').value
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
        """
        if cmd == "success":
            # Turn off the continuous dispense at this point and provide a 
            # duration reward
            pass
            
        self.new_state_flag = False
        

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
    

    def connect(self, port=None, baudrate=None):
        """Helper function for device connection.
        The device should already be connected, but we will flush the incoming bytes
        """
        if port is None:
            port = self.get_parameter('port').value
        if baudrate is None:
            baud = self.get_parameter('baudrate').value
        try:
            self.sock = serial.Serial(port, baud, timeout=1)
            self.dev_port = port
            self.dev_baud = baud
            #print(self.sock.readline())    
            print("Successful connection to {} on port {}, baudrate {} set up".format(self.dev_name, port, baud))
            self.send("q\nq\nq\n")
        except:
            print("Error connecting to '{}' on port {}, baudrate {}".format(self.dev_name, port, baud))

    def magnitude(self, vector):
        """ Helper function definition to compute the magnitude of the vector, passed as a list of numbers
        """
        return math.sqrt(sum(pow(element, 2) for element in vector))


def main(args=None):


    # Initialize ROS. args=None
    rclpy.init()
    
    # Run a node by passing control to ROS.
    try:
        
        # Initialize the node.
        node = RewardNode(verbose=True)
        
        # Spin the node.
        try: rclpy.spin(node)
        except KeyboardInterrupt: pass
        finally: 
            node.disconnect()
            node.destroy_node()
        
    # Shut ROS down.
    finally: rclpy.shutdown()
    
    # Return.
    return

if __name__ == '__main__':
    main()
    
  

