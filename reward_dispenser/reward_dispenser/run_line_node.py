import rclpy
from rclpy.node import Node
import time
import serial


class Node(Node):
    def __init__(self, *args, 
                    node_name='line_run', 
                    dev_name='Arduino Reward Pump',  
                    dev_port='COM6', 
                    baudrate=115200, 
                    dispense_time=60,
                    verbose=False, 
                    **kwargs):
        """
        This is the object that the single-run dispenser behavior, and connects to 
        the arduino via serial and fills/empties the reward line with
        
        Parameters:
        -----------
        node_name : str
            Specific name for ROS2 Node
        rate : int
            Publishing rate for running update commands
        dev_name: str 
            Custom device name for the cnnected device  
        dev_port : str 
            COM port to connect to (ex: 'COM6') 
        baudrate : int
            Communication baudrate for connected device 
        dispense_time : int
            Duration in seconds to dispense water
        verbose : bool
            Enable/disable verbose output for debugging on the terminal 
        
        """                 
        super().__init__(*args, node_name=node_name, **kwargs)
        self.dev_name = dev_name
        self.verbose = verbose
        self.flush_not_done = True
        self.create_timer(1, self.timer_callback)
        #self.create_timer(0.1, self.timer_callback)
        self.declare_parameter('dispense_time',40)
        self.declare_parameter('device.port', dev_port),
        self.declare_parameter('device.baudrate', baudrate),
        self.tick = time.time()

        port = self.get_parameter('device.port').value
        baud = self.get_parameter('device.baudrate').value

        self.sock = self.init_client(port, baud, False)

        param = self.get_parameter('dispense_time').value
        print("Dispensing water for {} seconds".format(param))
        self.cont_dispense(True)
        
    def timer_callback(self):
        """ Callback function that executes from the timer, checks whether dispense duration has passed
        """
        dispense_time = self.get_parameter('dispense_time').value
        if (time.time() - self.tick) > dispense_time and self.sock:
            print("Dispense done")
            time.sleep(0.5)
            self.cont_dispense(False)
            time.sleep(0.5)
            self.cont_dispense(False)
            time.sleep(0.5)
            self.cont_dispense(False)
            self.flush_not_done = False
        else:
            self.cont_dispense(True)


    def disconnect(self):
        """ Close socket connection to device      
        """
        try:
            self.sock.close()
            print("Device disconnected")
        finally: pass
        
    def cont_dispense(self, enable = False):
        """ Send command to relay to turn on/off
            ex: "d:30000x1" sent to the NML-NHP Reward device will dispense water for 30s
         """
        if enable:
            self.send("c\n")# Start relay ("continuous")
        else:            
            self.send("q\nq\n") # Stop relay ("quit")

    def send(self, msg, encode_type='utf-8'):
        """ Send a string message to the connected serial device and encode it into default utf-8 format
        """
        if self.sock:
            if self.verbose: print("Sending: {}".format(msg))
            self.sock.write(bytes(msg, encode_type))
        else:
            print("Error sending '{}' to device".format(msg))
    

    def init_client(self, port, baudrate, flush):
        """ Attempt to connect to device with port and baudrate, option to flush incoming bytes
        """
        sock = None
        try:
            sock = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
            print(self.sock.readline()) if flush else flush # empty the socket of any bytes if requested
            print("{} successfully connected to {}, baudrate={}".format(self.dev_name, port, baudrate))
        except:
            print("Error with connecting to port {}".format(port))
        
        return sock
        

def main(args=None):
    rclpy.init(args=args)
    try:   
        node = Node()
        while node.flush_not_done:
            rclpy.spin_once(node)
            
    except KeyboardInterrupt:
        print("Keyboard Interrupt - Stopping dispense")
    finally:
        node.disconnect()
        node.destroy_node()
        rclpy.shutdown()
    return
 

if __name__ == '__main__':
    main()
    
