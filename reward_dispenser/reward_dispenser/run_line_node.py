import rclpy
from rclpy.node import Node
import time
import serial

dispense_time = 65

class Node(Node):
    """
    Node that connects to the arduino via serial and fills/empties the reward line with water.

    ex: "d:30000x1" sent to the NML-NHP Reward device will dispense water for 30s
    """
    def __init__(self, *args, node_name='line_run', dev_name='Arduino Reward Pump',  dev_port='COM6', verbose=False, **kwargs):
        super().__init__(*args, node_name=node_name, **kwargs)
        self.serial_connected = False
        self.verbose = verbose
        self.dev_name = dev_name
        self.flush_not_done = True
        self.create_timer(0.1, self.timer_callback)
        self.tick = time.time()
        self.timer_ct = 0
        self.sock = serial.Serial(port=dev_port, baudrate=115200, timeout=0.1)
        self.init_client()

    def timer_callback(self):
        # Check whether dispense suration has passed
        tock = time.time()
        if (tock - self.tick) > dispense_time:
            print("Dispense done")
            self.flush_not_done = False
            self.cont_dispense(False)
        else:
            self.cont_dispense(True)
            self.timer_ct += 0.1
            #print(self.timer_ct)
            #print(self.timer_ct % 1)
            #if self.timer_ct % 1 == 0:
            #    print("timer callback")
            #    #self.cont_dispense(True)
    
        
    def disconnect(self):
        # Close socket connection to device      
        try:
            self.sock.close()
            print("Device disconnected")
        finally: pass
        
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
            if self.verbose: print("Sending: {}".format(msg))
            self.sock.write(bytes(msg, encode_type))
        else:
            print("Error sending '{}' to device".format(msg))
    

    def init_client(self):
        # Device should already be connected, but we will flush the incoming bytes
        #print(self.sock.readline())
        self.cont_dispense(True)
        print("{} set up".format(self.dev_name))
        print("Dispensing water for {} seconds".format(dispense_time))
        

def main(args=None):
    rclpy.init(args=args)
    try:   
        node = Node()
        while node.flush_not_done:
            rclpy.spin_once(node)
            
    except KeyboardInterrupt:
        print("Keyboard interupt. Stopping dispense")
    finally:
        node.disconnect()
        node.destroy_node()
        rclpy.shutdown()
    return
 

if __name__ == '__main__':
    main()
    