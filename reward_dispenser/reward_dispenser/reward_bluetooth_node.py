import numpy
import rclpy
import rclpy.node
import rclpy.qos
from example_interfaces.msg import String as event_message
import bluetooth
#import socket
import time
import asyncio
#from bleak import BleakScanner

DEFAULT_QOS = rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value

class Node(rclpy.node.Node):
    """
    Node that connects to the arduino bluetooth dispenser and sends reward dispense commands.

    ex: "d:1000x1" sent to the NML-NHP Reward device will dispense water for 1000ms
    """
    def __init__(self, *args, node_name='reward', bd_name='NML-NHP Reward', port=1, verbose=False, **kwargs):
        super().__init__(*args, node_name=node_name, **kwargs)
        self.bluetooth_connected = False
        self.bd_address = 'C8C9A3C77796'
        self.bd_name = bd_name
        self.bd_port = port
        self.verbose = verbose
        #self.sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM) 
        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

        
        self.initialize_parameters()
        self.initialize_subscribers()
        self.init_client()

    def initialize_parameters(self):
        # For now just need these 2 parameters for controlling reward dispensing. This may change to volume amounts
        self.declare_parameter('/duration', 1000)
        self.declare_parameter('/repeat', 1)

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
        if cmd == "success":
            # send reward command via bluetooth to reward dispenser
            duration = 2000#self.get_parameter('/duration').value
            repeat = 1#self.get_parameter('/repeat').value
            disp_msg = "d:{}x{}".format(str(duration), str(repeat))
            if self.verbose: print("Sending to {}: {}".format(self.bd_name, disp_msg))
            self.sock.send(bytes(disp_msg,'utf-8'))
            
        if cmd == "move_b":
            # send reward command via bluetooth to reward dispenser
            duration = 1000#self.get_parameter('/duration').value
            repeat = 1#self.get_parameter('/repeat').value
            disp_msg = "d:{}x{}".format(str(duration), str(repeat))
            if self.verbose: print("Sending to {}: {}".format(self.bd_name, disp_msg))
            self.sock.send(bytes(disp_msg,'utf-8'))
        else:
            if self.verbose: print("{} didn't match success".format(cmd))
        

    def init_client(self):
        # Keep trying to connect if disconnected

        #devices = BleakScanner.discover()
        #for d in devices:
        #    print(d)

        while not self.bluetooth_connected:
            # Create an array with all the MAC
            # addresses of the detected devices
            print("Searching for devices...")
            nearby_devices = bluetooth.discover_devices(duration=1, lookup_names=True,
                                            flush_cache=True, lookup_class=False)
                                                                            
            print("Found {} devices: ".format(len(nearby_devices)))

            for addr, name in nearby_devices:
                try:
                    print("   {} - {}".format(addr, name))
                except UnicodeEncodeError:
                    print("   {} - {}".format(addr, name.encode("utf-8", "replace")))
        
            # Run through all the devices found and list their name
            for addr, name in nearby_devices:
                if self.bd_name==name:
                    self.bd_address = addr
                    try:
                        print("Attempting to connect to Device: {} - {}".format(addr, name))
                        self.sock.connect((self.bd_address, self.bd_port))
                        print("Found target bluetooth device: {} - {}".format(addr, name))
                        self.bluetooth_connected = True
                    except:
                        print("could not connect to bluetooth device within time. Retrying...")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Node()
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
    
  

