import yarp;
import time;
import numpy as np;
import socket;
import errno;
from socket import error as SocketError;
import matplotlib.pyplot as plt;
from scipy.fft import fft, ifft, fftfreq
import select;


class Movement():
    def __init__(self):
        yarp.Network.init();
        props = yarp.Property()
        props.put("device", "remote_controlboard")
        props.put("local", "/client/left_arm")
        props.put("remote", "/icubSim/left_arm")

        armDriver = yarp.PolyDriver(props)

        self.iPos = armDriver.viewIPositionControl()
        self.iVel = armDriver.viewIVelocityControl()
        self.iTorque = armDriver.viewITorqueControl()
        self.iEnc = armDriver.viewIEncoders()
        self.iimp = armDriver.viewIImpedanceControl()
        self.ictrl = armDriver.viewIControlMode()

        self.jointsConn()
        self.setSpeed()
        self.MakeMovement()

    def jointsConn(self):
        # retrieve number of joints
        self.jnts = self.iPos.getAxes()
        print ('Controlling', self.jnts, 'joints')

        self.encs = yarp.Vector(self.jnts)  
        self.iEnc.getEncoders(self.encs.data())

    def setSpeed(self):
        self.iPos.setRefSpeed(1, 10)
        self.iPos.setRefAcceleration(1, 50)

        self.iPos.setRefSpeed(4, 180)
        self.iPos.setRefAcceleration(4,60)

        self.iPos.setRefSpeed(5, 50)
        self.iPos.setRefAcceleration(5,20)

        self.prono_pos = yarp.Vector(self.jnts)
        self.prono_pos.set(0, -23.1)
        self.prono_pos.set(1, 24)
        self.prono_pos.set(2, 14.04)
        self.prono_pos.set(3, 64.61)
        self.prono_pos.set(4, 60)

        self.supin_pos = yarp.Vector(self.jnts)
        self.supin_pos.set(0, -23.1)
        self.supin_pos.set(1, 16)
        self.supin_pos.set(2, 14.04)
        self.supin_pos.set(3, 64.61)
        self.supin_pos.set(4, -60)
        self.start = time.time()
        self.last_samp_t = self.start
        self.last_trem_t = self.start
        self.tremor_state = 0
        self.tremor_total = []
        self.PD_state = 0

    def generate_movement(self):
        while time.time() < self.start + 9000*0.01: # while max time of simulation has not been reached
            socket_as_list = [self.coeff]

            readable, _, _ = select.select(socket_as_list, [], [], 0.1)

            if self.coeff in readable: # if we received a new coefficient
                try:
                    self.tremor_state = np.array([self.tremor_total]).mean()
                    self.tremor_total = []
                    self.coeff.send(str(self.tremor_state).encode())
                    print('TREMOR MEAN SENT :',self.tremor_state)
                    self.tremor_state = 0
                    # Receive PD_state value from the brain model
                    self.PD_state = float(self.coeff.recv(1024).decode())
                    print('PD COEFF RECEIVED :', self.PD_state)


                except (socket.error, ValueError) as e:
                    print('Error receiving data:', e)

            print('Time : ', time.time()-self.start)

            # Moving to pronation position
            print('Moving pronation position')
            self.iPos.positionMove(self.prono_pos.data())

            while self.checkReachPos('sup', None, 55, 4) == False:
                self.check_t = time.time()
                if self.check_t- self.last_samp_t >= 0.01:
                    # Collect joints data
                    self.joint4_pos.append(self.getPos(4))
                    self.joint5_pos.append(self.getPos(5))
                    self.tremor_total.append(self.getPos(5))
                    self.t.append(self.check_t-self.start)
                    self.last_samp_t = self.check_t
                    #print('Joint 4 angle : ', joint4_pos[-1])
                if self.PD_state > 1: # if not healthy
                    if self.check_t - self.last_trem_t >= 0.2:
                        # Generate tremors whose amplitude is dicted by PDstate
                        trem_amp = 2.5/8 * self.PD_state + 0.2
                        self.generateSingleTremor(trem_amp,5)
                        self.last_trem_t = self.check_t


            # Moving to supination position
            print('Moving supination position')
            self.iPos.positionMove(self.supin_pos.data())

            while self.checkReachPos('inf', -55, None, 4) == False:
                self.check_t = time.time()
                if self.check_t- self.last_samp_t >= 0.01:
                    # Collect joints data
                    self.joint4_pos.append(self.getPos(4))
                    self.joint5_pos.append(self.getPos(5))
                    self.tremor_total.append(self.getPos(5))
                    self.t.append(self.check_t-self.start)
                    self.last_samp_t = self.check_t
                    #print('Joint 4 angle : ', joint4_pos[-1])
                if self.PD_state > 1: # if not healthy
                    if self.check_t - self.last_trem_t >= 0.01:
                        # Generate tremors whose amplitude is dicted by PDstate
                        trem_amp = 2.2/8 * self.PD_state + 0.2
                        self.generateSingleTremor(trem_amp,5)
                        self.last_trem_t = self.check_t

        print('t len : ', len(self.t), 'pos4 len : ', len(self.joint4_pos))


    def socketreceive(self):
        self.coeff = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_host = 'localhost'  # Replace with the appropriate IP address or hostname
        receiver_port = 12345 
        self.coeff.connect((server_host, receiver_port))
        print('Receiver socket connected')

    def StartPos(self):
        self.startpos = yarp.Vector(self.jnts)
        self.startpos.set(0, -23.1)
        self.startpos.set(1, 24)
        self.startpos.set(2, 14.04)
        self.startpos.set(3, 64.61)
        self.startpos.set(4, 60)
        self.iPos.positionMove(self.startpos.data())
        while self.checkReachPos('sup', None, 58, 4) == False:
            print('Going to start pos...')
        print('Start pos (pronation) reached, beginning movement in 3s')
        time.sleep(3)

    def getPos(self, jnt): # get the Pos of all joints
        self.iEnc.getEncoders(self.encs.data())
        return yarp.Vector(jnt+1, self.encs.data())[jnt]

    def checkReachPos(self,comp, boundary_inf, boundary_sup, jnt): # Check if the pos of a joint has reached a particular value
        if comp == 'inf':
            if self.getPos(jnt) <= boundary_inf:
                 return True
        elif comp == 'sup':
            if self.getPos(jnt) >= boundary_sup:
                 return True
        elif comp == 'between':
            if self.getPos(jnt) <= boundary_inf and self.getPos(jnt) >= boundary_sup:
                 return True
        return False
     
    def generateSingleTremor(self, amp, jnt):

    # "Up" part of the tremor
        self.iPos.positionMove(jnt, amp)

        while self.checkReachPos('sup', None, amp*0.8, jnt) == False:
           check_t = time.time()
           if check_t- self.last_samp_t >= 0.01:
                #print('Tremor pos (up): ', getPos(4))
                # Collect joints data
                self.joint4_pos.append(self.getPos(4))
                self.joint5_pos.append(self.getPos(5))
                self.tremor_total.append(self.getPos(5))
                self.t.append(check_t-self.start)
                self.last_samp_t = check_t
         #print('Tremor up pos reached')

         # "Down" part of the tremor
        self.iPos.positionMove(jnt, -amp)

        while self.checkReachPos('inf', -amp*0.8, None, jnt) == False:
            check_t = time.time()
            if check_t- self.last_samp_t >= 0.01:
                #print('Tremor pos (down): ', getPos(4))
                # Collect joints data
                self.joint4_pos.append(self.getPos(4))
                self.joint5_pos.append(self.getPos(5))
                self.tremor_total.append(self.getPos(5))
                self.t.append(check_t-self.start)
                self.last_samp_t = check_t
        #print('Tremor down pos reached')

        # Back to 0
        self.iPos.positionMove(jnt, 0)


    def closeconnection(self):
        self.coeff.close()

    def MakeMovement(self):
        self.interval = 5
        count = 1
        self.joint4_pos = []
        self.joint5_pos = []
        self.t = []
        self.socketreceive()
        self.StartPos()
        self.generate_movement()

        #fft_result = numpy.fft.fft(self.tttremor)
        #sampling_rate = 1000
        #n = len(self.tttremor)
        #frequency_axis = numpy.fft.fftfreq(n, d=1/sampling_rate)
        #positive_freq_mask = frequency_axis > 0
        #plt.plot(frequency_axis[positive_freq_mask], numpy.abs(fft_result)[positive_freq_mask])
        #plt.plot(range(0,len(self.tttremor)),self.tttremor)
        #plt.title("Fast Fourrier Transform of themors")
        #plt.ylabel("Magnitude")
        #plt.xlabel("Frequency(Hz)")
        #plt.show()

        # position = self.iEnc.getEncoders(self.encs.data())
        # print("Value of Joint : ")
        # print(yarp.Vector(self.jnts, self.encs.data())[6])


if __name__ == '__main__':
    Move = Movement()

# prepare a property object