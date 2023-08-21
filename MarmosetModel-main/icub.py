import yarp
import matplotlib.pyplot as plt
import time
import socket
import numpy
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
        self.MakeMovement()

    def jointsConn(self):
        # retrieve number of joints
        self.jnts = self.iPos.getAxes()
        self.encs = yarp.Vector(self.jnts)  # read encoders
        # self.iimp.setImpedance(4, 0.111, 0.014)
        self.ictrl.setControlMode(5, yarp.VOCAB_CM_TORQUE)
        self.ictrl.setControlMode(6, yarp.VOCAB_CM_TORQUE)

        self.iPos.setRefAcceleration(4, 180)
        self.iPos.setRefSpeed(4, 90)

        # self.iPos.setRefAcceleration(5, 50)
        self.iPos.setRefSpeed(5, 30)

    def StartPos(self):
        self.startpos = yarp.Vector(self.jnts)
        self.startpos.set(0, -25)
        self.startpos.set(1, 12)
        self.startpos.set(2, 0)
        self.startpos.set(3, 45)
        self.startpos.set(4, -60)
        self.iPos.positionMove(self.startpos.data())
        self.CheckPosition()
        time.sleep(self.interval)

    def Supination(self):
        self.sup = yarp.Vector(self.jnts)
        self.sup.set(0, -25)
        self.sup.set(1, 12)
        self.sup.set(2, 0)
        self.sup.set(3, 45)
        self.sup.set(4, -60)
        self.iPos.positionMove(self.sup.data())
        self.CheckPositionWithTremor()

    def Pronation(self):
        self.pro = yarp.Vector(self.jnts)
        self.pro.set(0, -25)
        self.pro.set(1, 19)
        self.pro.set(2, 0)
        self.pro.set(3, 45)
        self.pro.set(4, 60)
        self.iPos.positionMove(self.pro.data())
        self.CheckPositionWithTremor()

    def CheckPositionWithTremor(self):
        checkPos = False
        while checkPos != True:
            self.Tremor()
            self.WristMovement()
            #self.abcd=self.iEnc.getEncoder(4,self.encs.data())
            #self.abcd = yarp.Vector(4, self.encs.data())
            #self.tttremor.append(self.wristTremor[0])
            checkPos = self.iPos.checkMotionDone();

    def CheckPosition(self):
        checkPos = False
        while checkPos != True:
            checkPos = self.iPos.checkMotionDone();

    def Tremor(self):
        self.iTorque.getTorque(5, self.encs.data())
        value = yarp.Vector(5, self.encs.data())
        self.end = time.time()
        if self.has_pd == True:
            if (self.end - self.start) > 0.1:
                if self.tremor == 0.5 * self.PD_coeff:
                    self.tremor = -0.3 * self.PD_coeff
                else:
                    self.tremor = 0.5 * self.PD_coeff
                self.start = time.time()

            self.iTorque.setRefTorque(5, self.tremor)
            self.iTorque.setRefTorque(6, self.tremor)
        else:
            self.iTorque.setRefTorque(5, 0)
            self.iTorque.setRefTorque(6, 0)
            #print(value[0])

    def detectPD(self):
        print("-----------------------------------------------------")
        if self.PD_coeff <= 3 and self.PD_coeff >= 0.3\
                :
            self.has_pd = False
            print("has no Tremor Movements")
        else:
            self.has_pd = True
            print("has Tremor Movements")
        print("Parkinson Coefficient :", self.PD_coeff)
        if self.PD_coeff < 0.6:
             value = 0.6 - self.PD_coeff
             self.PD_coeff = 3+abs(value)
        print("NEW Coefficient :", self.PD_coeff)

    def WristMovement(self):
        self.iTorque.getTorque(5, self.encs.data())
        self.wristTremor = yarp.Vector(5, self.encs.data())
        self.IntervalTremor.append(self.wristTremor[0])

    def averageTremorInterval(self):
        for i in self.IntervalTremor:
            self.avgIntervalTremor += abs(i)
        if self.avgIntervalTremor != 0:
            self.avgIntervalTremor /= len(self.IntervalTremor)
        print("average Tremor", self.avgIntervalTremor)
        self.avgIntervalTremor = round(self.avgIntervalTremor, 2)
        print("-----------------------------------------------------")

    def closeconnection(self):
        self.coeff.close()

    def socketsend(self):
        self.coeff.send(str(self.avgIntervalTremor).encode())

    def socketreceive(self):
        self.coeff = socket.socket()
        port = 12345
        self.coeff.connect(('localhost', port))
        self.PD_coeff = float(self.coeff.recv(1024).decode())

    def MakeMovement(self):
        self.has_pd = True
        self.tremor = 0
        self.start = time.time()
        self.startint = time.time()
        self.interval = 5
        self.PD_coeff = 5
        self.IntervalTremor = []
        self.avgIntervalTremor = 0
        self.StartPos()
        self.WristMovement()
        self.tttremor = []
        count = 1
        while True:
            self.endint = time.time()
            if (self.endint - self.startint) > self.interval:
                self.socketreceive()
                self.startint = time.time()
                self.averageTremorInterval()
                self.detectPD()
                self.socketsend()
                self.IntervalTremor = []
                self.avgIntervalTremor = 0
                count += 1
            self.Supination()
            self.Pronation()
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
        # self.iPos.positionMove(0,-25)
        # self.iPos.positionMove(1,25)
        # self.iPos.positionMove(2,0)
        # self.iPos.positionMove(3,45)
        # self.iPos.positionMove(4,60)


if __name__ == '__main__':
    Move = Movement()

# prepare a property object