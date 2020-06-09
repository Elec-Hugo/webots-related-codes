import numpy as np
import time as t
import gym
from gym import spaces
import socket
import cv2
import numpy as np
from gym.utils import seeding

from keras.models import Sequential
from keras.layers import Dense, Activation, Flatten
from keras.optimizers import Adam


from keras.models import Sequential, Model
from keras.layers import Dense, Activation, Flatten, Input, Concatenate
from keras.optimizers import Adam

from rl.processors import WhiteningNormalizerProcessor
from rl.agents import DDPGAgent, CEMAgent
from rl.memory import SequentialMemory, EpisodeParameterMemory
from rl.random import OrnsteinUhlenbeckProcess





class CustomEnv(gym.Env):
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}
    def __init__(self, HEIGHT, WIDTH, N_CHANNELS,N_DISCRETE_ACTIONS):
        super(CustomEnv, self).__init__()

        self.HEIGHT = HEIGHT
        self.WIDTH = WIDTH
        self.N_CHANNELS = N_CHANNELS
        self.N_DISCRETE_ACTIONS = N_DISCRETE_ACTIONS

        self.action_space = spaces.Discrete(self.N_DISCRETE_ACTIONS)
        self.observation_space = spaces.Box(low=-255, high=255,
                                            shape=(self.HEIGHT, self.WIDTH, self.N_CHANNELS), dtype=np.uint8)
        ####################

        # local host IP '127.0.0.1'
        self.host = '127.0.0.2'
        # Define the port on which you want to connect
        self.port = 12345

        self.cmdStart = "S"
        self.cmdImages = "D"
        self.cmdVelocity = "V"
        self.cmdAngle = "A"
        self.cmdReset = "R"
        self.cmdSetup = "E"
        self.cmdGPS = "G"


        delay = 1
        #self.actions = [ [10, 0], [0, 10], [0, -10], [10, 10],  [10, -10], [-10, 0], [-10, 10], [-10, -10]]
        self.actions = [[10, 0,1], [10, 10,1], [10, -10,1], [-10, 0,1],
                        [-10, 10,1], [-10, -10,1],[10, 0,1], [10, 10,1],
                        [10, -10,1], [-10, 0,1], [-10, 10,1], [-10, -10,1]]
        #self.actions = [[1, 0], [1, 10], [1, -10]]

        #self.observation = 0

        self.speedInt = 0
        self.speedFlt = 0
        self.angleInt = 0
        self.angleFlt = 0
        self.n = 0

        self.s = None
        self.c = None

        self.desierdPaterns = self.loadProperPatern()

        self.conectionStatus = True

        try:
            # message sent to server
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # connect to server on local computer
            self.s.connect((self.host, self.port))
            # message you send to server
            self.s.send(self.cmdStart.encode('ascii'))
        except:
            print('Connection Error!')
            self.conectionStatus = False



        res, inputData = self.getInputImages()
        if res:
            print("Get input images problem")

        inputData = np.reshape(inputData, (3, 128, 64)).T
        size = (128, 64);
        leftImage = cv2.resize(inputData[:, :, 0].astype(np.uint8), size, cv2.INTER_CUBIC)
        rightImage = cv2.resize(inputData[:, :, 1].astype(np.uint8), size, cv2.INTER_CUBIC)

        res, gpsData = self.getGPS()
        print(gpsData)
        if res:
            print("Get GPS data problem")

        self.reward = 0
        self.counter = 0
        self.observation = gpsData #np.array(200 * self.phaseBasedMovements(leftImage, rightImage)).astype(int)

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def function2(self,lambdaF, p, img1, img2):
        # lambdaF = 250;
        sigma = lambdaF / 3;
        Re_kernel = cv2.getGaborKernel((16, 16), sigma, p, lambdaF, .5, 0, ktype=cv2.CV_32F)
        Im_kernel = cv2.getGaborKernel((16, 16), sigma, p, lambdaF, .5, np.pi / 2, ktype=cv2.CV_32F)
        filterAMP = np.multiply(Re_kernel, Re_kernel) + np.multiply(Im_kernel, Im_kernel)
        filtered_Re1 = cv2.filter2D(img1, cv2.CV_32FC3, Re_kernel) / 1.
        filtered_Re2 = cv2.filter2D(img2, cv2.CV_32FC3, Re_kernel) / 1.
        filtered_Im1 = cv2.filter2D(img1, cv2.CV_32FC3, Im_kernel) / 1.
        filtered_Im2 = cv2.filter2D(img2, cv2.CV_32FC3, Im_kernel) / 1.

        ang1 = np.angle(filtered_Im1 * 1j + filtered_Re1)
        ang2 = np.angle(filtered_Im2 * 1j + filtered_Re2)

        return ang1 - ang2, filterAMP

    def loadProperPatern(self):
        # np.save('16.ptn.npy',image)
        size = (25, 5);
        paterns = []
        for i in range(1, 13):
            paterns.append(cv2.resize(np.load('./desired images/' + str(i) + '.ptn.npy'), size))
        return paterns

    def getInputImages(self):

        res = 0
        try:
            self.s.send(self.cmdImages.encode('ascii'))
            data = self.s.recv(1024)
        except:
            res = 1
        while True:
            try:
                part = self.s.recv(1024)
            except:
                res = 1
                break
            data += part
            if len(data) >= 32767 * 3:
                # either 0 or end of data
                break

        i = 0
        inputData = []
        for number in str(data.decode('ascii')).split(','):
            i = i + 1
            if number != '':
                inputData.append(int(number))
        return res, inputData

    def phaseBasedMovements(self,inputImage1, inputImage2):
        lambdaL = 4
        output6, filter1 = self.function2(lambdaL, 0., inputImage1, inputImage2)
        output7, filter2 = self.function2(lambdaL, np.pi / 2., inputImage1, inputImage2)
        output8, filter3 = self.function2(lambdaL, np.pi / 4., inputImage1, inputImage2)
        output9, filter4 = self.function2(lambdaL, -np.pi / 4., inputImage1, inputImage2)
        output0 = output6 * output6 + output7 * output7 + output8 * output8 + output9 * output9
        return output0

    def getWebotsStatus(self,reward):
        res = self.counter >= 10
        return res

    def closestDesierdPatern(self,upsideStatus):
        size = (25, 5);
        upViwe = cv2.resize(upsideStatus.astype(np.uint8), size)
        similarities = []
        for i in range(12):
            similarities.append(max(max(cv2.matchTemplate(upViwe, self.desierdPaterns[i], cv2.TM_CCOEFF_NORMED))))

        reward = max(similarities)
        nearestStatus = np.where(similarities != reward, 0, similarities)
        nearestStatus = np.where(nearestStatus == reward, 1, nearestStatus)

        for i in range(12):
            if nearestStatus[i] == 1:
                #print("action-reward conputation")
                break

        return i+1, reward

    def sendMovementProfile(self):
        res = 0

        try:
            self.s.send(self.cmdVelocity.encode('ascii'))
            self.s.send(self.speedInt.encode('ascii'))
            self.s.send(self.speedFlt.encode('ascii'))
        except:
            print('Bye')
            res = 1
            return res

        try:
            self.s.send(self.cmdAngle.encode('ascii'))
            self.s.send(self.angleInt.encode('ascii'))
            self.s.send(self.angleFlt.encode('ascii'))
        except:
            print('Bye')
            res = 1
            return res

    def getGPS(self):
        xGPS_INT = None
        yGPS_INT = None
        zGPS_INT = None
        xGPS_FLT = None
        yGPS_FLT = None
        zGPS_FLT = None
        res = 0
        data = -100000
        try:
            self.s.send(self.cmdGPS.encode('ascii'))
            xGPS_INT = self.s.recv(5)
            xGPS_INT = int(xGPS_INT.decode('ascii'))
            yGPS_INT = self.s.recv(5)
            yGPS_INT = int(yGPS_INT.decode('ascii'))
            zGPS_INT = self.s.recv(5)
            zGPS_INT = int(zGPS_INT.decode('ascii'))
            xGPS_FLT = self.s.recv(5)
            xGPS_FLT = int(xGPS_FLT.decode('ascii'))
            yGPS_FLT = self.s.recv(5)
            yGPS_FLT = int(yGPS_FLT.decode('ascii'))
            zGPS_FLT = self.s.recv(5)
            zGPS_FLT = int(zGPS_FLT.decode('ascii'))
            data = [xGPS_INT + xGPS_FLT / 100000, yGPS_INT + yGPS_FLT / 100000, zGPS_INT + zGPS_FLT / 100000]
        except:
            res = 1

        return res , data

    def step(self, action):
        assert self.action_space.contains(action)

        res, inputData = self.getInputImages()
        if res:
            print("Get input images problem")

        #i, reward = self.closestDesierdPatern(inputData[:, :, 2].astype(np.uint8))
        #### You Must edit Observations in this class ####
        inputData = np.reshape(inputData, (3, 128, 64)).T
        size = (128, 64);
        leftImage = cv2.resize(inputData[:, :, 0].astype(np.uint8), size, cv2.INTER_CUBIC)
        rightImage = cv2.resize(inputData[:, :, 1].astype(np.uint8), size, cv2.INTER_CUBIC)


        #self.observation = np.array(200*self.phaseBasedMovements(leftImage, rightImage)).astype(int)

        self.speedInt = str(self.actions[action][0]).zfill(4)
        self.speedFlt = str(0).zfill(3)
        self.angleInt = str(self.actions[action][1]).zfill(4)
        self.angleFlt = str(0).zfill(3)

        if (self.sendMovementProfile()):
            res = 1
            print('Bye')
        t.sleep(self.actions[action][2])
        res, inputData = self.getInputImages()
        if res:
            print("Get input images problem")

        inputData = np.reshape(inputData, (3, 128, 64)).T
        size = (128, 64);


        while True:
            res, gpsData = self.getGPS()
            if res:
                pass #print("Get GPS data problem")
            else :
                break

        reward = np.exp(-1*np.power((gpsData[0] + 4.5)/100, 2))*np.exp(-1*np.power((gpsData[1] - 0.02)/100, 2))*np.exp(-1*np.power((gpsData[2] + 4.5)/100, 2))


        self.reward += reward
        print(self.reward)
        self.counter += 1
        done = self.getWebotsStatus(reward)

        info = {}

        return self.observation, self.reward, done, info

    def reset(self):

        try:
            self.s.send(self.cmdReset.encode('ascii'))
        except:
            print('Bye')
            res = 1
            return res

        try:
            self.s.send(self.cmdSetup.encode('ascii'))
        except:
            print('Bye')
            res = 1
            return res

        res, inputData = self.getInputImages()
        if res:
            print("Get input images problem")


        inputData = np.reshape(inputData, (3, 128, 64)).T
        size = (128, 64);
        leftImage = cv2.resize(inputData[:, :, 0].astype(np.uint8), size, cv2.INTER_CUBIC)
        rightImage = cv2.resize(inputData[:, :, 1].astype(np.uint8), size, cv2.INTER_CUBIC)
        #self.observation = np.array(200 * self.phaseBasedMovements(leftImage, rightImage)).astype(int)
        res, gpsData = self.getGPS()
        print(gpsData)
        if res:
            print("Get GPS data problem")

        self.reward = 0
        self.counter = 0
        return self.observation  # reward, done, info can't be included

    def render(self, mode='human'):
        pass
    def close (self):
        pass

    def __delete__(self, instance):
        self.s.close()

"""
import numpy as np

import gym
from gym import spaces
from gym.utils import seeding


class HotterColder(gym.Env):
    
    #Hotter Colder
    #The goal of hotter colder is to guess closer to a randomly selected number
    #After each step the agent receives an observation of:
    #0 - No guess yet submitted (only after reset)
    #1 - Guess is lower than the target
    #2 - Guess is equal to the target
    #3 - Guess is higher than the target
    #The rewards is calculated as:
    #(min(action, self.number) + self.range) / (max(action, self.number) + self.range)
    #Ideally an agent will be able to recognise the 'scent' of a higher reward and
    #increase the rate in which is guesses in that direction until the reward reaches
    #its maximum
    
    def __init__(self):
        self.range = 1000  # +/- value the randomly select number can be between
        self.bounds = 2000  # Action space bounds

        self.action_space = spaces.Box(low=np.array([-self.bounds]), high=np.array([self.bounds]),
                                       dtype=np.float32)
        self.observation_space = spaces.Discrete(4)

        self.number = 0
        self.guess_count = 0
        self.guess_max = 200
        self.observation = 0

        self.seed()
        self.reset()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        assert self.action_space.contains(action)

        if action < self.number:
            self.observation = 1

        elif action == self.number:
            self.observation = 2

        elif action > self.number:
            self.observation = 3

        reward = ((min(action, self.number) + self.bounds) / (max(action, self.number) + self.bounds)) ** 2

        self.guess_count += 1
        done = self.guess_count >= self.guess_max

        return self.observation, reward[0], done, {"number": self.number, "guesses": self.guess_count}

    def reset(self):
        self.number = self.np_random.uniform(-self.range, self.range)
        self.guess_count = 0
        self.observation = 0
        return self.observation
"""



env = CustomEnv(1,3,1,12)
ENV_NAME = "WebotsTest"

if (env.conectionStatus):
    print("OK")

    # Get the environment and extract the number of actions.
    np.random.seed(123)
    env.seed(123)

    nb_actions = env.action_space.n
    obs_dim = env.observation_space.shape[0]

    # Option 1 : Simple model
    model = Sequential()
    model.add(Flatten(input_shape= (1,3)))
    model.add(Dense(nb_actions))
    model.add(Activation('softmax'))

    # Option 2: deep network
    #model = Sequential()
    #model.add(Flatten(input_shape= (1, 3)))
    #model.add(Dense(5))
    #model.add(Activation('relu'))
    #model.add(Dense(2))
    #model.add(Activation('relu'))
    #model.add(Dense(1))
    #model.add(Activation('relu'))
    #model.add(Dense(nb_actions))
    #model.add(Activation('softmax'))


    print(model.summary())


    # Finally, we configure and compile our agent. You can use every built-in Keras optimizer and
    # even the metrics!
    memory = EpisodeParameterMemory(limit=1000, window_length=1)

    cem = CEMAgent(model=model, nb_actions=nb_actions, memory=memory,
                   batch_size=1, nb_steps_warmup=10, train_interval=10, elite_frac=0.5)
    cem.compile()

    # Okay, now it's time to learn something! We visualize the training here for show, but this
    # slows down training quite a lot. You can always safely abort the training prematurely using
    # Ctrl + C.
    cem.fit(env, nb_steps=100000, visualize=False, verbose=2)

    # After training is done, we save the best weights.
    cem.save_weights('cem_{}_params.h5f'.format(ENV_NAME), overwrite=True)

    # Finally, evaluate our algorithm for 5 episodes.
    cem.test(env, nb_episodes=10)
