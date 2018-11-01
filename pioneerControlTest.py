import vrep, sys, time, gym, os, math
import numpy as np
import qLearn
from gym import spaces
import ast 

class ObstacleAvoidanceVrepEnv(gym.Env):
    def __init__(self):
        # Connect to the API Server on V-Rep
        vrep.simxFinish(-1) # closing possible open connections
        self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

        if self.clientID !=-1:
            print('Connected to remote API server...')
        else:
            print('Connection not succesfull.')
            sys.exit('Could not connect')
        
        # Now I'm connected to the V-Rep Server 
        # Get motor handles
        error_codeL, self.left_motor = vrep.simxGetObjectHandle(self.clientID,
                                                                'Pioneer_p3dx_leftMotor',
                                                                vrep.simx_opmode_blocking)

        error_codeR, self.right_motor = vrep.simxGetObjectHandle(self.clientID,
                                                                 'Pioneer_p3dx_rightMotor',
                                                                 vrep.simx_opmode_blocking)
        
        # Get sensors handles
        self.usensors = []   
        for i in range(2,6):
            error, handle = vrep.simxGetObjectHandle(self.clientID,
                                                         "Pioneer_p3dx_ultrasonicSensor"
                                                         + str(i + 1),
                                                         vrep.simx_opmode_oneshot_wait)
            self.usensors.append(handle)
        
        print("Pioneer robot initialized...")
    
        # Set range for observations and actions
        obs = np.array([0] * 4)
        self.action_space = spaces.Discrete(3) 
        self.observation_space = spaces.Box(obs, obs + 2)

    def _self_observe(self):
        self.observation = [] 
        
        # Read proximity sensors and store their value
        for i in range(0,4):
            # Get obstacle position and sensor position
           error_code, res, obstacle, detHandle, _ = vrep.simxReadProximitySensor(self.clientID, self.usensors[i], vrep.simx_opmode_streaming)
           error_code, my_position = vrep.simxGetObjectPosition(self.clientID,
                                                                self.usensors[i],
                                                               -1,
                                                               vrep.simx_opmode_streaming)
           if res :
               # Compute eculedian distance between two points
               dist = np.linalg.norm(obstacle) * 10 
               self.observation.insert(i, '%.1f' % round(dist, 1))
           else:
               self.observation.insert(i, 0.0)
        '''
        #In the future we could implement a reward policy that takes in
        consideration the speed of the robot.

        # Finally append current joints velocity 
        error_code, left_motor_speed = vrep.simxGetObjectFloatParameter(self.clientID,
                                                      self.left_motor,vrep.sim_jointfloatparam_velocity,
                                                      vrep.simx_opmode_streaming)

        error_code, right_motor_speed = vrep.simxGetObjectFloatParameter(self.clientID,
                                                      self.right_motor, vrep.sim_jointfloatparam_velocity,
                                                      vrep.simx_opmode_streaming)
        self.observation.append(left_motor_speed)
        self.observation.append(right_motor_speed)
        '''
        
        #check for collision 
        error_code, collisionHandle = vrep.simxGetCollisionHandle(self.clientID, 'Collision', vrep.simx_opmode_blocking)
        error_code, collided = vrep.simxReadCollision(self.clientID, collisionHandle, vrep.simx_opmode_streaming)
        
        obs_float = list(map(float, self.observation))
        obs = [v for v in obs_float if v != 0]
        closest_sensor = 0
        if len(obs) > 0:
            closest_sensor = min(obs)
        if closest_sensor <= 0.2 and closest_sensor > 0:
            print('Sensor sum: ', closest_sensor)
            collided = True

        return collided

    def _step(self, action):
       '''
       # Save previous state
       b_obs_straight = float(self.observation[2]) + float(self.observation[3])
       b_obs_left = float(self.observation[0]) + float(self.observation[1]) + float(self.observation[2])
       b_obs_right = float(self.observation[3]) + float(self.observation[4]) + float(self.observation[5])
       '''
       
       # There are only three possible actions: left, right, forward
       if action == 0:
              # Go Forward
              vrep.simxSetJointTargetVelocity(self.clientID, self.left_motor, 8, vrep.simx_opmode_streaming)
              vrep.simxSetJointTargetVelocity(self.clientID, self.right_motor, 8, vrep.simx_opmode_streaming)
     
       elif action == 1:
            # Turn right
            vrep.simxSetJointTargetVelocity(self.clientID,
                                            self.right_motor,
                                            0,
                                            vrep.simx_opmode_streaming)
            vrep.simxSetJointTargetVelocity(self.clientID,
                                           self.left_motor,
                                           5,
                                           vrep.simx_opmode_streaming)
           

            time.sleep(0.2)
            
            vrep.simxSetJointTargetVelocity(self.clientID,
                                         self.right_motor,
                                         8,
                                         vrep.simx_opmode_streaming)
            
       elif action == 2:
            # Turn left
             vrep.simxSetJointTargetVelocity(self.clientID,
                                          self.left_motor,
                                          0,
                                          vrep.simx_opmode_streaming)
             
             vrep.simxSetJointTargetVelocity(self.clientID,
                                          self.right_motor,
                                          5,
                                          vrep.simx_opmode_streaming)
             
             time.sleep(0.2)

             vrep.simxSetJointTargetVelocity(self.clientID,
                                          self.left_motor,
                                          8,
                                          vrep.simx_opmode_streaming)

       # Observe again
       done = self._self_observe()
       sensor_sum = sum(list(map(float ,self.observation)))
       obs_straight = float(self.observation[1]) + float(self.observation[2])
       obs_left = float(self.observation[0]) + float(self.observation[1]) 
       obs_right = float(self.observation[2]) + float(self.observation[3])
       '''
       # Reward policy 1
       if not done:
           # Action going forward
           if action==0:
               # No obstacle then high reward
               if sensor_sum == 0:
                   reward = 5 
               else:
                   reward = -5 + sensor_sum
           else:
               # if turning in any direction and no obstacles, negative reward
               if sensor_sum == 0:
                   reward = 1
                # if an obstacle was close
               else:
                   reward = sensor_sum
       else:
           # Negative reward for colliding into something
           reward = -50

       '''
       # Reward policy 2
       if not done:
           # Action going forward
          if action==0:
              # No obstacle then high reward
              if obs_straight > 3 or obs_straight == 0:
                  reward = 5 
              else:
                  reward = -2 + obs_straight
          elif action == 1:
              # Turn right toward obstacle low reward
              if obs_right > 0 or obs_straight == 0:
                  reward = -5 + obs_right
              else:
                  reward = 5
          elif action == 2:
              # Turn left toward obstacle low reward
              if obs_left > 0 or obs_straight == 0:
                  reward = -5 + obs_left
              else:
                  reward = 5
       else:
           # Negative reward for colliding into something
           reward = -50
       
       return self.observation, reward, done, {}

    def _reset(self):
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
        time.sleep(0.5)
        vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)
        self.simulation_running = True
        self._self_observe()
        return self.observation

    def _destroy(self):
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
        time.sleep(1)
        vrep.simxFinish(-1)


if __name__ == '__main__':
    env = ObstacleAvoidanceVrepEnv()
    qLearn = qLearn.QLearn(actions=range(env.action_space.n), epsilon=0.3,
                           alpha=0.5, gamma=0.5)

    initial_epsilon = qLearn.epsilon

    epsilon_discount = 0.986

    tot_episodes = 100 
    print("Starting simulation...")

    for k in range(1, tot_episodes):
        observation = env.reset()
        
        qLearn.epsilon *= epsilon_discount
        
        observation, reward, done, info = env.step(env.action_space.sample())
        state = ''.join(map(str, observation))
        i = 0
        while not done:
            action = qLearn.chooseAction(state)
            observation, reward, done, info = env.step(action)
            nextState = ''.join(map(str, observation))
            qLearn.learn(state, action, reward, nextState)
            state = nextState
            #print('Observation: ', observation)
            #print('Action: ', action)
            #print('Reward: ', reward)
            print('Episode: ', k + 1)
            print('Epsilon: ', qLearn.epsilon)
            print('Frame: ', i)
            i+=1
            if i == 1500:
                break

    # Stop simulation and disconnect from V-Rep
    env._destroy()
    print('Simulation completed.')
    time.sleep(3)
    # Saving Trained model Q-Table
    f = open('trained_model3', 'w')
    f.write(str(qLearn.get_q()))
    f.close()
