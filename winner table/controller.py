import interfaces as controller_template
from itertools import product
from typing import Tuple, List
import pickle
from random import random, randint


class State(controller_template.State):
    def __init__(self, sensors: list):
        self.sensors = sensors

    def compute_features(self) -> Tuple:
        """
        This function should take the raw sensor information of the car (see below) and compute useful features for selecting an action
        The car has the following sensors:

        self.sensors contains (in order):
            0 track_distance_left: 1-100
            1 track_distance_center: 1-100
            2 track_distance_right: 1-100
            3 on_track: 0 if off track, 1 if on normal track, 2 if on ice
            4 checkpoint_distance: 0-???
            5 car_velocity: 10-200
            6 enemy_distance: -1 or 0-???
            7 enemy_position_angle: -180 to 180
            8 enemy_detected: 0 or 1
            9 checkpoint: 0 or 1
           10 incoming_track: 1 if normal track, 2 if ice track or 0 if car is off track
           11 bomb_distance = -1 or 0-???
           12 bomb_position_angle = -180 to 180
           13 bomb_detected = 0 or 1
          (see the specification file/manual for more details)
        :return: A Tuple containing the features you defined
        """
        track_distance_left = self.sensors[0]
        track_distance_center = self.sensors[1]
        track_distance_right = self.sensors[2]
        car_velocity = self.sensors[5]
        incoming_track = self.sensors[10]
        return (track_distance_left,track_distance_center,track_distance_right,car_velocity,incoming_track)

    def discretize_features(self, features: Tuple) -> Tuple:
        """
        This function should map the (possibly continuous) features (calculated by compute features) and discretize them.
        :param features 
        :return: A tuple containing the discretized features
        """

        levels = self.discretization_levels()

        track_distance_left = ((features[0] - 1) /(100 - 1)  )  
        track_distance_left = (track_distance_left*10)//(10/levels[0])

        if track_distance_left > 3:
            track_distance_left = 3

        track_distance_center = ((features[1] - 1) /(100 - 1)  )  
        track_distance_center = (track_distance_center*10)//(10/levels[1])

        if track_distance_center > 3:
            track_distance_center = 3

        track_distance_right = ((features[2] - 1) /(100 - 1)   )   
        track_distance_right = (track_distance_right*10)//(10/levels[2])

        if track_distance_right > 3:
            track_distance_right = 3


        car_velocity = ((features[3] - 10) / (200 - 10)   )  
        car_velocity = (car_velocity*10)//(10/levels[3])

        if car_velocity > 3:
            car_velocity = 3

        incoming_track = features[4]

        return (track_distance_left,track_distance_center,track_distance_right,car_velocity,incoming_track)

    @staticmethod
    def discretization_levels() -> Tuple:
        """
        This function should return a vector specifying how many discretization levels to use for each state feature.
        :return: A tuple containing the discretization levels of each feature
        """
        track_distance_left = 4
        track_distance_center = 4
        track_distance_right = 4
        car_velocity = 4
        incoming_track = 3

        return (track_distance_left,track_distance_center,track_distance_right,car_velocity,incoming_track)

    @staticmethod
    def enumerate_all_possible_states() -> List:
        """
        Handy function that generates a list with all possible states of the system.
        :return: List with all possible states
        """
        levels = State.discretization_levels()
        levels_possibilities = [(j for j in range(i)) for i in levels]
        return [i for i in product(*levels_possibilities)]


class QTable(controller_template.QTable):
    def __init__(self):
        """
        This class is used to create/load/store your Q-table. To store values we strongly recommend the use of a Python
        dictionary.
        """
        self.table = {}
        states = State.enumerate_all_possible_states() #tuplas

        for state in states: #p cad auma das tuplas
            self.table[State.get_state_id(state)] = [0,0,0,0,0,0] #aqui inicializaremos com os valores

    def get_q_value(self, key: State, action: int) -> float:
        """
        Used to securely access the values within this q-table
        :param key: a State object 
        :param action: an action
        :return: The Q-value associated with the given state/action pair
        """

        feat = key.get_current_state()
        ident = key.get_state_id(feat)

        try:
            values = self.table[ident]
        except KeyError:
            print("[GET] State " + str(key.get_current_state()) + " [id:" + str(ident) + "]"  + " not found in table.")
            return 0.0 # or raise except?
        
        try:
            qValue = values[action]
        except IndexError:
            print("Action " + str(action) + " out of table range.")
            return 0.0 # or raise except?
        
        return qValue

    def set_q_value(self, key: State, action: int, new_q_value: float) -> None:
        """
        Used to securely set the values within this q-table
        :param key: a State object 
        :param action: an action
        :param new_q_value: the new Q-value to associate with the specified state/action pair
        :return:
        """

        feat = key.get_current_state()
        ident = key.get_state_id(feat)

        try:
            values = self.table[ident]
        except KeyError:
            print("[SET] State " + str(key.get_current_state()) + " [id:" + str(ident) + "]"  + " not found in table.")
            return
        try:
            values[action] = new_q_value
        except:
            print("Action " + str(action) + " not found.")
            return

    
    @staticmethod
    def load(path: str) -> "QTable":
        """
        This method should load a Q-table from the specified file and return a corresponding QTable object
        :param path: path to file
        :return: a QTable object
        """
        try:
            with open(path, 'rb') as input:
                qTable = QTable()
                qTable.table = pickle.load(input)
        except IOError:
            print("File " + path + "couldn't be opened.")
            return

        return qTable

    def save(self, path: str, *args) -> None:
        """
        This method must save this QTable to disk in the file file specified by 'path'
        :param path: 
        :param args: Any optional args you may find relevant; beware that they are optional and the function must work
                     properly without them.
        """

        with open(path, 'wb') as output:
            pickle.dump(self.table, output, pickle.HIGHEST_PROTOCOL)

        return


class Controller(controller_template.Controller):
    def __init__(self, q_table_path: str):
        self.i = 0
        if q_table_path is None:
            self.q_table = QTable()
        else:
            self.q_table = QTable.load(q_table_path)

    def update_q(self, new_state: State, old_state: State, action: int, reward: float, end_of_race: bool) -> None:
        """
        This method is called by the learn() method in simulator.Simulation() to update your Q-table after each action is taken
        :param new_state: The state the car just entered
        :param old_state: The state the car just left
        :param action: the action the car performed to get to new_state
        :param reward: the reward the car received for getting to new_state  
        :param end_of_race: boolean indicating if a race timeout was reached
        """

        
        
        if(self.i == 99):
            self.q_table.save("in.txt")

        alpha = 0.3
        gamma = 0.999
        newQ = (1-alpha)*self.q_table.get_q_value(old_state,action) + alpha * (reward + gamma * self.maxQ(new_state))
        self.q_table.set_q_value(old_state,action,newQ)
        

    def maxQ(self,state: State) -> int:
        max = self.q_table.get_q_value(state,0)
        for action in range(1,4):
            n = self.q_table.get_q_value(state,action)
            if n > max:
                max = n

        return max

    def compute_reward(self, new_state: State, old_state: State, action: int, n_steps: int,
                       end_of_race: bool) -> float:
        """
        This method is called by the learn() method in simulator.Simulation() to calculate the reward to be given to the agent
        :param new_state: The state the car just entered
        :param old_state: The state the car just left
        :param action: the action the car performed to get in new_state
        :param n_steps: number of steps the car has taken so far in the current race
        :param end_of_race: boolean indicating if a race timeout was reached
        :return: The reward to be given to the agent
        """

        car_velocity = (new_state.sensors[5] - 10) / (200 - 10)

        checkpoint_distance = (new_state.sensors[4] - 0) / (200 - 0)
        on_track = new_state.sensors[3]
        if(on_track == 0):
            r = -20
        else:
            r=1
        return car_velocity  + r - checkpoint_distance

    def best_action(self,state: State):
        max = self.q_table.get_q_value(state,0)
        best = 0
        for action in range(1,4):
            n = self.q_table.get_q_value(state,action)
            if n > max:
                best = action
                max = n
        
        return best

    def take_action(self, new_state: State, episode_number: int) -> int:
        """
        Decides which action the car must execute based on its Q-Table and on its exploration policy
        :param new_state: The current state of the car 
        :param episode_number: current episode/race during the training period
        :return: The action the car chooses to execute
        (Esquerda, Direita, Acelerar, Frear, AçãoNula).
        """
        if(episode_number == 99):
            self.i = 99
     
        if(random() < 0.001):
            return randint(0,3)

        return self.best_action(new_state)

