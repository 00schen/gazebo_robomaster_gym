from robomaster_env import *
from copy import deepcopy
import scipy.spatial.distance.euclidean as L2

class CloseQuarter(RobomasterEnv):
    """
    Centers two opposing robots on a random section of
    battlefield. Objective is to inflict damage on
    opponent. Game ends when one dies or when one travels
    a set distance away from opponent.
    Reward - robots will cache a reward proportional to
    damage inflicted on opponent, and the opponent will
    lose this health. Upon leaving radius of conflict, 
    game will terminate and agents will gain cached reward.
    If a robot dies, it will gain no reward.
    """
    def __init__(self):
        """
        self._robot_hp - [r1_hp, r2_hp, r3_hp, r4_hp]
        self._num_projectiles = [[50], [50], [50], [50]]
        self._barrel_heat = [[0], [0], [0], [0]]
        self._shoot -
        self._robot_effects
        """
        super.__init__()
        self.rewards = [0, 0]
        self.damage_to_reward = lambda damage: damage
        self.radius = 2000


    def step(self, action1, action2):
        prev_robot_hp = deepcopy(self._robot_hp)
        state,_,_,_ = super.step(action1, action2)
        damage = subtract(self.prev) # how?
        self.rewards[0] += self.damage_to_reward(damage[1])
        self.rewards[1] += self.damage_to_reward(damage[0])
        distance = L2(self.robot_coords[0], self.robot_coords[1])
        if distance > radius or 0 in self._robot_hp:
            rewards, done = deepcopy(self.rewards), True
        else:
            rewards, done = [0,0], False
        
        return state, reward, done, _



