import pybullet as p
import pybullet_data
import numpy as np
import time
import functools
import os
import numpy as np
import random
from gymnasium import Env
from gymnasium.spaces import Discrete, Box, Dict,Tuple, MultiDiscrete, MultiBinary
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from gymnasium import spaces
import math

from pettingzoo import AECEnv
from gymnasium.utils import seeding

from pettingzoo.utils import agent_selector

from utils.physics_helpers import (
    create_wall, create_cube_block, create_point_agent,
    distance_between_bodies, create_fixed_constraint
)


class PyBulletPushEnv(Env):
    # metadata = {"render_modes": ["human"], "name": "pybullet_push_aec_v0", "is_parallelizable": True}

    def __init__(self, render_mode=None):
        super().__init__()
        self.numb_agents = 1
        self.agents = [f"agent_{i+1}" for i in range(self.numb_agents)]
        self.possible_agents = self.agents[:]

        self.action_space =MultiDiscrete([5,2])
        # → [movement_action, grab_action]
        # movement_action: 0=noop, 1=up, 2=down, 3=left, 4=right
        # grab_action: 0 = do nothing, 1 = grab (or release if already grabbed)
         
        self.observation_space= Box(low=-3, high=30, shape=(self.numb_agents*2 + 6,), dtype=np.float32)
        self.attached = {agent: False for agent in self.agents}

        self.agent_name_mapping = {agent: i for i, agent in enumerate(self.agents)}

        # self._agent_selector = agent_selector(self.agents)
        # self.agent_selection = None
        self.render_mode = render_mode
        if self.render_mode == "human":
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)
       
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.render_mode = render_mode
        self.time_step = 1. / 60.

    def _setup_world(self):
        if p.getConnectionInfo()['isConnected'] == 0:
            if self.render_mode == "human":
                p.connect(p.GUI)
            else:
                p.connect(p.DIRECT)
            # p.connect(p.GUI)  # Use DIRECT mode for headless training (no GUI)
        p.resetSimulation()
        p.setGravity(0, 0, -9.8)
        p.loadURDF("plane.urdf")

        create_wall(0, 3.0, 3.0, 0.05)
        create_wall(0, -3.0, 3.0, 0.05)
        create_wall(-3.0, 0.0, 0.05, 3.0)
        create_wall(3.0, 0.0, 0.05, 3.0)
        create_wall(0, 1.7, 0.05, 1.3)
        create_wall(0, -1.7, 0.05, 1.3)

        
        rng = np.random.default_rng()                # optional: set a seed for repeatability
        colour_palette = [
            [1, 0, 0, 1],   # red
            [0, 0, 1, 1],   # blue
            [0, 1, 0, 1],   # green
            [1, 1, 0, 1],   # yellow
            [1, 0, 1, 1],   # magenta
            [0, 1, 1, 1],   # cyan
        ]
        # self.agent_ids = {}
        # for i in range(self.numb_agents):
        #     name = f"agent_{i+1}"

    #     # random spawn within the requested area
        
        cube_x_spawn = -0.8#rng.uniform(-2.5, -1.0)
        cube_y_spawn =0 #rng.uniform(-2.5,  2.5)
        self.cube_id = create_cube_block(cube_x_spawn, cube_y_spawn)
        x_spawn = rng.uniform(-2.5, -1.0)
        y_spawn = rng.uniform(-2.5,  2.5)

        
        self.agent_ids = {
            "agent_1": create_point_agent(x_spawn, y_spawn, color=[1, 0, 0, 1]),
            # "agent_1": create_point_agent(-0.5, -2.5, color=[0, 0, 1, 1])
        }
        self.attach_constraint_ids = {agent: None for agent in self.agents}

    def seed(self, seed=None):
        np.random.seed(seed)
        random.seed(seed)
    def reset(self,*, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            np.random.seed(seed)
            random.seed(seed)
        self._setup_world()
        

        self.agents = self.possible_agents[:]
        self.rewards = {agent: 0 for agent in self.agents}
        self._cumulative_rewards = {agent: 0 for agent in self.agents}
        self.terminations = {agent: False for agent in self.agents}
        self.truncations = {agent: False for agent in self.agents}
        self.infos = {agent: {} for agent in self.agents}
        self._agent_steps = {agent: 0 for agent in self.agents}
        self.previous_dist= 0
        self.cube_previous_dist= 0

        self.attached = {agent: False for agent in self.agents}
        self.goal_pose = [2.0,0.0]

        
        #Get observation
        agent_id = self.agent_ids[self.agents[0]]
        agent_pos, _ = p.getBasePositionAndOrientation(agent_id)
        cube_pos, _ = p.getBasePositionAndOrientation(self.cube_id)
        dist_btw_cube_agent = distance_between_bodies(agent_id, self.cube_id)
        goal_distance = np.linalg.norm(np.array(cube_pos[:2]) - np.array(self.goal_pose[:2]))


        self.obs = np.array(agent_pos[:2] + cube_pos[:2]+tuple(self.goal_pose)+(dist_btw_cube_agent,)+(goal_distance,), dtype=np.float32)
        self.info = {}
        return self.obs, self.info

    def step(self, action):
        
        movement_action = action[0]
        grab_action = action[1]
        

        # agent = self.agent_selection
        for agent in self.agents:
            if self.terminations[agent] or self.truncations[agent]:
                continue
            agent_id = self.agent_ids[agent]
            # Clears the instant rewards for the current agent from prev iteration before applying the action
            # self._clear_rewards()
            success_bonus = 100.0  # Reward bonus for reaching the goal
            vx, vy = 0, 0
            speed = 5.5
            if movement_action == 1: vy = speed
            elif movement_action == 2: vy = -speed
            elif movement_action == 3: vx = -speed
            elif movement_action == 4: vx = speed

            if grab_action == 1:
                if not self.attached[agent]:
                    self.attach_constraint_ids[agent] = create_fixed_constraint(
                            agent_id, self.cube_id, grab_distance_threshold=0.2
                        )
                    if self.attach_constraint_ids[agent] is not None:
                        self.attached[agent] = True
                        attach_reward = 12 # Reward for successfully attaching to the cube
                        self.start_time = self._agent_steps[agent] # Store the start time of the attachment
                    else:
                        attach_reward = 0 # No reward if the attachment fails
                        self.start_time = 0

                else:
                    attach_reward = 0.100 * math.exp(-0.01*(self._agent_steps[agent]-self.start_time))# Reward for holding the cube
                    

            elif grab_action == 0:
                if self.attached[agent]:
                    p.removeConstraint(self.attach_constraint_ids[agent])
                    self.attach_constraint_ids[agent] = None
                    self.attached[agent] = False
                    attach_reward = -18 # Reward for releasing the cube

                    

                else:
                    attach_reward = -0.5 # Reward for not holding the cube
                    





            p.resetBaseVelocity(agent_id, [vx, vy, 0])
            p.stepSimulation()
            if self.render_mode == "human":
                time.sleep(self.time_step)

            # Reward based on distance to cube
            dist_between_cube_agent = distance_between_bodies(agent_id, self.cube_id)
           
            # Reward based on distance to goal
            agent_pos, _ = p.getBasePositionAndOrientation(agent_id)
            cube_pos, _ = p.getBasePositionAndOrientation(self.cube_id)
            goal_pos = self.goal_pose
            goal_distance = np.linalg.norm(np.array(cube_pos[:2]) - np.array(goal_pos[:2]))
           

            if self.attached[agent]:
                if not hasattr(self, "cube_previous_dist") or self.cube_previous_dist is None:
                    cube_incentive = 0
                else:
                    cube_incentive = 300*(self.cube_previous_dist - goal_distance)
                self.cube_previous_dist = goal_distance
                expo_dist=1.0*math.exp(-0.4*goal_distance)
                expo_dist1=0.2*math.exp(-0.03*(self._agent_steps[agent]-self.start_time))
                incentive = 0

            else:
                if self.previous_dist == 0:
                    incentive = 0
                else:
                    incentive= (self.previous_dist - dist_between_cube_agent)
                expo_dist1=0.4*math.exp(-0.4*dist_between_cube_agent)
                cube_incentive = 0
                self.cube_previous_dist = None
                expo_dist=0
            self.previous_dist = dist_between_cube_agent
            

            
            self.rewards[agent] = 20*incentive+expo_dist1+attach_reward+cube_incentive+expo_dist-0.0001 * self._agent_steps[agent] 
            self._cumulative_rewards[agent]+=self.rewards[agent]
           
            goal_threshold = 0.2
            agent_pos, _ = p.getBasePositionAndOrientation(agent_id)
            self.obs = np.array(agent_pos[:2] + cube_pos[:2]+tuple(self.goal_pose)+(dist_between_cube_agent,)+(goal_distance,), dtype=np.float32)

            # self.obs = np.array(agent_pos[:2] + cube_pos[:2]+ (dist_between_cube_agent,), dtype=np.float32)
            cube_to_agent_distance = np.linalg.norm(np.array(agent_pos[:2]) - np.array(cube_pos[:2]))

            if  goal_distance<goal_threshold:#self.attached[agent]: #cube_to_agent_distance < goal_threshold:
                self.rewards[agent] += success_bonus
                self.terminations[agent] = True
                self.truncations[agent] = False
                print("Terminnation due to goal reached at step: ", self._agent_steps[agent])
                break

            else:
                self.terminations[agent] = False

          
            self._agent_steps[agent] += 1
            if self._agent_steps[agent] >= 500:
                self.terminations[agent] =True
                break
            

            # Info dict
            self.info = {}
            # print("AT TP ", self._agent_steps[agent]," CR: ", self._cumulative_rewards[agent], " RW: ", self.rewards[agent], " IN: ", incentive,"exp_dist1: ",expo_dist1, " AR: ", attach_reward, "EXp: ",expo_dist, "CIN: ", cube_incentive)

            
            
            # self.agent_selection = self._agent_selector.next()
            
            if self.render_mode == "human":
                self.render()
        return self.obs, self.rewards[agent], self.terminations[agent], self.truncations[agent], self.info


    @functools.lru_cache(maxsize=None)
    def action_space(self, agent):
        # We can seed the action space to make the environment deterministic.
        return Discrete(5, seed=self.np_random_seed)
    def observe(self, agent):
        agent_ids = self.agent_ids.copy()
        current_agent_pos, _ = p.getBasePositionAndOrientation(agent_ids[agent])
        cube_pos, _ = p.getBasePositionAndOrientation(self.cube_id)

        other_agent_poses = []
        for other_agent in self.agents:
            if other_agent != agent:
                pos, _ = p.getBasePositionAndOrientation(agent_ids[other_agent])
                other_agent_poses.extend(pos[:2])

        obs = np.array(
            list(current_agent_pos[:2]) + list(cube_pos[:2]) + other_agent_poses,
            dtype=np.float32
        )
        return obs

    def render(self):
        pass

    def close(self):
        p.disconnect()
PPO_Path = os.path.join("Training", "Saved Models", "PPO_PybulletPusdEnv_TS_100000_v9")


env=PyBulletPushEnv(render_mode="human") # Creating the environment
model = PPO.load(PPO_Path) # Load the model
episode=5
for episode in range(1, episode + 1): #for loop to run the simulation for number of episodes
    # Reset the environment for each episode
    obs, info = env.reset()
    # Initialize done variable to False which is used to raise flage of termination
    done = False
    # Variable to keep track of the score; Score is cumulative reward
    score=0
    # Loop until the episode is done (or not terminated)
    #we could have also used max reward or max time steps to terminate the episode
    while not done:
        # Sample a random action from the action space
        # In a real scenario, you would use a trained model to predict the action
        action, _= model.predict(obs)  # Random action for demonstration
        # print(f"Action: {action}")
        # Take a step in the environment with the sampled action
        # The step function returns the next observation, reward, done flag, info dictionary and the time step
        obs, reward, done, truncated, info = env.step(action)
        # render the environment to visualize the action taken
        # print(f"Episode: {episode}, obs: {obs}, Done: {done}, Reward: {reward}, Truncated: {truncated}")

        env.render()
        #keep the score of the episode
        score += reward
        # print(f"Episode: {episode}, Score: {score}, Action: {action}, Reward: {reward}")
    print(f"Episode {episode} finished with score: {score}")

env.close() 