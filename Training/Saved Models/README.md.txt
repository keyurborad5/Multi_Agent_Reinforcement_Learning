PPO_PybulletPusdEnv_TS_100000_v2 : 
	Reward : -0.1*self._agent_steps[agent]+10000*incentive
	Success bonus : 10000
	Cube: random spawn
	Agent: random spawn
	Remarks: Agent moves smooth in Y direction but struggles in X direction
PPO_PybulletPusdEnv_TS_100000_v5 : 
	Reward : incentive (:previous_dist-dist)
	```
	 self.action_space =Discrete([5])
	```
	Success bonus : 100
	Cube: random spawn
	Agent: random spawn
	Remarks: Works best
PPO_PybulletPusdEnv_TS_100000_v6 : 
	Reward : incentive (:previous_dist-dist) + attached reward
	```
        self.action_space =MultiDiscrete([5,2])

	if grab_action == 1:
                if not self.attached[agent]:
                    self.attach_constraint_ids[agent] = create_fixed_constraint(
                            agent_id, self.cube_id, grab_distance_threshold=0.2
                        )
                    if self.attach_constraint_ids[agent] is not None:
                        self.attached[agent] = True
                        attach_reward = 100
                    else:
                        attach_reward = 0
                else:
                    attach_reward = 2
            elif grab_action == 0:
                if self.attached[agent]:
                    p.removeConstraint(self.attach_constraint_ids[agent])
                    self.attach_constraint_ids[agent] = None
                    self.attached[agent] = False
                    attach_reward = 0
                else:
                    attach_reward = 0.1
	```
	Success bonus : 100
	Cube: random spawn
	Agent: random spawn
	Remarks: Works best

PPO_PybulletPusdEnv_TS_100000_v7 : 
	Reward : incentive (:previous_dist-dist)
	```
	 self.action_space =MultiDiscrete([5,2])
	```
	Success bonus : 100
	Cube: random spawn
	Agent: random spawn
	Remarks: Works best

        