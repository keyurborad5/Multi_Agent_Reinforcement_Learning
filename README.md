# Multi-Agent Reinforcement Learning for Cooperation Without Communication in Object Maneuvering

The project aims to develop a Multi-Agent Reinforcement Learning (MARL) framework where a team of robots cooperates to maneuver a large, oddly shaped object through a confined space without explicit communication.
 - Inspired by the [Ant vs. Human Cooperation Study](https://www.pnas.org/doi/10.1073/pnas.2414274121), this project explores how robots can self-organize, adapt, and optimize strategies for collective decision-making.
 -  The main research question: "Can reinforcement learning enable a group of robots to outperform individual decision-making in a constrained environment?"

 ## Problem Description
 The Piano Movers' Problem involves maneuvering a large, oddly shaped object through a tight, constrained space using multiple agents.
 - Agents (robots) must cooperate without communication to find an optimal path.
 - Similar to how ants outperform humans under communication constraints, we aim to see if MARL-trained robots can learn optimal cooperative strategies.

 ### Tools Selection [(Selection Criterion)](Tools_select.md)
- Pybullet for physics based env
- Pytorch for MARL implementation



 ## My Approach

 MARL Implementation & Training Process(These are my initial thoughts on how I will be proceeding and might change based on results I observe)
1. State Representation:
    - Each agent observes its position, neighboring agents, object position, and applied forces.
2. Action Space:
    - 8 actions : (attached , detached to an obeject) X (Move forward, backward, left, right)
3. Reward Function:
    -  +1 for successful movement towards the goal.
    - -1 for collision with obstacles.
    - +10 when the object reaches the final destination.
    - Penalty for excessive force to encourage efficient movements.
4. Comparison With Other RL Methods
    - MADDPG vs. QMIX vs. PPO: Compare different MARL approaches.
    - With Communication vs. Without Communication: Evaluate the impact of information exchange.