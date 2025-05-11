### Windows Implementation

#### Clone Repository
```
# Clone Repository
git clone https://github.com/keyurborad5/Multi_Agent_Reinforcement_Learning.git
```
#### Create virtual environment
```
cd Multi_Agent_Reinforcement_Learning

# Make a python environemnt
# this creats "marl" named  virtual env in ".rl-env"
python3 -m marl .rl-env

# activate the virtual environment
source .rl-env\Scripts\activate
```

#### Install all dependencies
```
pip install -r win/requirements.txt
```

#### Run the script
```
python3 group13_enpm690.py
```

#### Expected Output
 - You will observe a pybullet GUI with an environment with 1 agent and 1 cube
 - First agent will go towards the cube
 - Second agent willl get attached to it
 - Third agent will push or full the object to the next room