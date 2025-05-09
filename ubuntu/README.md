# ENPM690 Final submission Group13
### Execution of files
CONTAINTS OF THE FOLDER 
1. Dockerfile
2. requirements.txt
3. group13.zip
4. README.md 
5. env.png

STEP 1: # Build the docker IMAGE from Docker file
```
# Navigate your terminal to this folder containing my Dockerfile

docker build --network=host -t enpm690_group13 .
# This will create a docker image and you can validate using
docker images
```
STEP 2: Make a docker Container using this docker Image
```
# give permissiop for GUI
xhost +local:docker
# Docker container
docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --name final_project_group13 enpm690_group13
```

Now you should me inside the docker.

STEP3: Execute my program
```
cd group13
python group13_enpm690.py 

```
Once the program starts you should see the simulation begins and there will be 5 episodes.

Terminal will print the rewards after each episodes

Expected Output is you should be able to see
- Agents spawning at different location
- Agent moves towards the cube
- Agents attached to the cube
- Agent drags the cube to the other side of the space thorugb narrow wall
- Hence episode ends

Simulation image

![image](env.png)