# keyboard_agent

This repo can use keyboard to control the agent in training environments.

Please pull the docker image 

```
docker pull argnctu/keyboard_agent:latest
```

for terminal 1
```
source docker_run.sh
```
other terminals
```
source docker_join.sh
```

to run the keyboard agent, please run 
```
python3 key_press.py
```

you can see two windows pop up :
### lunar lander game 


### plot of agent's reward  


also, you can see the result of your lunar lander :
success or crashed
