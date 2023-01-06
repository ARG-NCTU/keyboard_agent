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
<p align="center">
<img src="img/lunar_lander_cut.gif"><br>
</p>

### plot of agent's reward  
<p align="center">
<img src="img/reward_cut.gif"><br>
</p>

also, you can see the result of your lunar lander :
success or crashed

<p align="center">
<img src="img/terminal.gif"><br>
</p>
