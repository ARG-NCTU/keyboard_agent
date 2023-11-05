# keyboard_agent

This repo can use keyboard to control the agent in training environments.

Please pull the docker image 

```
docker pull argnctu/oop
```

For the first terminal,
```
source docker_run.sh
```
other terminals
```
source docker_join.sh
```

To run the keyboard agent, please run 
```
source install_custom_gym.sh
python3 scripts/uav_lander.py
```

### To control the agent
**W** : fire the main engine  
**A** : fire right orientation engine  
**D** : fire left orientation engine  
  
you can see two windows pop up :
### lunar lander game 
<p align="center">
<img src="img/lunar_lander_cut.gif" width="500px"><br>
</p>

### plot of agent's reward  
<p align="center">
<img src="img/reward_cut.gif" width="500px" ><br>
</p>

also, you can see the result of your lunar lander in terminal :  
success or crashed

<p align="center">
<img src="img/terminal.gif" width="500px"><br>
</p>

## UAV_Lander
If you want to run uav-lander, execute the following command after entering docker. \
There's a 'pip3 install -e uav_gym' command inside the 'install_custom_gym.sh' script. It'll register our custom gymnasium named "uav_gym."
```
source install_custom_gym.sh
cd scripts/
python3 uav_lander.py
```
