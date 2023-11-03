# Tabletennis Robot for CAIR Engineering Competition of  2022 RoboComRobot Developer Competition (2022机器人开发者大赛CAIR工程竞赛——乒乓乐扣) 
[![A Python Robotics Package](https://raw.githubusercontent.com/petercorke/robotics-toolbox-python/master/.github/svg/py_collection.min.svg)](https://github.com/petercorke/robotics-toolbox-python)        [![Powered by Spatial Maths](https://raw.githubusercontent.com/petercorke/spatialmath-python/master/.github/svg/sm_powered.min.svg)](https://github.com/petercorke/spatialmath-python)   [![QUT Centre for Robotics Open Source](https://github.com/qcr/qcr.github.io/raw/master/misc/badge.svg)](https://qcr.github.io)   [![PyPI version](https://badge.fury.io/py/roboticstoolbox-python.svg)](https://badge.fury.io/py/roboticstoolbox-python)  [![Anaconda version](https://anaconda.org/conda-forge/roboticstoolbox-python/badges/version.svg)](https://anaconda.org/conda-forge/-python)  ![PyPI - Python Version](https://img.shields.io/pypi/pyversions/roboticstoolbox-python.svg)


## 1. Introduction

&emsp;&emsp;We design this tabletennis robot in webot virtual environment to participate in CAIR Engineering Competition of 2022 RoboComRObot Developer Competition. We constructed the appropriate virtual environment in webots by ourselves and we changed some parts of [robotic-toolbox-python](https://github.com/petercorke/robotics-toolbox-python)(a github project contributed by [petercorke](https://github.com/petercorke)) to achieve the path planning for robotic arm. 
&emsp;&emsp;At last, we won third place in this competition.

  

## 2. Teamwork
 &emsp;&emsp; As the team leader, I was responsible for the planning and scheduling of the entire project design. 
 &emsp;&emsp;First of all, I collected relevant literature and information on the Internet, and then I organized a group discussion, letting each group member provide their own views from their own imagination. 
 &emsp;&emsp;After the meeting, I summarized the ideas of each group member and combined them with the information I had collected before, to plan the design of the whole project. 
 &emsp;&emsp;Finally, I came up with the three most urgent problems, firstly, how to predict the trajectory of the ball, secondly, what direction of force should be applied to the ball when hitting the ball, and lastly, how to plan the trajectory of the robotic arm. 
 &emsp;&emsp;I reviewed many relevant papers and online materials, and finally assigned relevant tasks to my group members, monitored their progress regularly, and also held regular group meetings so that everyone could share their gains and questions in recent times, and help group members to complete their tasks. 
 &emsp;&emsp;I also took up the most difficult task in the assignment of tasks, mainly solving the problem of the way of hitting the ball and the trajectory planning of the robotic arm. After reviewing a lot of literature and information, we chose [robotic-toolbox-python](https://github.com/petercorke/robotics-toolbox-python)(a github project contributed by [petercorke](https://github.com/petercorke)) for the control of the robotic arm and solved the problem of which direction of force should be applied when hitting the ball after referring to many papers.  
  
## 3. Environment and Dependencies

 ### 3.1 Prerequisites
 * python>=3.6
 * [robotic-toolbox-python](https://github.com/petercorke/robotics-toolbox-python)
* [WebotsR2022a](https://github.com/cyberbotics/webots/releases/download/R2022a/webots-R2022a_setup.exe )

### 3.2 Construct environment and install robotic-toolbox-python

```bash
 # create virtual env
 mkdir Robotic && cd Robotic
 conda create -n rtb_env python=3.7
 conda activate rtb_env
# install robotic-toolbox-python
 pip3 install roboticstoolbox-python
	 
# git clone the project to current folder
 git clone https://github.com/unswimmingduck/Tabletennis-Robot.git
```
&emsp;&emsp;if you want to have more insights into robotic-toolbox-python you, you can use the the guide below.
<table style="border:0px">
<tr style="border:0px">
<td style="border:0px">
<img src="https://github.com/petercorke/robotics-toolbox-python/raw/master/docs/figs/RobToolBox_RoundLogoB.png" width="200"></td>
<td style="border:0px">
A Python implementation of the <a href="https://github.com/petercorke/robotics-toolbox-matlab">Robotics Toolbox for MATLAB<sup>&reg;</sup></a>
<ul>
<li><a href="https://github.com/petercorke/robotics-toolbox-python">GitHub repository </a></li>
<li><a href="https://petercorke.github.io/robotics-toolbox-python">Documentation</a></li>
<li><a href="#6">ICRA Paper</a></li>
<li><a href="https://github.com/petercorke/robotics-toolbox-python/wiki">Wiki (examples and details)</a></li>
</ul>
</td>
</tr>
</table>

### 3.3 Deploy the virtual env in Webots
```bash
# create the deployment configuration file
 cd Tabletennis-Robot/controllers/robot_controller
 touch runtime.ini
```
&emsp;&emsp;Then, we need to write the runtime.ini file. The content is needed to write on runtime.ini file is below. You need to write the file based on your path to virtual conda environment in the 3.1 section.
  ```bash
 ;  runtime.ini for a Specified Python Interpreter

[python]
COMMAND = path\to\your\virtual_env\Scripts\python.exe
```
### 3.4 Install Webots
&emsp;&emsp;Because we depoly the simulation in Webots, we also need to install the Webots to see how the simulation works.
&emsp;&emsp; To installing Webots, we could use [Webots official guide](https://www.cyberbotics.com/#download), you can deploy Webots on Win, Mac or Linux. If you want to acquire more details about Webots, you can also go to theie official web ([click here](https://www.cyberbotics.com/#webots)).
## 4. Adaptation in robotic-toolbox-python
&emsp;&emsp; Firstly, we should change the parameters of robtic arm in robotic-toolbox-python according to the robotic arm we used in the Webots simulation environment. We should replace the file named Puma560.py in path\to\your\virtual_env\roboticstoolbox\models\DH with rtb_change\Puma560.py in our project. 

## 5. Run and Simulation
&emsp;&emsp; After those steps, you can open the webot_world/worlds/table_tennis_robot.wbt in Webots. Then you can start the simulation, so that you can see the result of our simulation in Webots.

## 6. Result and Prize  
