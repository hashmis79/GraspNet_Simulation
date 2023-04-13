# GraspNet_Simulation

## Installation
First create a workspace and in that create an src folder. In that Src folder, clone the repo.
```
mkdir final_ws/src
cd final_ws/src
git clone https://github.com/hashmis79/GraspNet_Simulation.git
```
### GraspNET installations
In the cloned repo, go to scripts/graspnet_baseline, to perform installations related to graspnet and follow the below step

```bash
pip install -r requirements.txt
```
Compile and install pointnet2 operators (code adapted from [votenet](https://github.com/facebookresearch/votenet)).
```bash
cd pointnet2
python setup.py install
```
Compile and install knn operator (code adapted from [pytorch_knn_cuda](https://github.com/chrischoy/pytorch_knn_cuda)).
```bash
cd knn
python setup.py install
```
Install graspnetAPI for evaluation.
```bash
git clone https://github.com/graspnet/graspnetAPI.git
cd graspnetAPI
pip install .
```
### Autolab_core installation
Start by cloning the repo into your workspace
```
$ cd {PATH_TO_YOUR_CATKIN_WORKSPACE}/src
$ git clone https://github.com/BerkeleyAutomation/autolab_core.git
```
Change directories to autolab_core and run the following command
```
$ python setup.py install
```
finally build your workspace
```
$ cd {PATH_TO_YOUR_CATKIN_WORKSPACE}
$ catkin_make
```
## Running the Program
Open 2 terminal windows. In the first window, run the following command
```
roslaunch manipulator_1 demo_gazebo.launch
```
and in the other terminal, run the following command
```
rosrun manipulator_1 graspnet_node.py
```