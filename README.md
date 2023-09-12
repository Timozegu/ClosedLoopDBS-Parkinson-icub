#  Closed loop DBS computational model implemented on iCub robot
## Docker

To build the Docker container:
```
cd docker
bash build-docker.sh
```
If running for the first time, compile the ```.mod``` files by running:
```
nrnivmodl
```

## Computational model

The computational model needs to be launched outside from the docker.

Download the necessary packages in the requirements.txt beforehand.

To run the computational model:
```
python MarmosetBG.py
```

## Launch Gazebo Simulation
install and compile the [robotology superbuild](https://github.com/robotology/robotology-superbuild) before starting simulation.

 To launch the Gazebo iCub simulation, first initiate the yarp server  : 
```
yarpserver
```

 then in another Command window launch the gazebo simulation : 
```
gazebo tutorial_joint-interface.sdf
```

## Icub pronation-supination movements
Once both Gazebo and Computational model are initiated, run the following command : 
```
python icub.py
```


## Acknowledgement
This work is part of the Neuro4PD project, granted by Royal Society and Newton Fund (NAF\R2\180773), and São Paulo Research Foundation (FAPESP), grants 2017/02377-5 and 2018/25902-0. Moioli and Araujo acknowledge the support from the National Institute of Science and Technology, program Brain Machine Interface (INCT INCEMAQ) of the National Council for Scientific and Technological Development(CNPq/MCTI), the Rio Grande do Norte Research Foundation (FAPERN), the Coordination for the Improvement of Higher Education Personnel (CAPES), the Brazilian Innovation Agency (FINEP), and the Ministry of Education (MEC). Romano was the recipient of a master's scholarship from FAPESP, grant 2018/11075-5. Elias is funded by a CNPq Research Productivity Grant (314231/2020-0). This research was carried out using the computational resources from the Center for Mathematical Sciences Applied to Industry (CeMEAI) funded by FAPESP, grant 2013/07375-0.
Additional resources were provided by the Robotics Lab within the Edinburgh Centre for Robotics, and by the Nvidia Grants program.

