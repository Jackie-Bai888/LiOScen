# LiOScen
LiOScen: Liability-Oriented Scenario Generation from Accident Reports for the Validation of Autonomous Driving Systems. 
## Installation
1. Install [Apollo](https://github.com/ApolloAuto/apollo/tree/master) 
2. Install [Carla](https://carla.readthedocs.io/en/latest/)
3. Install [Carla_Apollo_bridge](https://github.com/guardstrikelab/carla_apollo_bridge/tree/master?tab=readme-ov-file)
4. To get all the dependencies, it is sufficient to run the following command.
```
pip install -r requirements.txt
```
## Setup/Getting Started
1. You can obtain the prompts for extracting the crash sequence, clustering interaction behaviors, and generating OpenSCENARIO from several files in the 'prompt' folder.
2. Execute fuzzy testing by running the runner.py file. 
```
python runner.py
```
