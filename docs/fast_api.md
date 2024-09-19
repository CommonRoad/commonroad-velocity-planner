## Fast Api
The fast API allows users with only a few lines of code to generate the global trajectory from a scenario and a planning
problem. 

### Example 1: From scenario and planning problem

```Python
from commonroad.common.file_reader import CommonRoadFileReader
import commonroad_velocity_planner.fast_api as fast_api

scenario, planning_problem_set = CommonRoadFileReader("PATH/TO/YOUR/SCENARIO").open()
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

global_trajectory = fast_api.global_trajectory_from_scenario_and_planning_problem(
    scenario=scenario, 
    planning_problem=planning_problem
)

```


### Example 2: From lanelet network and planning problem

```Python
from commonroad.common.file_reader import CommonRoadFileReader
import commonroad_velocity_planner.fast_api as fast_api

scenario, planning_problem_set = CommonRoadFileReader("PATH/TO/YOUR/SCENARIO").open()
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

global_trajectory = fast_api.global_trajectory_from_lanelet_network_and_planning_problem(
    lanelet_network=scenario.lanelet_network, 
    planning_problem=planning_problem
)

```