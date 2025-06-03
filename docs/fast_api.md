from commonroad_route_planner.reference_path import ReferencePath

## Fast Api
The fast API allows users with only a few lines of code to generate the global trajectory from a scenario and a planning
problem. 


<<<<<<< HEAD
=======
=======

>>>>>>> develop
### Example 1: From scenario and planning problem
When giving a scenario, one can optionally choose to consider stop lines and traffic lights

```Python
from commonroad.common.file_reader import CommonRoadFileReader
import commonroad_velocity_planner.fast_api as fast_api

scenario, planning_problem_set = CommonRoadFileReader("PATH/TO/YOUR/SCENARIO").open()
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

global_trajectory = fast_api.global_trajectory_from_scenario_and_planning_problem(
    scenario=scenario, 
    planning_problem=planning_problem, 
    use_regulatory_elements=True
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


### Example 3: From CommonRoad reference path

```Python
from commonroad.common.file_reader import CommonRoadFileReader
import commonroad_route_planner.fast_api.fast_api as rfapi
from commonroad_route_planner.reference_path import ReferencePath
import commonroad_velocity_planner.fast_api as fast_api

scenario, planning_problem_set = CommonRoadFileReader("PATH/TO/YOUR/SCENARIO").open()
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

reference_path: ReferencePath = rfapi.generate_reference_path_from_scenario_and_planning_problem(
    scenario=scenario,
    planning_problem=planning_problem, 
)

global_trajectory = fast_api.global_trajectory_from_cr_reference_path_and_planning_problem(
    cr_reference_path=reference_path, 
    planning_problem=planning_problem
)
```