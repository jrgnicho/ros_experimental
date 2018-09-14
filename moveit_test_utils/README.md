### MoveIt! Test Utilities
---
#### - moveit goal commander tool

This tool allows sending joint and cartesian goals to a motion planner loaded into the ```move_group``` node.   
- First run your ```demo.launch``` file of any available _moveit_config_ package that runs the MoveIt! motion planning pipeline.  
- Then run the **moveit_goal_commander** node for an existing planning group.  This examples uses the "manipulator_tool" group.
    ```
    rosrun moveit_test_utils moveit_goal_commander _planning_group:=manipulator_tool
    ```
- In the prompt, enter a command followed by the goal arguments.  For instance, to send a joint goal one would enter:
    ```
    1 -1.0 -1.57 0.0 0.01 0.01 0.01 0.04
    ```
    This will send a motion plan request to the joint position [-1.0 -1.57 0.0 0.01 0.01 0.01 0.04]. If a valid
    plan is found then you'll see the preview in RViz.
    
- To send a cartesian position goal then enter the following command:
    ```
        2 1.0 2.0 1.4 0.015 0.04 0.02
    ```
    This creates a position goal to the location [1.0 2.0 1.4] with tolerances [0.015 0.04 0.02]
    
- To send a full cartesian pose goal then enter the following command:
    ```
        4 1.0 2.0 1.4 0.015 0.04 0.02
    ```
    This creates a fully defined cartesian pose goal to the location [1.0 2.0 1.4 0.015 0.04 0.02]
    
The commands syntax are the following:
- **JOINT**: 
	    ```1 j1 j2 j3 j4 j5 j6``` 
- **CARTESIAN_POSITION**: 
	    ```4 px py pz tol_px tol_py tol_pz```
- **CARTESIAN_ORIENTATION**:
	    ```3 rx ry rz tol_rx tol_ry tol_rz```
- **CARTESIAN_POSE**: 
	    ```4 px py pz rx ry rz```

>Note: All Cartesian goals are created relative to the planning frame.