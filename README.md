# 2 Link Scara Controller
repo: https://github.com/DiogoLima03/2-Link-SCARA-Controler.git

## Requirements
- MATLAB R2023b or newer
- Simulink
- Control System Toolbox
- Symbolic Math Toolbox
- Image Processing Toolbox

## To run:
- Open "main.slx" file in simulink 
- Run the file in the run button

## To define the simulation desired (e.g. controller/observer type):
- Open "initializer.mlx" file in MATLAB
- Go to the section of **Defining simulation**
- Chose using the dropdowns the options desired for simulation
- Then going to the simulink file "main.slx" and clicking run, automatically updates preferences
- Note if changed the robot parameters insure that is selected True in Reload Functions to upadate offline generated functions (if left always True simulation will become very slow).

## PLotting
- Plotting is showed automatically after running
- For rerunning some figure, click the button in the top level view (named "main") in the "main.slx" simulink file (partically usefull for rerunning the SCARA Animation)

## To change parameters
- Open "sim_config_parameters.m" inside the folder **configuration**
- Parameters will be place there by sections 

### Notes:
- Pay attention that in the "initializer.mlx" file, in the section **Defining simulation**, some dropdown options might not be implemented yet, confirm in simulink if it has been actually implemented
- Some parameters may depende of the options selected in **Defining simulation** section, those dependencies are implemented in MATLAB code in the "sim_config_parameters.m" file or inside MATlAB Function Block in Simulink like for the case of the Non-Linear controller Feedback Linearisation inside the Virtual Controller block. 

## Folder and Files Explanation
```
C:.
│   .gitattributes
│   .gitignore 
│   initializer.mlx - File used to Defined the Simulation (runs before simulation to give variables to Simulink) 
│   main.slx - Simulink Simulation File
│   main.slx.autosave 
│   main.slxc 
│   README.md - Documentation File
│   SCARARobot.prj 
│
├───configuration - Simulation configuration folder 
|   ├───fcn_parameters_calc - Helper Fncs to calculate parameters
│   |       RectangleHollowLink.m - calculates Links mass and inertia based on rectangle cross section values
│   |   sim_config_parameters.m - Robot and Simulation parameters are defines here
|
├───images - Images for the Maks of the subsystems blocks
├───MATLAB_system_helpers - Helper fnc not related to simulation
│   ├───enums - Enums for Defining Simulation options in "initializer.mlx" file and in simulink block variant
│   |       ControllerAction.m 
│   |       ControllerPayloadCompensation.m
│   |       ControllerType.m
│   |       ExtDisturbances.m
│   |       InputSaturation.m
│   |       MeasDisturbances.m
│   |       Payload.m 
│   |       RefGeneration.m
│   |       StateObservability.m
│   |       SystemSym.m
│   └───functions
|           ensure_linear_system_functions.mlx - Checks if there is the offline linear matrices and updates if commanded
|           generate_localConfig_from_workspace.m - Based on the parameters creates an code generation compatible file "localConfig.m"
|           localConfig.m
|           requirementsCheck.mlx - Checks if the user has installed all the required toolboxes
|
├───plot - Folder with scripts for plotting results
│       Animate2LinkSCARA.m
│       PlotError.m
│       PlotInput.m
│       PlotStatesEstDes.m
│       plotting_results.m - main file that reads the data from simulink and stores it in a struct plotData variable then uses this varaible to run the plotting functions stored in this folder
│
├───resources
│  
├───simulation_helper_functions - functions used in simulink
│   ├───linear_system - linear system matrices
|   |       A_eq_P.m - robot with Payload
|   |       A_eq_P_obsv.m - for using with an observer
|   |       A_eq_WP.m - robot without Payload
|   |       B_eq_P.m
|   |       C_eq_P.m
|   |       C_eq_WP.m
|   |       D_eq_P.m
|   |       D_eq_WP.m
|   |       linear_matrixs_generator.m - generates functions for getting the linear matrices A, B, C, D around a input eq point x_eq
|   |
│   ├───non_linear_system - Func for dynamics equations, foward/inv kin, observers, ... 
│   │   │   c_vec.m
│   │   │   D_mat.m
│   │   │   f.m
│   │   │   foward_kin.m
|   |   |   f_payload.m
│   │   │   G_vec.m
│   │   │   inverse_kin.m
│   │   │   J.m
│   │   │   J_inv.m
│   │   │   J_T.m
│   │   │   M.m
│   │   │   robot_ddq.m
|   |   |   robot_dqq_payload.m
│   │   │   W_ext.m
│   │   │
│   │   └───ExtKalFil
│   │           f_jacobian_sym.m
│   │           myMeasurementFcn.m
│   │           myMeasurementJacobianFcn.m
│   │           myStateTransitionFcn.m
│   │           myStateTransitionJacobianFcn.m
│   │
│   └───tragectory_generation
│           trag_gen.m
│
├───slprj
│   
└───System-Analysis and Controller-Designn Workspace - Folder for storing files related to the study of the system but not used for simulation 
        ideas.txt
        Planning etc.txt
        system_analisys_and_controller_design.m - File used to study and analyse the system (file initialy used)
```