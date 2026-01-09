# 2 Link Scara Controller

## To run:
- Open "main.slx" file in simulink 
- Run the file normally in the run button

## To define the simulation desired (e.g. controller/observer type):
- Open "initializer.mlx" file in MATLAB
- Go to the section of **Defining simulation**
- Chose using the dropdowns the options desired for simulation
- Then going to the simulink file "main.slx" and clicking run, automatically updates preferences

## PLotting
- Plotting is showed automatically after running
- For rerunning some figure, click the button in the top level view (named "mains") in the "main.slx" simulink file (partically usefull for rerunning the SCARA Animation)

## To change parameters
- Open "sim_config_parameters.m" inside the folder **configuration**
- Parameters will be place there by section 

### Notes:
- Pay attention that in the "initializer.mlx" file, in the section **Defining simulation**, some dropdown options might not be implemented yet, confirm in simulink if it has been actually implemented
- Some parameters may depende of the options selected in **Defining simulation** section, those dependencies are implemented in MATLAB code in the "sim_config_parameters.m" file or inside MATlAB Function Block in Simulink like for the case of the Non-Linear controller Feedback Linearisation inside the Virtual Controller block. 
- For the case of the Extended Kalman Filter Simulink Block changing the parameters will not update this implementation as this is code generation sensitive (meaning MATLAB doesn't allow for external parameter dependencies to be used) and the linearised A matrix around a equilibrium point x defined by the input of fucntion "myStateTransitionFcn" is implemented offline. Likewise for other functions used on this Simulink Block.


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
│   │   sim_config_parameters.m - Robot and Simulation parameters are defines here
│   │
│   └───enums - Enums for Defining Simulation options in "initializer.mlx" file and in simulink block variant
│           ControllerAction.m 
│           ControllerPayloadCompensation.m
│           ControllerType.m
│           ExtDisturbances.m
│           InputSaturation.m
│           MeasDisturbances.m
│           Payload.m 
│           RectangleHollowLink.m
│           RefGeneration.m
│           StateObservability.m
│           SystemSym.m
│
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
│   ├───linear_system
│   ├───non_linear_system - Func for dynamics equations, foward/inv kin, observers, ... 
│   │   │   c_vec.m
│   │   │   D_mat.m
│   │   │   f.m
│   │   │   foward_kin.m
│   │   │   G_vec.m
│   │   │   inverse_kin.m
│   │   │   J.m
│   │   │   J_inv.m
│   │   │   J_T.m
│   │   │   M.m
│   │   │   robot_ddq.m
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