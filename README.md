# 2 Link Scara Controller

## To run:
- Open "main.slx" in simulink 
- Run the file normally in the run button

## To define the simulation desired (e.g. controller/observer type):
- Open "initializer.mlx" in MATLAB
- Go to the section of **Defining simulation**
- Chose using the dropdowns the options desired for simulation
- Then going to the simulink file and click run, automatically updates preferences

## To change parameters
- Open "sim_config_parameters.m" inside the folder **configuration**
- Parameters will be place there by section 

### Notes:
- Pay attention that in the "initializer.mlx" file, in the section **Defining simulation**, some dropdown options might not be implemented yet, confirm in simulink if it has been actually implemented
- Some parameters may depende of the options selected in **Defining simulation** section, those dependencies are implemented in MATLAB code in the "sim_config_parameters.m" file or inside MATlAB Function Block in Simulink like for the case of the Non-Linear controller Feedback Linearisation inside the Virtual Controller block. 
- For the case of the Extended Kalman Filter Simulink Block changing the parameters will not update this implementation as this is code generation sensitive (meaning MATLAB doesn't allow for external parameter dependencies to be used) and the linearised A matrix around a equilibrium point x defined by the input of fucntion "myStateTransitionFcn" is implemented offline. Likewise for other functions used on this Simulink Block.


## Folder and Files Explanation
C:.
│   .gitattributes
│   .gitignore
│   initializer.mlx
│   main.slx
│   main.slx.autosave
│   main.slxc
│   README.md
│   SCARARobot.prj 
│
├───configuration
│   │   sim_config_parameters.m
│   │
│   └───enums
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
├───plot
│       Animate2LinkSCARA.m
│       PlotError.m
│       PlotInput.m
│       PlotStatesEstDes.m
│       plotting_results.m
│
├───resources
│  
├───simulation_helper_functions
│   ├───linear_system
│   ├───non_linear_system
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
└───System-Analysis and Controller-Designn Workspace
        ideas.txt
        Planning etc.txt
        system_analisys_and_controller_design.m