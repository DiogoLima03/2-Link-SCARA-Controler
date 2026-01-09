# 2 Link Scara Controller

## To run:
- Open "main.slx" file in simulink 
- Run the file normally in the run button

## To define the simulation desired (e.g. controller/observer type):
- Open "initializer.mlx" file in MATLAB
- Go to the section of **Defining simulation**
- Chose using the dropdowns the options desired for simulation
- Then going to the simulink file "main.slx" and clicking run, automatically updates preferences

## To change parameters
- Open "sim_config_parameters.m" inside the folder **configuration**
- Parameters will be place there by section 

### Notes:
- Pay attention that in the "initializer.mlx" file, in the section **Defining simulation**, some dropdown options might not be implemented yet, confirm in simulink if it has been actually implemented
- Some parameters may depende of the options selected in **Defining simulation** section, those dependencies are implemented in MATLAB code in the "sim_config_parameters.m" file or inside MATlAB Function Block in Simulink like for the case of the Non-Linear controller Feedback Linearisation inside the Virtual Controller block. 
- For the case of the Extended Kalman Filter Simulink Block changing the parameters will not update this implementation as this is code generation sensitive (meaning MATLAB doesn't allow for external parameter dependencies to be used) and the linearised A matrix around a equilibrium point x defined by the input of fucntion "myStateTransitionFcn" is implemented offline. Likewise for other functions used on this Simulink Block.


## Folder and Files Explanation
C:.
|   .gitattributes
|   .gitignore
|   initializer.mlx
|   main.slx
|   main.slx.autosave
|   main.slxc
|   README.md
|   SCARARobot.prj
|
+---configuration
|   |   sim_config_parameters.m
|   |
|   \---enums
|           ControllerAction.m
|           ControllerPayloadCompensation.m
|           ControllerType.m
|           ExtDisturbances.m
|           InputSaturation.m
|           MeasDisturbances.m
|           Payload.m
|           RectangleHollowLink.m
|           RefGeneration.m
|           StateObservability.m
|           SystemSym.m
|
+---plot
|       Animate2LinkSCARA.m
|       PlotError.m
|       PlotInput.m
|       PlotStatesEstDes.m
|       plotting_results.m
|
+---resources
|   \---project
|       |   Project.xml
|       |   rootp.xml
|       |
|       +---fjRQtWiSIy7hIlj-Kmk87M7s21k
|       |       NjSPEMsIuLUyIpr2u1Js5bVPsOsd.xml
|       |       NjSPEMsIuLUyIpr2u1Js5bVPsOsp.xml
|       |
|       +---HoHDHQ_WvHAAKj5aJOrvrg_vpt8
|       |       xXlmKuOQ7YT_G1elNhbKQIUqSRMd.xml
|       |       xXlmKuOQ7YT_G1elNhbKQIUqSRMp.xml
|       |
|       +---NjSPEMsIuLUyIpr2u1Js5bVPsOs
|       |       2kj09UetkV_lru3gvSPXnY6-nM4d.xml
|       |       2kj09UetkV_lru3gvSPXnY6-nM4p.xml
|       |       aEHSZBIY-yve10yGis12Zr5DLZod.xml
|       |       aEHSZBIY-yve10yGis12Zr5DLZop.xml
|       |       j4xwF_j8iFTVayUMfxLgMnTbencd.xml
|       |       j4xwF_j8iFTVayUMfxLgMnTbencp.xml
|       |       KKyDJtbdIBOlaeHmIZd5VX6vqx8d.xml
|       |       KKyDJtbdIBOlaeHmIZd5VX6vqx8p.xml
|       |       QWNDYJD5mGW1bWYvPx9DtKnxzw4d.xml
|       |       QWNDYJD5mGW1bWYvPx9DtKnxzw4p.xml
|       |       R1RggVhA72agIvELiuhWPRS8F0Id.xml
|       |       R1RggVhA72agIvELiuhWPRS8F0Ip.xml
|       |       r8LR4nLmg9ai3oHrW1r_-KocQzkd.xml
|       |       r8LR4nLmg9ai3oHrW1r_-KocQzkp.xml
|       |
|       \---root
|               6x1BhZX_fLnKpcwqra0qFwv1jIgp.xml
|               fjRQtWiSIy7hIlj-Kmk87M7s21kp.xml
|               GiiBklLgTxteCEmomM8RCvWT0nQd.xml
|               GiiBklLgTxteCEmomM8RCvWT0nQp.xml
|               HoHDHQ_WvHAAKj5aJOrvrg_vpt8p.xml
|               KAXfQgCar2Yb8zOxgvf9hdmLP1Ep.xml
|               NmGqIpAwUJcXFyLjFAGnU9uyN5Yp.xml
|               qaw0eS1zuuY1ar9TdPn1GMfrjbQp.xml
|
+---simulation_helper_functions
|   +---linear_system
|   +---non_linear_system
|   |   |   c_vec.m
|   |   |   D_mat.m
|   |   |   f.m
|   |   |   foward_kin.m
|   |   |   G_vec.m
|   |   |   inverse_kin.m
|   |   |   J.m
|   |   |   J_inv.m
|   |   |   J_T.m
|   |   |   M.m
|   |   |   robot_ddq.m
|   |   |   W_ext.m
|   |   |
|   |   \---ExtKalFil
|   |           f_jacobian_sym.m
|   |           myMeasurementFcn.m
|   |           myMeasurementJacobianFcn.m
|   |           myStateTransitionFcn.m
|   |           myStateTransitionJacobianFcn.m
|   |
|   \---tragectory_generation
|           trag_gen.m
|
+---slprj
|   +---sim
|   |   \---varcache
|   |       \---main
|   |           |   checksumOfCache.mat
|   |           |   varInfo.mat
|   |           |
|   |           \---tmwinternal
|   |                   simulink_cache.xml
|   |
|   +---_cgxe
|   |   \---main
|   |       |   main_Cache.mat
|   |       |
|   |       \---info
|   |               report.mldatx
|   |
|   +---_cprj
|   +---_jitprj
|   |       jitEngineAccessInfo.mat
|   |       NhsWNBhGUDfa7N84p12N9D.l
|   |       NhsWNBhGUDfa7N84p12N9D.mat
|   |       OVDtD31uGx4n8l4cWFIHDH.l
|   |       OVDtD31uGx4n8l4cWFIHDH.mat
|   |       s0PATMuVIVg27sLS8VaUsw.l
|   |       s0PATMuVIVg27sLS8VaUsw.mat
|   |       s0qkvyD0BDMt1V32WR2lTKD.l
|   |       s0qkvyD0BDMt1V32WR2lTKD.mat
|   |       s1FrsijRKasL1TgGen8QNN.l
|   |       s1FrsijRKasL1TgGen8QNN.mat
|   |       s1TRqAFI1ABt4WqqHBGNJeB.l
|   |       s1TRqAFI1ABt4WqqHBGNJeB.mat
|   |       s20pA5JrKY0xSR3Exyo0wTC.l
|   |       s20pA5JrKY0xSR3Exyo0wTC.mat
|   |       s2ePUZyRQKYPG4K4YeF0lHE.l
|   |       s2ePUZyRQKYPG4K4YeF0lHE.mat
|   |       s2rQvnAVCWStdmNUDB6F1aE.l
|   |       s2rQvnAVCWStdmNUDB6F1aE.mat
|   |       s2sKRHS3XMR1I5GHb8yiUoD.l
|   |       s2sKRHS3XMR1I5GHb8yiUoD.mat
|   |       s2z05O6ogYkH0aVPeB53EXF.l
|   |       s2z05O6ogYkH0aVPeB53EXF.mat
|   |       s3ndqJJp4TgTtunFeQQEtNF.l
|   |       s3ndqJJp4TgTtunFeQQEtNF.mat
|   |       s3qWNgXzxUq2WjXCjbfEo5B.l
|   |       s3qWNgXzxUq2WjXCjbfEo5B.mat
|   |       s3rUVYjY7w08OOolwuYK0eB.l
|   |       s3rUVYjY7w08OOolwuYK0eB.mat
|   |       s42i6TllMo59UtiAST7pJMD.l
|   |       s42i6TllMo59UtiAST7pJMD.mat
|   |       s4iCcfQCqOOPqvbRs0dXUQD.l
|   |       s4iCcfQCqOOPqvbRs0dXUQD.mat
|   |       s4sFsE45CjLvVlrtwM89t7.l
|   |       s4sFsE45CjLvVlrtwM89t7.mat
|   |       s574K5WfJA043JAlQNHXC9F.l
|   |       s574K5WfJA043JAlQNHXC9F.mat
|   |       s63pbG4Saia75pYjOPH6qKG.l
|   |       s63pbG4Saia75pYjOPH6qKG.mat
|   |       s6L1lx33ZfBWez3JDHU69VB.l
|   |       s6L1lx33ZfBWez3JDHU69VB.mat
|   |       s6nOuRWjxLCjkbkIAjfzAZE.l
|   |       s6nOuRWjxLCjkbkIAjfzAZE.mat
|   |       s6QSG2GQkCb8QFmtVyKHy2G.l
|   |       s6QSG2GQkCb8QFmtVyKHy2G.mat
|   |       s7Kqhl5qIuCBBWRpXkYgiYD.l
|   |       s7Kqhl5qIuCBBWRpXkYgiYD.mat
|   |       s7uJ7rsx3TqoatrSbqUaJ7D.l
|   |       s7uJ7rsx3TqoatrSbqUaJ7D.mat
|   |       s7x48rsLX3s4tZvhGkXh2dH.l
|   |       s7x48rsLX3s4tZvhGkXh2dH.mat
|   |       s7zbpUo4gARKO64VpwSHRhF.l
|   |       s7zbpUo4gARKO64VpwSHRhF.mat
|   |       s8T9orj4aCVZ5V0hk0GrtmB.l
|   |       s8T9orj4aCVZ5V0hk0GrtmB.mat
|   |       s8yBd0AfOyPssjYHmKqGQoE.l
|   |       s8yBd0AfOyPssjYHmKqGQoE.mat
|   |       s9FgiNPyyGUckSA1jldiH4F.l
|   |       s9FgiNPyyGUckSA1jldiH4F.mat
|   |       s9fKRK90vyUkOqx2afsEU7F.l
|   |       s9fKRK90vyUkOqx2afsEU7F.mat
|   |       s9hcZGi4Zq6IduTY6MMNgl.l
|   |       s9hcZGi4Zq6IduTY6MMNgl.mat
|   |       s9HXf31Ikajmg1ZEqXplX7G.l
|   |       s9HXf31Ikajmg1ZEqXplX7G.mat
|   |       s9QSq31uanJyA0KV85a6CZD.l
|   |       s9QSq31uanJyA0KV85a6CZD.mat
|   |       sA95bu9Rb0KpAta0X98OcUF.l
|   |       sA95bu9Rb0KpAta0X98OcUF.mat
|   |       sagYptL1AVvtFL1EzXL2FaE.l
|   |       sagYptL1AVvtFL1EzXL2FaE.mat
|   |       sApNFIJo6GbzhgpwGuwdqoF.l
|   |       sApNFIJo6GbzhgpwGuwdqoF.mat
|   |       sAPSzxa6btwUQWgTvDIFgVB.l
|   |       sAPSzxa6btwUQWgTvDIFgVB.mat
|   |       sAwerErpR9VvcqKi9xmyLjF.l
|   |       sAwerErpR9VvcqKi9xmyLjF.mat
|   |       saXHW7pW4FopG5XoFNanphC.l
|   |       saXHW7pW4FopG5XoFNanphC.mat
|   |       sAyNc0MrAiOe2FzCYDklSFG.l
|   |       sAyNc0MrAiOe2FzCYDklSFG.mat
|   |       sB6OnJyQE1lORZaR1OP0ns.l
|   |       sB6OnJyQE1lORZaR1OP0ns.mat
|   |       sbfkkQ02UeVtOTNKZunjnQH.l
|   |       sbfkkQ02UeVtOTNKZunjnQH.mat
|   |       sBfNtygtmn1WFvT3X0dTubG.l
|   |       sBfNtygtmn1WFvT3X0dTubG.mat
|   |       sBJ2vF8iRggXM6GSN2JQCUG.l
|   |       sBJ2vF8iRggXM6GSN2JQCUG.mat
|   |       sbjJUB16rBHJIQliSLfsnzD.l
|   |       sbjJUB16rBHJIQliSLfsnzD.mat
|   |       sbnaqLoEMLA8yHq1v3qJew.l
|   |       sbnaqLoEMLA8yHq1v3qJew.mat
|   |       sBozCUKfgu64jc2IwyF0zAE.l
|   |       sBozCUKfgu64jc2IwyF0zAE.mat
|   |       sbS7Zhk2aERLIp1JGpPKtnB.l
|   |       sbS7Zhk2aERLIp1JGpPKtnB.mat
|   |       sC2aoHjgwgBq247iIkgnz0C.l
|   |       sC2aoHjgwgBq247iIkgnz0C.mat
|   |       sC3Z8TaMkruvkSlzHVNYesG.l
|   |       sC3Z8TaMkruvkSlzHVNYesG.mat
|   |       sCh7m7nTv0VdgmRDPtVr3OH.l
|   |       sCh7m7nTv0VdgmRDPtVr3OH.mat
|   |       sCRf7hZ5NCKhrciJ9iumfU.l
|   |       sCRf7hZ5NCKhrciJ9iumfU.mat
|   |       sCrmBdIQi3prSHi47e2m7dF.l
|   |       sCrmBdIQi3prSHi47e2m7dF.mat
|   |       scRPUpnitwfeRWf6IZUUDQC.l
|   |       scRPUpnitwfeRWf6IZUUDQC.mat
|   |       sCZsABUNWfuWuWHYoAWqcnC.l
|   |       sCZsABUNWfuWuWHYoAWqcnC.mat
|   |       sdCjIb4DzD2u1aHDG0lXfZD.l
|   |       sdCjIb4DzD2u1aHDG0lXfZD.mat
|   |       sdNC5U6LwZoL6oGQODaUIcF.l
|   |       sdNC5U6LwZoL6oGQODaUIcF.mat
|   |       sDnRXsFAzIn4uS8SXpwfbbB.l
|   |       sDnRXsFAzIn4uS8SXpwfbbB.mat
|   |       sdXXwHKXx69yS99DZqGN0cF.l
|   |       sdXXwHKXx69yS99DZqGN0cF.mat
|   |       sehd52tSdVu6XklRwao12GB.l
|   |       sehd52tSdVu6XklRwao12GB.mat
|   |       sEi7xLVDn1JoFUavoAVvJd.l
|   |       sEi7xLVDn1JoFUavoAVvJd.mat
|   |       sex79I1GAA3BlXE5vHq6YCC.l
|   |       sex79I1GAA3BlXE5vHq6YCC.mat
|   |       sExkb2NcyfvXEQACcaLxvzE.l
|   |       sExkb2NcyfvXEQACcaLxvzE.mat
|   |       sexSAqBPwvcRMxVoYPUBVoB.l
|   |       sexSAqBPwvcRMxVoYPUBVoB.mat
|   |       sF55znzj4AkMPM5agkZGdCD.l
|   |       sF55znzj4AkMPM5agkZGdCD.mat
|   |       sFHuA1LLXAAoDm3UcTbkDNG.l
|   |       sFHuA1LLXAAoDm3UcTbkDNG.mat
|   |       sg4xh2hyDATqedcNy1jP6LF.l
|   |       sg4xh2hyDATqedcNy1jP6LF.mat
|   |       sgAVWZAmbGwuCJIGxHjSK4B.l
|   |       sgAVWZAmbGwuCJIGxHjSK4B.mat
|   |       sghwmmrklRLoWGAAdtyxt2C.l
|   |       sghwmmrklRLoWGAAdtyxt2C.mat
|   |       sGIEeBFRz2bWpCakp5NSI0C.l
|   |       sGIEeBFRz2bWpCakp5NSI0C.mat
|   |       sGNPKrxHWC2ftbvSv3yYHtC.l
|   |       sGNPKrxHWC2ftbvSv3yYHtC.mat
|   |       sGQSvLDUW7oFSGFbrEcL4VD.l
|   |       sGQSvLDUW7oFSGFbrEcL4VD.mat
|   |       sh6ILuxkQoSyN0hKFKiBttD.l
|   |       sh6ILuxkQoSyN0hKFKiBttD.mat
|   |       sHAFyuz21zEZhCQqd1crCgF.l
|   |       sHAFyuz21zEZhCQqd1crCgF.mat
|   |       sHDvlbC1FmSguAX0GmWiCxD.l
|   |       sHDvlbC1FmSguAX0GmWiCxD.mat
|   |       shFukl5VTETVjMLwKbn6tQC.l
|   |       shFukl5VTETVjMLwKbn6tQC.mat
|   |       shkQk8u2gP8OGydsyVWgj9B.l
|   |       shkQk8u2gP8OGydsyVWgj9B.mat
|   |       shlg6UOMulejVLgVS4hW2GE.l
|   |       shlg6UOMulejVLgVS4hW2GE.mat
|   |       sHUvYazWyw5fZ1sOUugANgD.l
|   |       sHUvYazWyw5fZ1sOUugANgD.mat
|   |       shwWikJ0DcEXIRy7FeShteD.l
|   |       shwWikJ0DcEXIRy7FeShteD.mat
|   |       shxG2ORNYY45BFavRi717V.l
|   |       shxG2ORNYY45BFavRi717V.mat
|   |       si2VH4FIOfSynE0XwJikBiG.l
|   |       si2VH4FIOfSynE0XwJikBiG.mat
|   |       sI4TXQLLQt7LcAwOuEVIb5.l
|   |       sI4TXQLLQt7LcAwOuEVIb5.mat
|   |       sIRi7xJuHq6PZeVCdKIbNG.l
|   |       sIRi7xJuHq6PZeVCdKIbNG.mat
|   |       sirIYDqL6NRytqL6DRuKdDC.l
|   |       sirIYDqL6NRytqL6DRuKdDC.mat
|   |       siUTvYmUDZvtjtFUWBpWhqE.l
|   |       siUTvYmUDZvtjtFUWBpWhqE.mat
|   |       sJ0xUNK4cG0t0lkdGj2hFfC.l
|   |       sJ0xUNK4cG0t0lkdGj2hFfC.mat
|   |       sJ2Nx6ukWCdI8SAbuqTf32F.l
|   |       sJ2Nx6ukWCdI8SAbuqTf32F.mat
|   |       sJ2q8LUS2Byb4g9WxWzJie.l
|   |       sJ2q8LUS2Byb4g9WxWzJie.mat
|   |       sjKjNVn07i4Yernm7Y1YlcD.l
|   |       sjKjNVn07i4Yernm7Y1YlcD.mat
|   |       sJlbOVujdEq2T47R2VClMFF.l
|   |       sJlbOVujdEq2T47R2VClMFF.mat
|   |       sJNjgArSmnvYT43pSX2COIF.l
|   |       sJNjgArSmnvYT43pSX2COIF.mat
|   |       sJT3atrbEBqU8aH5UDpDXh.l
|   |       sJT3atrbEBqU8aH5UDpDXh.mat
|   |       sjTjfPOHy77sgYxGQuNEQT.l
|   |       sjTjfPOHy77sgYxGQuNEQT.mat
|   |       sk4af6XYeT3VeposV4lMUuH.l
|   |       sk4af6XYeT3VeposV4lMUuH.mat
|   |       sKGAyIljFKXPDd20858MkAE.l
|   |       sKGAyIljFKXPDd20858MkAE.mat
|   |       skHQlazxz0fSbml08rCAjWF.l
|   |       skHQlazxz0fSbml08rCAjWF.mat
|   |       skiu48hisp6sodNOPUBVNtC.l
|   |       skiu48hisp6sodNOPUBVNtC.mat
|   |       sKlYyGQ2N6MOx1k92LTQAbG.l
|   |       sKlYyGQ2N6MOx1k92LTQAbG.mat
|   |       skrRDjk0ZpXjbFIaeP2h8jB.l
|   |       skrRDjk0ZpXjbFIaeP2h8jB.mat
|   |       sKuU2kyLY6zDGpyZCXIt3iC.l
|   |       sKuU2kyLY6zDGpyZCXIt3iC.mat
|   |       slBF2TwuD4osJ3bO5xHgGZG.l
|   |       slBF2TwuD4osJ3bO5xHgGZG.mat
|   |       sLhuMIs2de5TtWqV8b3BYuB.l
|   |       sLhuMIs2de5TtWqV8b3BYuB.mat
|   |       sLMO4T3nrbY8Srb85uwJQFF.l
|   |       sLMO4T3nrbY8Srb85uwJQFF.mat
|   |       sLP0WLCoWEBFXmpFe097AB.l
|   |       sLP0WLCoWEBFXmpFe097AB.mat
|   |       sLV7X0RKmQxaTZNDzTZXUhB.l
|   |       sLV7X0RKmQxaTZNDzTZXUhB.mat
|   |       sm2mI1IhQnaL2N4IaJYZpS.l
|   |       sm2mI1IhQnaL2N4IaJYZpS.mat
|   |       sM8dsV8K73SHJly7mQKunpF.l
|   |       sM8dsV8K73SHJly7mQKunpF.mat
|   |       smbHiHzesosfCeyEllcxfOE.l
|   |       smbHiHzesosfCeyEllcxfOE.mat
|   |       sMJdAv9momEhXUzho6w32dB.l
|   |       sMJdAv9momEhXUzho6w32dB.mat
|   |       sMKnPPDivhdPRRUze2GlDQH.l
|   |       sMKnPPDivhdPRRUze2GlDQH.mat
|   |       smULgCl2NvTFej3p0FI5NmD.l
|   |       smULgCl2NvTFej3p0FI5NmD.mat
|   |       sMvEaMvMuOgL1kvjdFncbMD.l
|   |       sMvEaMvMuOgL1kvjdFncbMD.mat
|   |       sn0CuIwWM1HLMzL5LMvxqM.l
|   |       sn0CuIwWM1HLMzL5LMvxqM.mat
|   |       sN9bwuliKO4hikO3394JucC.l
|   |       sN9bwuliKO4hikO3394JucC.mat
|   |       sNAlwTxH12trC9WqMlpDHGD.l
|   |       sNAlwTxH12trC9WqMlpDHGD.mat
|   |       sNCfGma9vaXvtbcVIc8naJG.l
|   |       sNCfGma9vaXvtbcVIc8naJG.mat
|   |       sNFWH45RaTVXMInTb3vkSKC.l
|   |       sNFWH45RaTVXMInTb3vkSKC.mat
|   |       snIJbNadyZYxwgu19AlhUgE.l
|   |       snIJbNadyZYxwgu19AlhUgE.mat
|   |       snKfSSqSnZ18w9W4iG6EPsH.l
|   |       snKfSSqSnZ18w9W4iG6EPsH.mat
|   |       sNKsyN05uFRkKAWrxB6Nzl.l
|   |       sNKsyN05uFRkKAWrxB6Nzl.mat
|   |       snoU3fpAK6Sb8QpQNMqZBYH.l
|   |       snoU3fpAK6Sb8QpQNMqZBYH.mat
|   |       sntnbKlLdv1QtBg4uBPw2RD.l
|   |       sntnbKlLdv1QtBg4uBPw2RD.mat
|   |       sNu86yuUc5Ged9pzIs5pZUG.l
|   |       sNu86yuUc5Ged9pzIs5pZUG.mat
|   |       snwEwcimO881TeKR9ggM6pH.l
|   |       snwEwcimO881TeKR9ggM6pH.mat
|   |       sNwhe7Z6Xt2v3fkdHHZkUIB.l
|   |       sNwhe7Z6Xt2v3fkdHHZkUIB.mat
|   |       snyLpjJCtUfdKPidISV6cG.l
|   |       snyLpjJCtUfdKPidISV6cG.mat
|   |       soAFKuXhUsajl9iaEuGkHXF.l
|   |       soAFKuXhUsajl9iaEuGkHXF.mat
|   |       sObiE0qQU79KQ3CxRjQiJaD.l
|   |       sObiE0qQU79KQ3CxRjQiJaD.mat
|   |       sOgT8emN3lupiceCYXGKsWD.l
|   |       sOgT8emN3lupiceCYXGKsWD.mat
|   |       sOPFAbGBaJDojhwukjxPefH.l
|   |       sOPFAbGBaJDojhwukjxPefH.mat
|   |       sP5pph5xqFWQ60XgcEOAonE.l
|   |       sP5pph5xqFWQ60XgcEOAonE.mat
|   |       spLdAorwomNthQS8aHor9pH.l
|   |       spLdAorwomNthQS8aHor9pH.mat
|   |       spwLPO1G8kNH4UG4VewaKhB.l
|   |       spwLPO1G8kNH4UG4VewaKhB.mat
|   |       sq1n5Ww7H2rQ8Kl0fOQB5F.l
|   |       sq1n5Ww7H2rQ8Kl0fOQB5F.mat
|   |       sQ1TSeLsVya3PQwToQKixhE.l
|   |       sQ1TSeLsVya3PQwToQKixhE.mat
|   |       sq2JsgaklRGvcTqkWrJxWM.l
|   |       sq2JsgaklRGvcTqkWrJxWM.mat
|   |       sqjhcyl6VJyfePFgQPQAZmE.l
|   |       sqjhcyl6VJyfePFgQPQAZmE.mat
|   |       sqKSsPoqnhInnlJle5BO4N.l
|   |       sqKSsPoqnhInnlJle5BO4N.mat
|   |       sQQRSLVbpMDJIBA70F4MFnF.l
|   |       sQQRSLVbpMDJIBA70F4MFnF.mat
|   |       sQRQNmVydQdzXOexgbZhE5C.l
|   |       sQRQNmVydQdzXOexgbZhE5C.mat
|   |       sQRZfnekfD4vt0GNw2EiB7G.l
|   |       sQRZfnekfD4vt0GNw2EiB7G.mat
|   |       sqwHN6YgMiAQw48lst0lCnH.l
|   |       sqwHN6YgMiAQw48lst0lCnH.mat
|   |       sQykTzmc4gzjx2yIxUa4Bu.l
|   |       sQykTzmc4gzjx2yIxUa4Bu.mat
|   |       sR3oOQrb9pFYH1r2s6Z32XB.l
|   |       sR3oOQrb9pFYH1r2s6Z32XB.mat
|   |       sR8ZoshWjE17Sd7shDp7BvH.l
|   |       sR8ZoshWjE17Sd7shDp7BvH.mat
|   |       srAUK8C4ER85s42pogS65QB.l
|   |       srAUK8C4ER85s42pogS65QB.mat
|   |       sreu6YQKWKFaaEwTZpvxEOB.l
|   |       sreu6YQKWKFaaEwTZpvxEOB.mat
|   |       srIGWz8AaDJrQ6TUc9fFq6C.l
|   |       srIGWz8AaDJrQ6TUc9fFq6C.mat
|   |       srROgzrADg1qFQWaBxxWo0B.l
|   |       srROgzrADg1qFQWaBxxWo0B.mat
|   |       ss0ROQpm6udZ9jNpYz7EXZE.l
|   |       ss0ROQpm6udZ9jNpYz7EXZE.mat
|   |       sS1mQ1gUVZSP7Uxk7k3KcrC.l
|   |       sS1mQ1gUVZSP7Uxk7k3KcrC.mat
|   |       sSagSBdps999bw5Qpsco7BE.l
|   |       sSagSBdps999bw5Qpsco7BE.mat
|   |       ssfYir96vUyKkTzq0cnbcrF.l
|   |       ssfYir96vUyKkTzq0cnbcrF.mat
|   |       sSSuSNOGGkn1XGnIru2oykD.l
|   |       sSSuSNOGGkn1XGnIru2oykD.mat
|   |       sSuq4PMOqAfzjkXHZsLwmnD.l
|   |       sSuq4PMOqAfzjkXHZsLwmnD.mat
|   |       sTg1z3sIIS44ZHcA9EsfM9D.l
|   |       sTg1z3sIIS44ZHcA9EsfM9D.mat
|   |       sTHUpw9E13rrSOZLaRjVnD.l
|   |       sTHUpw9E13rrSOZLaRjVnD.mat
|   |       stL3DI8CUdLmT7Zu7sRfKsG.l
|   |       stL3DI8CUdLmT7Zu7sRfKsG.mat
|   |       sTLbIRaV39LzKNZ3tpmcizG.l
|   |       sTLbIRaV39LzKNZ3tpmcizG.mat
|   |       stPsXgaBNNZDs7vk2tDZZzE.l
|   |       stPsXgaBNNZDs7vk2tDZZzE.mat
|   |       sTTwwO3LEr79E0Skrj0iv2F.l
|   |       sTTwwO3LEr79E0Skrj0iv2F.mat
|   |       styKYvhT65mD5d8FxxqTujF.l
|   |       styKYvhT65mD5d8FxxqTujF.mat
|   |       stZFGM1iMBh1wSYDJezv0g.l
|   |       stZFGM1iMBh1wSYDJezv0g.mat
|   |       sU1pqhMMSCsmexQ3bGCnFmG.l
|   |       sU1pqhMMSCsmexQ3bGCnFmG.mat
|   |       su9x6NcfDCrgXwjosOsQgsG.l
|   |       su9x6NcfDCrgXwjosOsQgsG.mat
|   |       suic7RlIdgTB8dZmBjVtEQD.l
|   |       suic7RlIdgTB8dZmBjVtEQD.mat
|   |       sUlftOenhxdkLIfq8VYrgrD.l
|   |       sUlftOenhxdkLIfq8VYrgrD.mat
|   |       sUq9LSvXgDIkmhCa0ycYSGD.l
|   |       sUq9LSvXgDIkmhCa0ycYSGD.mat
|   |       suvK91DquOKNJhQETbSETgG.l
|   |       suvK91DquOKNJhQETbSETgG.mat
|   |       sV2z6rPe9Ak2fBpz4lb7oBG.l
|   |       sV2z6rPe9Ak2fBpz4lb7oBG.mat
|   |       svdDeFCWRVdWITvKfpM0fuE.l
|   |       svdDeFCWRVdWITvKfpM0fuE.mat
|   |       svfOokReg0754qINE1niM5.l
|   |       svfOokReg0754qINE1niM5.mat
|   |       svgsGlOHvGtkTlcUQbQBagC.l
|   |       svgsGlOHvGtkTlcUQbQBagC.mat
|   |       sVluVDcRkSj6BNtFuGg4XQ.l
|   |       sVluVDcRkSj6BNtFuGg4XQ.mat
|   |       svp8woUEyfB0BfK039Si6BB.l
|   |       svp8woUEyfB0BfK039Si6BB.mat
|   |       svpGklcDgdSvzYN3SOA16wF.l
|   |       svpGklcDgdSvzYN3SOA16wF.mat
|   |       sVX1orjt2GyEGqvhC0KPpFE.l
|   |       sVX1orjt2GyEGqvhC0KPpFE.mat
|   |       sw1Mvqm58H5IiwbOj45scAF.l
|   |       sw1Mvqm58H5IiwbOj45scAF.mat
|   |       sw3qwoj3SlREMxfUBFXe7JE.l
|   |       sw3qwoj3SlREMxfUBFXe7JE.mat
|   |       sw78dZWfg33jjPgxacdkqjF.l
|   |       sw78dZWfg33jjPgxacdkqjF.mat
|   |       swGDODtKddkB4gj46aaVnyE.l
|   |       swGDODtKddkB4gj46aaVnyE.mat
|   |       swHp4jQOV5CN8RX0SwKrWlF.l
|   |       swHp4jQOV5CN8RX0SwKrWlF.mat
|   |       swIvYasHAIh2N92MOIHcr4B.l
|   |       swIvYasHAIh2N92MOIHcr4B.mat
|   |       swkjDPyjyeo64VT4wEV1utH.l
|   |       swkjDPyjyeo64VT4wEV1utH.mat
|   |       sWq4wuxLSHaROBuJIK67zCB.l
|   |       sWq4wuxLSHaROBuJIK67zCB.mat
|   |       sWsJtmwzACS951mtQ87nZNG.l
|   |       sWsJtmwzACS951mtQ87nZNG.mat
|   |       sWuKSYUuEHCYwtTQHraUL6G.l
|   |       sWuKSYUuEHCYwtTQHraUL6G.mat
|   |       sWV9wBb7fNIVqWzUvm2DEtE.l
|   |       sWV9wBb7fNIVqWzUvm2DEtE.mat
|   |       swWj6NWE9CCqAXta37vAxzB.l
|   |       swWj6NWE9CCqAXta37vAxzB.mat
|   |       sx43tGCPeUoI8wXWJLAORiD.l
|   |       sx43tGCPeUoI8wXWJLAORiD.mat
|   |       sX6ESfuq7qSMMKTKA2FO0oG.l
|   |       sX6ESfuq7qSMMKTKA2FO0oG.mat
|   |       sXCB2tWHMlKYqmKGE0fGuPE.l
|   |       sXCB2tWHMlKYqmKGE0fGuPE.mat
|   |       sxJ2y2qz9ZpgBsdNEO9dw1.l
|   |       sxJ2y2qz9ZpgBsdNEO9dw1.mat
|   |       sxkQiLaw06Qma8WsAl8YnpH.l
|   |       sxkQiLaw06Qma8WsAl8YnpH.mat
|   |       sxQRzNDeAY9UtPUlXdY4g8C.l
|   |       sxQRzNDeAY9UtPUlXdY4g8C.mat
|   |       sXrPVACuhw2a56xiyNjWxIB.l
|   |       sXrPVACuhw2a56xiyNjWxIB.mat
|   |       syfKuNX7L8Tvn6XZYhd3zLB.l
|   |       syfKuNX7L8Tvn6XZYhd3zLB.mat
|   |       syiRHx54JLoev1sJX8KyCcG.l
|   |       syiRHx54JLoev1sJX8KyCcG.mat
|   |       sYKWjM3QTrq4ESo2oF2c3HG.l
|   |       sYKWjM3QTrq4ESo2oF2c3HG.mat
|   |       sYr3wgy5qYvngyW2eKspXrE.l
|   |       sYr3wgy5qYvngyW2eKspXrE.mat
|   |       sySTSSI3bdtLzCH8vNu1IKD.l
|   |       sySTSSI3bdtLzCH8vNu1IKD.mat
|   |       sYvbo3ithYAgubBNko1MCqD.l
|   |       sYvbo3ithYAgubBNko1MCqD.mat
|   |       sYXMkLX4ktQEZW78RUfw2sB.l
|   |       sYXMkLX4ktQEZW78RUfw2sB.mat
|   |       sz0TzbAj1WstAQsUzWS5ySE.l
|   |       sz0TzbAj1WstAQsUzWS5ySE.mat
|   |       sz1AjPPdVSSzjUK36C9AHRC.l
|   |       sz1AjPPdVSSzjUK36C9AHRC.mat
|   |       sZBUc0rxw9xhQxgkTWUt1JD.l
|   |       sZBUc0rxw9xhQxgkTWUt1JD.mat
|   |       sZhRTr4tNpNjGsuPjmpaY1B.l
|   |       sZhRTr4tNpNjGsuPjmpaY1B.mat
|   |       sZx4PDN0XYRnByrTTVhJYL.l
|   |       sZx4PDN0XYRnByrTTVhJYL.mat
|   |       sZxE8wIEjoBYmqWrg0xwXGG.l
|   |       sZxE8wIEjoBYmqWrg0xwXGG.mat
|   |       u8lKcPMRe3S0V2iSK5dJUG.l
|   |       u8lKcPMRe3S0V2iSK5dJUG.mat
|   |       wBYXtCcDUMIlseXkwUy96C.l
|   |       wBYXtCcDUMIlseXkwUy96C.mat
|   |
|   \---_sfprj
|       +---EMLReport
|       |   |   32cUKpCdG099gdLo98Uli.mat
|       |   |   3kZoJN1NXHbIRTAx7Gfk5G.mat
|       |   |   emlReportAccessInfo.mat
|       |   |   oA2P0bBmSJEkIYj8jDAJUD.mat
|       |   |   oS173ws4ejH8nM5ouknJvE.mat
|       |   |   rJS6y8lMyD5fZRK2yWsPhF.mat
|       |   |   s0PATMuVIVg27sLS8VaUsw.mat
|       |   |   s0qkvyD0BDMt1V32WR2lTKD.mat
|       |   |   s1FrsijRKasL1TgGen8QNN.mat
|       |   |   s1TRqAFI1ABt4WqqHBGNJeB.mat
|       |   |   s20pA5JrKY0xSR3Exyo0wTC.mat
|       |   |   s2ePUZyRQKYPG4K4YeF0lHE.mat
|       |   |   s2rQvnAVCWStdmNUDB6F1aE.mat
|       |   |   s2sKRHS3XMR1I5GHb8yiUoD.mat
|       |   |   s2z05O6ogYkH0aVPeB53EXF.mat
|       |   |   s3ndqJJp4TgTtunFeQQEtNF.mat
|       |   |   s3qWNgXzxUq2WjXCjbfEo5B.mat
|       |   |   s3rUVYjY7w08OOolwuYK0eB.mat
|       |   |   s42i6TllMo59UtiAST7pJMD.mat
|       |   |   s4iCcfQCqOOPqvbRs0dXUQD.mat
|       |   |   s4sFsE45CjLvVlrtwM89t7.mat
|       |   |   s574K5WfJA043JAlQNHXC9F.mat
|       |   |   s63pbG4Saia75pYjOPH6qKG.mat
|       |   |   s6L1lx33ZfBWez3JDHU69VB.mat
|       |   |   s6nOuRWjxLCjkbkIAjfzAZE.mat
|       |   |   s6QSG2GQkCb8QFmtVyKHy2G.mat
|       |   |   s7Kqhl5qIuCBBWRpXkYgiYD.mat
|       |   |   s7uJ7rsx3TqoatrSbqUaJ7D.mat
|       |   |   s7x48rsLX3s4tZvhGkXh2dH.mat
|       |   |   s7zbpUo4gARKO64VpwSHRhF.mat
|       |   |   s83zvvAUA2L6c02wtPn0BUE.mat
|       |   |   s8T9orj4aCVZ5V0hk0GrtmB.mat
|       |   |   s8yBd0AfOyPssjYHmKqGQoE.mat
|       |   |   s9FgiNPyyGUckSA1jldiH4F.mat
|       |   |   s9fKRK90vyUkOqx2afsEU7F.mat
|       |   |   s9hcZGi4Zq6IduTY6MMNgl.mat
|       |   |   s9HXf31Ikajmg1ZEqXplX7G.mat
|       |   |   s9QSq31uanJyA0KV85a6CZD.mat
|       |   |   sA95bu9Rb0KpAta0X98OcUF.mat
|       |   |   sagYptL1AVvtFL1EzXL2FaE.mat
|       |   |   sApNFIJo6GbzhgpwGuwdqoF.mat
|       |   |   sAPSzxa6btwUQWgTvDIFgVB.mat
|       |   |   sAwerErpR9VvcqKi9xmyLjF.mat
|       |   |   saXHW7pW4FopG5XoFNanphC.mat
|       |   |   sAyNc0MrAiOe2FzCYDklSFG.mat
|       |   |   sB6OnJyQE1lORZaR1OP0ns.mat
|       |   |   sbfkkQ02UeVtOTNKZunjnQH.mat
|       |   |   sBfNtygtmn1WFvT3X0dTubG.mat
|       |   |   sBJ2vF8iRggXM6GSN2JQCUG.mat
|       |   |   sbjJUB16rBHJIQliSLfsnzD.mat
|       |   |   sbnaqLoEMLA8yHq1v3qJew.mat
|       |   |   sBozCUKfgu64jc2IwyF0zAE.mat
|       |   |   sbS7Zhk2aERLIp1JGpPKtnB.mat
|       |   |   sC2aoHjgwgBq247iIkgnz0C.mat
|       |   |   sC3Z8TaMkruvkSlzHVNYesG.mat
|       |   |   sC83QP0AM0UXEAJP9IUSwMD.mat
|       |   |   sCh7m7nTv0VdgmRDPtVr3OH.mat
|       |   |   sCRf7hZ5NCKhrciJ9iumfU.mat
|       |   |   sCrmBdIQi3prSHi47e2m7dF.mat
|       |   |   scRPUpnitwfeRWf6IZUUDQC.mat
|       |   |   sCZsABUNWfuWuWHYoAWqcnC.mat
|       |   |   sdCjIb4DzD2u1aHDG0lXfZD.mat
|       |   |   sdNC5U6LwZoL6oGQODaUIcF.mat
|       |   |   sDnRXsFAzIn4uS8SXpwfbbB.mat
|       |   |   sdXXwHKXx69yS99DZqGN0cF.mat
|       |   |   sehd52tSdVu6XklRwao12GB.mat
|       |   |   sEi7xLVDn1JoFUavoAVvJd.mat
|       |   |   sex79I1GAA3BlXE5vHq6YCC.mat
|       |   |   sExkb2NcyfvXEQACcaLxvzE.mat
|       |   |   sexSAqBPwvcRMxVoYPUBVoB.mat
|       |   |   sF55znzj4AkMPM5agkZGdCD.mat
|       |   |   sFHuA1LLXAAoDm3UcTbkDNG.mat
|       |   |   sg4xh2hyDATqedcNy1jP6LF.mat
|       |   |   sgAVWZAmbGwuCJIGxHjSK4B.mat
|       |   |   sghwmmrklRLoWGAAdtyxt2C.mat
|       |   |   sGIEeBFRz2bWpCakp5NSI0C.mat
|       |   |   sGNPKrxHWC2ftbvSv3yYHtC.mat
|       |   |   sGQSvLDUW7oFSGFbrEcL4VD.mat
|       |   |   sh6ILuxkQoSyN0hKFKiBttD.mat
|       |   |   sHAFyuz21zEZhCQqd1crCgF.mat
|       |   |   sHDvlbC1FmSguAX0GmWiCxD.mat
|       |   |   shFukl5VTETVjMLwKbn6tQC.mat
|       |   |   shkQk8u2gP8OGydsyVWgj9B.mat
|       |   |   shlg6UOMulejVLgVS4hW2GE.mat
|       |   |   sHUvYazWyw5fZ1sOUugANgD.mat
|       |   |   shwWikJ0DcEXIRy7FeShteD.mat
|       |   |   shxG2ORNYY45BFavRi717V.mat
|       |   |   si2VH4FIOfSynE0XwJikBiG.mat
|       |   |   sI4TXQLLQt7LcAwOuEVIb5.mat
|       |   |   sIRi7xJuHq6PZeVCdKIbNG.mat
|       |   |   sirIYDqL6NRytqL6DRuKdDC.mat
|       |   |   siUTvYmUDZvtjtFUWBpWhqE.mat
|       |   |   sJ0xUNK4cG0t0lkdGj2hFfC.mat
|       |   |   sJ2Nx6ukWCdI8SAbuqTf32F.mat
|       |   |   sJ2q8LUS2Byb4g9WxWzJie.mat
|       |   |   sjKjNVn07i4Yernm7Y1YlcD.mat
|       |   |   sJlbOVujdEq2T47R2VClMFF.mat
|       |   |   sJNjgArSmnvYT43pSX2COIF.mat
|       |   |   sJT3atrbEBqU8aH5UDpDXh.mat
|       |   |   sjTjfPOHy77sgYxGQuNEQT.mat
|       |   |   sk4af6XYeT3VeposV4lMUuH.mat
|       |   |   sKGAyIljFKXPDd20858MkAE.mat
|       |   |   skHQlazxz0fSbml08rCAjWF.mat
|       |   |   skiu48hisp6sodNOPUBVNtC.mat
|       |   |   sKlYyGQ2N6MOx1k92LTQAbG.mat
|       |   |   skrRDjk0ZpXjbFIaeP2h8jB.mat
|       |   |   sKuU2kyLY6zDGpyZCXIt3iC.mat
|       |   |   slBF2TwuD4osJ3bO5xHgGZG.mat
|       |   |   sLhuMIs2de5TtWqV8b3BYuB.mat
|       |   |   sLMO4T3nrbY8Srb85uwJQFF.mat
|       |   |   sLP0WLCoWEBFXmpFe097AB.mat
|       |   |   sLV7X0RKmQxaTZNDzTZXUhB.mat
|       |   |   sm2mI1IhQnaL2N4IaJYZpS.mat
|       |   |   sM8dsV8K73SHJly7mQKunpF.mat
|       |   |   smbHiHzesosfCeyEllcxfOE.mat
|       |   |   sMJdAv9momEhXUzho6w32dB.mat
|       |   |   sMKnPPDivhdPRRUze2GlDQH.mat
|       |   |   smULgCl2NvTFej3p0FI5NmD.mat
|       |   |   sMvEaMvMuOgL1kvjdFncbMD.mat
|       |   |   sn0CuIwWM1HLMzL5LMvxqM.mat
|       |   |   sN9bwuliKO4hikO3394JucC.mat
|       |   |   sNAlwTxH12trC9WqMlpDHGD.mat
|       |   |   sNCfGma9vaXvtbcVIc8naJG.mat
|       |   |   sNFWH45RaTVXMInTb3vkSKC.mat
|       |   |   snIJbNadyZYxwgu19AlhUgE.mat
|       |   |   snKfSSqSnZ18w9W4iG6EPsH.mat
|       |   |   sNKsyN05uFRkKAWrxB6Nzl.mat
|       |   |   snoU3fpAK6Sb8QpQNMqZBYH.mat
|       |   |   sntnbKlLdv1QtBg4uBPw2RD.mat
|       |   |   sNu86yuUc5Ged9pzIs5pZUG.mat
|       |   |   snwEwcimO881TeKR9ggM6pH.mat
|       |   |   sNwhe7Z6Xt2v3fkdHHZkUIB.mat
|       |   |   snyLpjJCtUfdKPidISV6cG.mat
|       |   |   soAFKuXhUsajl9iaEuGkHXF.mat
|       |   |   sObiE0qQU79KQ3CxRjQiJaD.mat
|       |   |   sOgT8emN3lupiceCYXGKsWD.mat
|       |   |   sOPFAbGBaJDojhwukjxPefH.mat
|       |   |   sP5pph5xqFWQ60XgcEOAonE.mat
|       |   |   spLdAorwomNthQS8aHor9pH.mat
|       |   |   spwLPO1G8kNH4UG4VewaKhB.mat
|       |   |   sq1n5Ww7H2rQ8Kl0fOQB5F.mat
|       |   |   sQ1TSeLsVya3PQwToQKixhE.mat
|       |   |   sq2JsgaklRGvcTqkWrJxWM.mat
|       |   |   sqjhcyl6VJyfePFgQPQAZmE.mat
|       |   |   sqKSsPoqnhInnlJle5BO4N.mat
|       |   |   sQQRSLVbpMDJIBA70F4MFnF.mat
|       |   |   sQRQNmVydQdzXOexgbZhE5C.mat
|       |   |   sQRZfnekfD4vt0GNw2EiB7G.mat
|       |   |   sqwHN6YgMiAQw48lst0lCnH.mat
|       |   |   sQykTzmc4gzjx2yIxUa4Bu.mat
|       |   |   sR3oOQrb9pFYH1r2s6Z32XB.mat
|       |   |   sR8ZoshWjE17Sd7shDp7BvH.mat
|       |   |   srAUK8C4ER85s42pogS65QB.mat
|       |   |   sreu6YQKWKFaaEwTZpvxEOB.mat
|       |   |   srIGWz8AaDJrQ6TUc9fFq6C.mat
|       |   |   srROgzrADg1qFQWaBxxWo0B.mat
|       |   |   ss0ROQpm6udZ9jNpYz7EXZE.mat
|       |   |   sS1mQ1gUVZSP7Uxk7k3KcrC.mat
|       |   |   sSagSBdps999bw5Qpsco7BE.mat
|       |   |   ssfYir96vUyKkTzq0cnbcrF.mat
|       |   |   sSSuSNOGGkn1XGnIru2oykD.mat
|       |   |   sSuq4PMOqAfzjkXHZsLwmnD.mat
|       |   |   sTg1z3sIIS44ZHcA9EsfM9D.mat
|       |   |   stL3DI8CUdLmT7Zu7sRfKsG.mat
|       |   |   sTLbIRaV39LzKNZ3tpmcizG.mat
|       |   |   stPsXgaBNNZDs7vk2tDZZzE.mat
|       |   |   sTTwwO3LEr79E0Skrj0iv2F.mat
|       |   |   styKYvhT65mD5d8FxxqTujF.mat
|       |   |   stZFGM1iMBh1wSYDJezv0g.mat
|       |   |   sU1pqhMMSCsmexQ3bGCnFmG.mat
|       |   |   su9x6NcfDCrgXwjosOsQgsG.mat
|       |   |   suic7RlIdgTB8dZmBjVtEQD.mat
|       |   |   sUlftOenhxdkLIfq8VYrgrD.mat
|       |   |   sUq9LSvXgDIkmhCa0ycYSGD.mat
|       |   |   suvK91DquOKNJhQETbSETgG.mat
|       |   |   sV2z6rPe9Ak2fBpz4lb7oBG.mat
|       |   |   svdDeFCWRVdWITvKfpM0fuE.mat
|       |   |   svfOokReg0754qINE1niM5.mat
|       |   |   svgsGlOHvGtkTlcUQbQBagC.mat
|       |   |   sVluVDcRkSj6BNtFuGg4XQ.mat
|       |   |   svp8woUEyfB0BfK039Si6BB.mat
|       |   |   svpGklcDgdSvzYN3SOA16wF.mat
|       |   |   sVX1orjt2GyEGqvhC0KPpFE.mat
|       |   |   sw1Mvqm58H5IiwbOj45scAF.mat
|       |   |   sw3qwoj3SlREMxfUBFXe7JE.mat
|       |   |   sw78dZWfg33jjPgxacdkqjF.mat
|       |   |   swGDODtKddkB4gj46aaVnyE.mat
|       |   |   swHp4jQOV5CN8RX0SwKrWlF.mat
|       |   |   swIvYasHAIh2N92MOIHcr4B.mat
|       |   |   swkjDPyjyeo64VT4wEV1utH.mat
|       |   |   sWq4wuxLSHaROBuJIK67zCB.mat
|       |   |   sWsJtmwzACS951mtQ87nZNG.mat
|       |   |   sWuKSYUuEHCYwtTQHraUL6G.mat
|       |   |   sWV9wBb7fNIVqWzUvm2DEtE.mat
|       |   |   swWj6NWE9CCqAXta37vAxzB.mat
|       |   |   sx43tGCPeUoI8wXWJLAORiD.mat
|       |   |   sX6ESfuq7qSMMKTKA2FO0oG.mat
|       |   |   sXCB2tWHMlKYqmKGE0fGuPE.mat
|       |   |   sxJ2y2qz9ZpgBsdNEO9dw1.mat
|       |   |   sxkQiLaw06Qma8WsAl8YnpH.mat
|       |   |   sxQRzNDeAY9UtPUlXdY4g8C.mat
|       |   |   sXrPVACuhw2a56xiyNjWxIB.mat
|       |   |   sxSATtdjKV2mvpHoUPeQ95F.mat
|       |   |   syfKuNX7L8Tvn6XZYhd3zLB.mat
|       |   |   syiRHx54JLoev1sJX8KyCcG.mat
|       |   |   sYKWjM3QTrq4ESo2oF2c3HG.mat
|       |   |   sYr3wgy5qYvngyW2eKspXrE.mat
|       |   |   sySTSSI3bdtLzCH8vNu1IKD.mat
|       |   |   sYvbo3ithYAgubBNko1MCqD.mat
|       |   |   sYXMkLX4ktQEZW78RUfw2sB.mat
|       |   |   sz0TzbAj1WstAQsUzWS5ySE.mat
|       |   |   sz1AjPPdVSSzjUK36C9AHRC.mat
|       |   |   sZBUc0rxw9xhQxgkTWUt1JD.mat
|       |   |   sZGIdGM2Iomy6V8m44HTv2B.mat
|       |   |   sZhRTr4tNpNjGsuPjmpaY1B.mat
|       |   |   sZx4PDN0XYRnByrTTVhJYL.mat
|       |   |   sZxE8wIEjoBYmqWrg0xwXGG.mat
|       |   |   y3IdA20QY4Qrnf3lTrkTOG.mat
|       |   |   ZZdWgBs536nrM86zFEqGSG.mat
|       |   |
|       |   +---32cUKpCdG099gdLo98Uli
|       |   +---3kZoJN1NXHbIRTAx7Gfk5G
|       |   +---oA2P0bBmSJEkIYj8jDAJUD
|       |   +---oS173ws4ejH8nM5ouknJvE
|       |   +---rJS6y8lMyD5fZRK2yWsPhF
|       |   +---s0PATMuVIVg27sLS8VaUsw
|       |   +---s0qkvyD0BDMt1V32WR2lTKD
|       |   +---s1FrsijRKasL1TgGen8QNN
|       |   +---s1TRqAFI1ABt4WqqHBGNJeB
|       |   +---s20pA5JrKY0xSR3Exyo0wTC
|       |   +---s2ePUZyRQKYPG4K4YeF0lHE
|       |   +---s2rQvnAVCWStdmNUDB6F1aE
|       |   +---s2sKRHS3XMR1I5GHb8yiUoD
|       |   +---s2z05O6ogYkH0aVPeB53EXF
|       |   +---s3ndqJJp4TgTtunFeQQEtNF
|       |   +---s3qWNgXzxUq2WjXCjbfEo5B
|       |   +---s3rUVYjY7w08OOolwuYK0eB
|       |   +---s42i6TllMo59UtiAST7pJMD
|       |   +---s4iCcfQCqOOPqvbRs0dXUQD
|       |   +---s4sFsE45CjLvVlrtwM89t7
|       |   +---s574K5WfJA043JAlQNHXC9F
|       |   +---s63pbG4Saia75pYjOPH6qKG
|       |   +---s6L1lx33ZfBWez3JDHU69VB
|       |   +---s6nOuRWjxLCjkbkIAjfzAZE
|       |   +---s6QSG2GQkCb8QFmtVyKHy2G
|       |   +---s7Kqhl5qIuCBBWRpXkYgiYD
|       |   +---s7uJ7rsx3TqoatrSbqUaJ7D
|       |   +---s7x48rsLX3s4tZvhGkXh2dH
|       |   +---s7zbpUo4gARKO64VpwSHRhF
|       |   +---s83zvvAUA2L6c02wtPn0BUE
|       |   +---s8T9orj4aCVZ5V0hk0GrtmB
|       |   +---s8yBd0AfOyPssjYHmKqGQoE
|       |   +---s9FgiNPyyGUckSA1jldiH4F
|       |   +---s9fKRK90vyUkOqx2afsEU7F
|       |   +---s9hcZGi4Zq6IduTY6MMNgl
|       |   +---s9HXf31Ikajmg1ZEqXplX7G
|       |   +---s9QSq31uanJyA0KV85a6CZD
|       |   +---sA95bu9Rb0KpAta0X98OcUF
|       |   +---sagYptL1AVvtFL1EzXL2FaE
|       |   +---sApNFIJo6GbzhgpwGuwdqoF
|       |   +---sAPSzxa6btwUQWgTvDIFgVB
|       |   +---sAwerErpR9VvcqKi9xmyLjF
|       |   +---saXHW7pW4FopG5XoFNanphC
|       |   +---sAyNc0MrAiOe2FzCYDklSFG
|       |   +---sB6OnJyQE1lORZaR1OP0ns
|       |   +---sbfkkQ02UeVtOTNKZunjnQH
|       |   +---sBfNtygtmn1WFvT3X0dTubG
|       |   +---sBJ2vF8iRggXM6GSN2JQCUG
|       |   +---sbjJUB16rBHJIQliSLfsnzD
|       |   +---sbnaqLoEMLA8yHq1v3qJew
|       |   +---sBozCUKfgu64jc2IwyF0zAE
|       |   +---sbS7Zhk2aERLIp1JGpPKtnB
|       |   +---sC2aoHjgwgBq247iIkgnz0C
|       |   +---sC3Z8TaMkruvkSlzHVNYesG
|       |   +---sC83QP0AM0UXEAJP9IUSwMD
|       |   +---sCh7m7nTv0VdgmRDPtVr3OH
|       |   +---sCRf7hZ5NCKhrciJ9iumfU
|       |   +---sCrmBdIQi3prSHi47e2m7dF
|       |   +---scRPUpnitwfeRWf6IZUUDQC
|       |   +---sCZsABUNWfuWuWHYoAWqcnC
|       |   +---sdCjIb4DzD2u1aHDG0lXfZD
|       |   +---sdNC5U6LwZoL6oGQODaUIcF
|       |   +---sDnRXsFAzIn4uS8SXpwfbbB
|       |   +---sdXXwHKXx69yS99DZqGN0cF
|       |   +---sehd52tSdVu6XklRwao12GB
|       |   +---sEi7xLVDn1JoFUavoAVvJd
|       |   +---sex79I1GAA3BlXE5vHq6YCC
|       |   +---sExkb2NcyfvXEQACcaLxvzE
|       |   +---sexSAqBPwvcRMxVoYPUBVoB
|       |   +---sF55znzj4AkMPM5agkZGdCD
|       |   +---sFHuA1LLXAAoDm3UcTbkDNG
|       |   +---sg4xh2hyDATqedcNy1jP6LF
|       |   +---sgAVWZAmbGwuCJIGxHjSK4B
|       |   +---sghwmmrklRLoWGAAdtyxt2C
|       |   +---sGIEeBFRz2bWpCakp5NSI0C
|       |   +---sGNPKrxHWC2ftbvSv3yYHtC
|       |   +---sGQSvLDUW7oFSGFbrEcL4VD
|       |   +---sh6ILuxkQoSyN0hKFKiBttD
|       |   +---sHAFyuz21zEZhCQqd1crCgF
|       |   +---sHDvlbC1FmSguAX0GmWiCxD
|       |   +---shFukl5VTETVjMLwKbn6tQC
|       |   +---shkQk8u2gP8OGydsyVWgj9B
|       |   +---shlg6UOMulejVLgVS4hW2GE
|       |   +---sHUvYazWyw5fZ1sOUugANgD
|       |   +---shwWikJ0DcEXIRy7FeShteD
|       |   +---shxG2ORNYY45BFavRi717V
|       |   +---si2VH4FIOfSynE0XwJikBiG
|       |   +---sI4TXQLLQt7LcAwOuEVIb5
|       |   +---sIRi7xJuHq6PZeVCdKIbNG
|       |   +---sirIYDqL6NRytqL6DRuKdDC
|       |   +---siUTvYmUDZvtjtFUWBpWhqE
|       |   +---sJ0xUNK4cG0t0lkdGj2hFfC
|       |   +---sJ2Nx6ukWCdI8SAbuqTf32F
|       |   +---sJ2q8LUS2Byb4g9WxWzJie
|       |   +---sjKjNVn07i4Yernm7Y1YlcD
|       |   +---sJlbOVujdEq2T47R2VClMFF
|       |   +---sJNjgArSmnvYT43pSX2COIF
|       |   +---sJT3atrbEBqU8aH5UDpDXh
|       |   +---sjTjfPOHy77sgYxGQuNEQT
|       |   +---sk4af6XYeT3VeposV4lMUuH
|       |   +---sKGAyIljFKXPDd20858MkAE
|       |   +---skHQlazxz0fSbml08rCAjWF
|       |   +---skiu48hisp6sodNOPUBVNtC
|       |   +---sKlYyGQ2N6MOx1k92LTQAbG
|       |   +---skrRDjk0ZpXjbFIaeP2h8jB
|       |   +---sKuU2kyLY6zDGpyZCXIt3iC
|       |   +---slBF2TwuD4osJ3bO5xHgGZG
|       |   +---sLhuMIs2de5TtWqV8b3BYuB
|       |   +---sLMO4T3nrbY8Srb85uwJQFF
|       |   +---sLP0WLCoWEBFXmpFe097AB
|       |   +---sLV7X0RKmQxaTZNDzTZXUhB
|       |   +---sm2mI1IhQnaL2N4IaJYZpS
|       |   +---sM8dsV8K73SHJly7mQKunpF
|       |   +---smbHiHzesosfCeyEllcxfOE
|       |   +---sMJdAv9momEhXUzho6w32dB
|       |   +---sMKnPPDivhdPRRUze2GlDQH
|       |   +---smULgCl2NvTFej3p0FI5NmD
|       |   +---sMvEaMvMuOgL1kvjdFncbMD
|       |   +---sn0CuIwWM1HLMzL5LMvxqM
|       |   +---sN9bwuliKO4hikO3394JucC
|       |   +---sNAlwTxH12trC9WqMlpDHGD
|       |   +---sNCfGma9vaXvtbcVIc8naJG
|       |   +---sNFWH45RaTVXMInTb3vkSKC
|       |   +---snIJbNadyZYxwgu19AlhUgE
|       |   +---snKfSSqSnZ18w9W4iG6EPsH
|       |   +---sNKsyN05uFRkKAWrxB6Nzl
|       |   +---snoU3fpAK6Sb8QpQNMqZBYH
|       |   +---sntnbKlLdv1QtBg4uBPw2RD
|       |   +---sNu86yuUc5Ged9pzIs5pZUG
|       |   +---snwEwcimO881TeKR9ggM6pH
|       |   +---sNwhe7Z6Xt2v3fkdHHZkUIB
|       |   +---snyLpjJCtUfdKPidISV6cG
|       |   +---soAFKuXhUsajl9iaEuGkHXF
|       |   +---sObiE0qQU79KQ3CxRjQiJaD
|       |   +---sOgT8emN3lupiceCYXGKsWD
|       |   +---sOPFAbGBaJDojhwukjxPefH
|       |   +---sP5pph5xqFWQ60XgcEOAonE
|       |   +---spLdAorwomNthQS8aHor9pH
|       |   +---spwLPO1G8kNH4UG4VewaKhB
|       |   +---sq1n5Ww7H2rQ8Kl0fOQB5F
|       |   +---sQ1TSeLsVya3PQwToQKixhE
|       |   +---sq2JsgaklRGvcTqkWrJxWM
|       |   +---sqjhcyl6VJyfePFgQPQAZmE
|       |   +---sqKSsPoqnhInnlJle5BO4N
|       |   +---sQQRSLVbpMDJIBA70F4MFnF
|       |   +---sQRQNmVydQdzXOexgbZhE5C
|       |   +---sQRZfnekfD4vt0GNw2EiB7G
|       |   +---sqwHN6YgMiAQw48lst0lCnH
|       |   +---sQykTzmc4gzjx2yIxUa4Bu
|       |   +---sR3oOQrb9pFYH1r2s6Z32XB
|       |   +---sR8ZoshWjE17Sd7shDp7BvH
|       |   +---srAUK8C4ER85s42pogS65QB
|       |   +---sreu6YQKWKFaaEwTZpvxEOB
|       |   +---srIGWz8AaDJrQ6TUc9fFq6C
|       |   +---srROgzrADg1qFQWaBxxWo0B
|       |   +---ss0ROQpm6udZ9jNpYz7EXZE
|       |   +---sS1mQ1gUVZSP7Uxk7k3KcrC
|       |   +---sSagSBdps999bw5Qpsco7BE
|       |   +---ssfYir96vUyKkTzq0cnbcrF
|       |   +---sSSuSNOGGkn1XGnIru2oykD
|       |   +---sSuq4PMOqAfzjkXHZsLwmnD
|       |   +---sTg1z3sIIS44ZHcA9EsfM9D
|       |   +---stL3DI8CUdLmT7Zu7sRfKsG
|       |   +---sTLbIRaV39LzKNZ3tpmcizG
|       |   +---stPsXgaBNNZDs7vk2tDZZzE
|       |   +---sTTwwO3LEr79E0Skrj0iv2F
|       |   +---styKYvhT65mD5d8FxxqTujF
|       |   +---stZFGM1iMBh1wSYDJezv0g
|       |   +---sU1pqhMMSCsmexQ3bGCnFmG
|       |   +---su9x6NcfDCrgXwjosOsQgsG
|       |   +---suic7RlIdgTB8dZmBjVtEQD
|       |   +---sUlftOenhxdkLIfq8VYrgrD
|       |   +---sUq9LSvXgDIkmhCa0ycYSGD
|       |   +---suvK91DquOKNJhQETbSETgG
|       |   +---sV2z6rPe9Ak2fBpz4lb7oBG
|       |   +---svdDeFCWRVdWITvKfpM0fuE
|       |   +---svfOokReg0754qINE1niM5
|       |   +---svgsGlOHvGtkTlcUQbQBagC
|       |   +---sVluVDcRkSj6BNtFuGg4XQ
|       |   +---svp8woUEyfB0BfK039Si6BB
|       |   +---svpGklcDgdSvzYN3SOA16wF
|       |   +---sVX1orjt2GyEGqvhC0KPpFE
|       |   +---sw1Mvqm58H5IiwbOj45scAF
|       |   +---sw3qwoj3SlREMxfUBFXe7JE
|       |   +---sw78dZWfg33jjPgxacdkqjF
|       |   +---swGDODtKddkB4gj46aaVnyE
|       |   +---swHp4jQOV5CN8RX0SwKrWlF
|       |   +---swIvYasHAIh2N92MOIHcr4B
|       |   +---swkjDPyjyeo64VT4wEV1utH
|       |   +---sWq4wuxLSHaROBuJIK67zCB
|       |   +---sWsJtmwzACS951mtQ87nZNG
|       |   +---sWuKSYUuEHCYwtTQHraUL6G
|       |   +---sWV9wBb7fNIVqWzUvm2DEtE
|       |   +---swWj6NWE9CCqAXta37vAxzB
|       |   +---sx43tGCPeUoI8wXWJLAORiD
|       |   +---sX6ESfuq7qSMMKTKA2FO0oG
|       |   +---sXCB2tWHMlKYqmKGE0fGuPE
|       |   +---sxJ2y2qz9ZpgBsdNEO9dw1
|       |   +---sxkQiLaw06Qma8WsAl8YnpH
|       |   +---sxQRzNDeAY9UtPUlXdY4g8C
|       |   +---sXrPVACuhw2a56xiyNjWxIB
|       |   +---sxSATtdjKV2mvpHoUPeQ95F
|       |   +---syfKuNX7L8Tvn6XZYhd3zLB
|       |   +---syiRHx54JLoev1sJX8KyCcG
|       |   +---sYKWjM3QTrq4ESo2oF2c3HG
|       |   +---sYr3wgy5qYvngyW2eKspXrE
|       |   +---sySTSSI3bdtLzCH8vNu1IKD
|       |   +---sYvbo3ithYAgubBNko1MCqD
|       |   +---sYXMkLX4ktQEZW78RUfw2sB
|       |   +---sz0TzbAj1WstAQsUzWS5ySE
|       |   +---sz1AjPPdVSSzjUK36C9AHRC
|       |   +---sZBUc0rxw9xhQxgkTWUt1JD
|       |   +---sZGIdGM2Iomy6V8m44HTv2B
|       |   +---sZhRTr4tNpNjGsuPjmpaY1B
|       |   +---sZx4PDN0XYRnByrTTVhJYL
|       |   +---sZxE8wIEjoBYmqWrg0xwXGG
|       |   +---y3IdA20QY4Qrnf3lTrkTOG
|       |   \---ZZdWgBs536nrM86zFEqGSG
|       +---main
|       |   |   amsi_serial.mat
|       |   |
|       |   \---_self
|       |       \---sfun
|       |           \---info
|       |                   binfo.mat
|       |
|       \---precompile
|               0cEipZDdhDa0FYwEPlgQSE.mat
|               0DRChDMvzEQq0j9iMo8J2G.mat
|               0FaAUtouW3UYIaxKqS2F8E.mat
|               0j3Azvq2z9sPqyj70b1KU.mat
|               0wCMfaMcoMMPK41eCnmmLD.mat
|               0ZnqCEVx6DscALuR63cU1D.mat
|               14n3LD609j1q1htK0VKB2C.mat
|               166am3Onp53odUwva1hYvE.mat
|               1aUSAsFE0YTLKDV5bKdMUH.mat
|               1dfgNpE23g1PBE6pEpglMB.mat
|               1EjuE9umSfYY7PuUyocLmH.mat
|               1KxrLnDMx69lxyWfWxoJ2B.mat
|               1OB7yTRMjIEP4RTHJW3BBD.mat
|               1pWfRSMLRMmMbEeMEWrRjF.mat
|               1QjEsTpNaFbQA1oLsWg8nC.mat
|               1WcNxbIjIji068KkDYGeGD.mat
|               2aJSkiWtaSCpAezPg477bC.mat
|               2G2uPTEovt1dbB8Bosyz4E.mat
|               2G7Nit3PVpr5zC90kAXo3C.mat
|               2gkDFb5FyiT4KPcS4rN5zB.mat
|               2pkZJpHQquI3QufbWOZuVH.mat
|               2RIG3SNEwEq6hstQ52xGdD.mat
|               2WLO3NAOTC5tzsoxudOg9G.mat
|               3EgnfCECMhsFjFYwLP18vG.mat
|               3HFWUdFRsN33rP5ecWfSVG.mat
|               3KyoLEUBU4fbrlV7fH2F0E.mat
|               3lwOySMEJMVhfTRvSGwmsH.mat
|               3nlkl5vSb6wdttlsQD6Z0G.mat
|               3voxjxd3sZwmKnnEc3Enp.mat
|               3xY2LPejn4wTsinKI3ofIH.mat
|               3ZG2Y5E2h5yZWQAfMa0uFG.mat
|               40zD2noiQxdSnjbQd8WOcF.mat
|               48gXu6tivd9xlq9BgLsCiC.mat
|               4dMgYIPMpifcZ5vrzdCaBH.mat
|               4EhORird7S4NlcGSzdChYH.mat
|               4ePJqLAw3xgohXxUqXoXCC.mat
|               4Pr2YTw7NFaphIRTLXODKH.mat
|               4r8gJED4HFXeIzm4wL7xHH.mat
|               4WAaPlJFkZaO81dfG50tjE.mat
|               58XwzugoMfs5EylIyWWFFD.mat
|               5eAd3AYRvZJY724hxZlV0F.mat
|               5EysX1sj99MFaXIYwg4slF.mat
|               5icVEoSj3hmN5pTdHcRDOC.mat
|               5iWz34Kqbsk0AlUOFefIQD.mat
|               5lj8Jiz2Q2JYTEJ7HGL60F.mat
|               5y6dnKRHf9CWciVvPMoAqG.mat
|               656ybzVRE8ERTvSAuc6uQF.mat
|               6Ha9tYttf79RGEVSiAXF8F.mat
|               6Hm8hk798AVb4PftYWRnRG.mat
|               6Mj3vDw8vqUb3IyUPCRQQC.mat
|               6ZfA8k1gjIJrrgRMecqRgG.mat
|               76Sox2vBjvv6kShADG2MSH.mat
|               7HobAPGnATNNmQCTMN76GD.mat
|               7IKVaSOTxvVBdFviQKWa9D.mat
|               7KCfeXkxHrD0ZzeTCMHRFC.mat
|               7QKmSSn0bvU82OWaCC1cEF.mat
|               7rsuOWiIAcxgAZJZyfj1kH.mat
|               87wPLxb9fOQCJKEKmF5y9B.mat
|               8g4GbZ0xoJGVfEXA68z1Y.mat
|               8iw0tNeVuEhRXipIzFX7IB.mat
|               8JQR05cGNzVy2z78khvVpC.mat
|               8VBEfo2WDOil0kOOc1x53C.mat
|               8zapXVuFQBuSwaa9J1GivB.mat
|               8zPwrhrTHaMIzrVYv0Iv8B.mat
|               8zupL5sIAJmWwcm2UBf1OC.mat
|               9aLCNPpYgVfWc4vRqhzHx.mat
|               9iR4anYA6c8N3QLqYQIsAG.mat
|               9kLzK2PZUqRoFzn6b6GHmH.mat
|               9o19PmGGeTfk5jwXYMM9NB.mat
|               9sz7pLC3qkzQXKD1C6Wo1C.mat
|               9TRPDGOnc0EJnAgFmcw4WH.mat
|               9Tsg67CuUopEq8rtewNG2.mat
|               9vRqcDwpyTCKJY4aGn0xcG.mat
|               AGui64FIjtAgy89QgJLUCH.mat
|               aHv9JoohlqXWWzYpFZLE5E.mat
|               AKoQe95C9VYPWVUUQHkBjC.mat
|               am42VlYF343UaXycpaPZKC.mat
|               anxEyVCbBevXxA6rbvmscC.mat
|               AnXWMpxhNqcAiG5Gms1haG.mat
|               APCwcVudcE7cEXS6jwXSaG.mat
|               aPXuwY8GzhJ9jszauKcWgC.mat
|               autoInferAccessInfo.mat
|               AX5TFnftWYhQRpl0Yht8TF.mat
|               aZ1KFvI58Co0UShaSUDvAB.mat
|               B8owEMt5UuyKY6WOy8MOwB.mat
|               bCiVFEydscicJxiHWriOCD.mat
|               BF3CnqMbaDrCKTbIcXEyOH.mat
|               BHqyMV8zbYOuaZANEX0WiD.mat
|               bHwkhAvPxAlpyXKqWbdRUF.mat
|               bHYTAIScBf8VJyksCCtlZF.mat
|               bj2d0zxx5zAotKeX0n0IUB.mat
|               bk3tKhXQPWfSDBOiVTgVBH.mat
|               BklEeDatopKB7632jO3wCG.mat
|               bpwBLVfgarG07DriuM7rpB.mat
|               BSYBrTZwxwMAHdwAOOWrGC.mat
|               BwEhMap2lgWVKIpz2LoOBC.mat
|               BzmvRVck5WVP9EmiiBnUjD.mat
|               c5mekGo0Co32eIioEUeBQC.mat
|               CB0KzqkLMcT0FneQTf6coF.mat
|               CBOghQQpS8rQM5C8BR0uu.mat
|               CFWXZk6hmDMAxxXxbVQXOC.mat
|               CIdgRJ5hGJMbc2g7Iy3dJD.mat
|               CshRTPR4RVGTzEyarKLX1B.mat
|               CTlldIbq3dirJhOc2ZhO6.mat
|               CV8OP33lAPF6eaEvornu0C.mat
|               cvpJaq4nsBKxqIbSH0zGLH.mat
|               cX7PXP0rhmaM4YsD7Xc4qE.mat
|               cXIfymzPisByLFNb8IXc4C.mat
|               CYLulVxFJ2bvdHxZh80D4F.mat
|               d3zSiGR5EaDdzSflsXxeoD.mat
|               D4LuixWdSQE9M1grnn3Or.mat
|               d6KsULEFI7AZtjXdQV4msE.mat
|               decVV8RfIf00PBjUknKxzG.mat
|               deSHNMGwmkiS5biT3gA8FC.mat
|               dF7XlofpNBzrsFf3FgK12E.mat
|               dfYhm16qlxG7ZB5H3rlHRE.mat
|               Dg9XoDPVVOlsuc2MRzSSrD.mat
|               DhQonUxeaIjLJLSG5dtNkE.mat
|               dKML0JWW9t2P7lfHUJdTWB.mat
|               dpTCRJUfJoR6TQM7BuR8i.mat
|               dszqqIA9pJr2VbsVsiwNt.mat
|               E9dfElYbVGe41bguojMllC.mat
|               eatcl9MEstcYQAbu6HuzfF.mat
|               EC9KYA16HBQ2Tni1rlzyEE.mat
|               EF6Jn8ON06nXnr1p9YFRrF.mat
|               eFJ8hJGIGSFWgWTuEUHuAG.mat
|               eI2N5cjSzSke9AflXwtT6B.mat
|               ENzPAHxxIpIahjPTxX3EhH.mat
|               eUzlMTHSXPUZ5hwTYZwl0C.mat
|               eWDgXP125EmI92JWzx9JHF.mat
|               EyOqpEdmVUu8aTcXFecFKH.mat
|               FATwa2Q8ylOeVK2k8tQquG.mat
|               fB3HVnpzomCmw25BIueG2G.mat
|               fEhyeBejOFjQEWHnPLJci.mat
|               FhKSdDt4VpOgynod5VLI7F.mat
|               fhrPsgn2LIHoyB5AYPAOyF.mat
|               FKG9jDoJhS0cZEJzlnOoPE.mat
|               fkVTWtDjTC9DcDdBT9m0uB.mat
|               FP4fpHZmheDRAETJZsg0YD.mat
|               Fsixt8tf8Xvd5sXPB0NFJH.mat
|               fUabyXN8oYgSQaJCI7oDmD.mat
|               G1fg6yS1mQeZjREHV3lcRB.mat
|               g1sJ667hXnxYy8lbiGhcJB.mat
|               G3zuA9xzsnVkYzV7CHRqxD.mat
|               GdiumohLGbh6NpZ4QgH1PH.mat
|               GIbhb0RnnK3zptn9pLFQfB.mat
|               gnXSESsJEFTsaJXcpGYa3D.mat
|               gPsvtF4Lvluzic2y28TSYF.mat
|               GpTSJ7eyB8aJe7AXF2AQXE.mat
|               GSWzNLkOKxKfBDaBsSjYBD.mat
|               gz77arJFy0nBxo7A37ZoK.mat
|               H5GP90dn8BwhOmCmmExLzD.mat
|               H92iLg6fN0uq0Q5hM5UW4B.mat
|               ha9ABKGtgRLgAa6yuZE1cH.mat
|               hgZHDhy6KofDChLEAIm2m.mat
|               Hh0onUrrNB65WskPAAEAOE.mat
|               HHqiJ67dZ2m2IL4OKN4WGG.mat
|               HJuPpRtQauIy1tDcNPCOzB.mat
|               HKq64w4gD3j4gIUEAEQecC.mat
|               hM9TeO6xwQ7pT0HpTx0MIF.mat
|               HrunIlAXH5x7gOSFXAOjWG.mat
|               HSFGDoN7McmUSE1t0EdidC.mat
|               HXiClPFma2TyABc5kdx2eD.mat
|               Hzs240DhgUhYUJyVsrCPZF.mat
|               i27L80TM8cnikwvoPIiEQE.mat
|               Ibfu9LCvNsw1EstZQqgprC.mat
|               ieFxMQOm8V8Ezs9IBfyMDC.mat
|               InRnuogfe2c2unFRtKWHQD.mat
|               iTuILCiS2t5D6dbaLR7WqG.mat
|               IwZoFoZDLEq2pst0xv7rbH.mat
|               IzX4LDYZ5zkfApYyZP31aC.mat
|               IZZoGl0OALknZqnkqAVUeG.mat
|               j3gscEBcN3yMZkGfldjhPG.mat
|               jbsEYCFqWgMNdi5SCD9BbD.mat
|               jCmTvzmrqz25a0s5XWEK1E.mat
|               JIQqq6nrBDDZI65v84TmLD.mat
|               jpEISpytEsnByedcxwaugG.mat
|               jpMufc7vrwoUEctKarUSQF.mat
|               JQisn3RQRwXeHySV7UIMVC.mat
|               jVnfU9T0cJlCV7vGX6n85B.mat
|               jWccxqeHFRbsW7TWUnsAUE.mat
|               jX2Wpk1dbSFK5ak3LiU6uD.mat
|               jxvBql2s7JZr4N4SvJ70NF.mat
|               JzbKhMkbMNIPRfjH1BlwbH.mat
|               K1yY9L0q5YTry1nJ6oDvLB.mat
|               K2sCWMvqPYPlWOirzhFR3C.mat
|               K89mSXS4BusFJAXR2xXqzC.mat
|               KcOu67CDZUVTP7C5vFgKFG.mat
|               kCTDXGscpRzUz5c5MyYVcF.mat
|               khDferee20wZaBWQo1et6B.mat
|               KiLabbIe68ZaOewXcZanhF.mat
|               KJDmnGt3sOu97R6vE4BHoF.mat
|               kK6zVCzCEoL2jYKdnwIVnC.mat
|               kn2c86PDbbZmW2tFL60UHG.mat
|               kTQN73UREmfzzHVnjmwYrD.mat
|               kZFdI7dYshD8FalGxE9deE.mat
|               L3OThIxwYEWrlojjICMpGF.mat
|               lAfqzJPrVtQKQpZJgsXPgG.mat
|               LDaPbzLJLMEDhG25GJaMHB.mat
|               LFX8VYqICkeMECvOtaE4UF.mat
|               lG7OSSmAqfaYuIziNRh2jG.mat
|               lpf4ECb7nsGlfwGJIAJgGH.mat
|               lqDpnONllAo5Fz0udPRKSD.mat
|               lQZjk9x3FcM4QjkCr6ANVF.mat
|               LVDVsnMV2lwCyEWeRsR1wF.mat
|               M0o9V0mf2lXplHEo0peDsC.mat
|               M5fncwwNZt7qHOyIlngQqC.mat
|               M6rJw3Pu9aTEUPzM5mgW1B.mat
|               mN3E422gyObO1Zp4FY9AhB.mat
|               MnQgT41UUEgi1Trp90fiNF.mat
|               mpyl5nek691mnRfkMOCiiE.mat
|               mReKe2xpTvxFf22mQG8c2F.mat
|               mSI678WlD5K1VniWwmXgID.mat
|               N9iFuIWCUli1Z95eV3bPxD.mat
|               nA3WPgBci5j2uckqUOMrp.mat
|               nCOph2j8cVFTT3mjYaxLvE.mat
|               NEdfyNOcwT7ZBhJdRUxh2B.mat
|               Nh4DNApOiaHWTEfU0e8I5G.mat
|               nHexx75VLLPgJ2IXgWQJGE.mat
|               nI3sjoYKrp13nNmsGNJ9RF.mat
|               npVWJ2YUz7wdUvRmSBSJVC.mat
|               Nqoxjv0ztoEF1SkYe58c4G.mat
|               nRkyakWKXM4kV8O1APJx2G.mat
|               NtuCjlyCn0Zg0iqT8QDSW.mat
|               nwONWq1yKbdWAfHEIjV5VG.mat
|               NYfjvtKzz0tm3DTjxemg5F.mat
|               o6JIBencwl9F5ED0clgvyC.mat
|               ob0Ub5hSt49ExqWX32iqKH.mat
|               ocOWBGCFan0ulQLkdOidmG.mat
|               oGI5LevjrC0J3RVvApPoBC.mat
|               okyjOhJto9Kfxmtcy8vruF.mat
|               onS9YzoDmibOiVoMOekVUE.mat
|               OqC0OYFmPJwQgZejPhz88F.mat
|               oqhY5GitiYxpQfPumcZxm.mat
|               OvI5Xn9PpRyqudMSXWjhxD.mat
|               Oxj8nhSgLkumqwGDkBg0c.mat
|               oXxGRUDjwJb9N3IrzGSRbC.mat
|               oZFcgs6Kbnz8m3yN4r3nHE.mat
|               paW9l6TLJ7oDAAvBPJpkN.mat
|               pcKM89b7pNQrnL8c6hVfvH.mat
|               pFZ4SwadzdPTRkGOqURJbF.mat
|               Pg006wlXY7iH4eqaRL0hqF.mat
|               pHu7AsZV2SKg6ENIV8KIpF.mat
|               pJCn2HRoxAsOoyqWootzoC.mat
|               pJvYrI60DTWOYz5hzjCU1C.mat
|               pOpe1FjwYyQMyJcal4AxOC.mat
|               PPIMDV8VK0eeDf72GgA1B.mat
|               PtefTVJJiEHI98YEsl4BwB.mat
|               pwMqYCuz2ZH2g1qKvZWJEF.mat
|               pWyFdj5juQtSRRXT9XRPDF.mat
|               pysAmQ6UeUVsI33Zow4PRG.mat
|               QCTK7tsWMaS7UdAVzpfP.mat
|               QGcSNKiOnp3yuaBW4EUErF.mat
|               qgMncmK5ZFgE1qyeXrzG6B.mat
|               Qh6DrHeIg52zQPamc6XGbG.mat
|               QhbWHXjH6qUj2JXyOoeNg.mat
|               qlWvMkqZiRFo2lPH01Em0E.mat
|               Qsr8ZdNkP4fSR0XG0wkD7G.mat
|               QTeELaeUivZWAWj6LotZvB.mat
|               QuqfCuk3VHFmNMyEMkcXBH.mat
|               qv7Azzp0awOpoS10LJ4pU.mat
|               QVCcB5Xn6Sijm1xzrtAkZG.mat
|               r0dWBUHDD8kS2Uck19jpQG.mat
|               R12sRBpHdy38eBla12W2WG.mat
|               r17bh2O4AgA1XeqcA3cFMH.mat
|               r4eMFIKkhuRXxPyjYSgCiF.mat
|               R8MVqj6YW0QrAtYjfmSewC.mat
|               r8RwpEhDBZXJy3JYeujV7D.mat
|               RaQp4zocFRR4WKc7OOCynB.mat
|               rKGYnRJQguDgrARCwe5OnD.mat
|               rQvYUSYFFg7YlF1h4JoYs.mat
|               rXRncrzcHHMYo4EjGpfvxG.mat
|               s04U0UDou2wkFVvn50aeNG.mat
|               si5RJMueaB0k63WR7LQVfD.mat
|               sMfF9cEsqSigPGCwFCSvqF.mat
|               sOM95h5f1FAyxe7IiHoVaB.mat
|               sortp3vgUCHFp912JTky3D.mat
|               T1Nmtc2u3gRcNoCN6vEJPB.mat
|               T3I3CUyh8Wgoc1CEyQDSEB.mat
|               t3mWzkMHJO1W50CU510tLH.mat
|               T4vki0sYsrLTeO6POdeAy.mat
|               t6FiIdvtaE3HSbL1NpxS1G.mat
|               t9XBeOXg2KAK6NdhTrx0HE.mat
|               TbrHIFOJWjZRk4Bs3YzKzC.mat
|               Tchd8DGZHRymFu00T6tNDB.mat
|               teYsgI8FbEI3vUELYKNZyE.mat
|               tfcYFINrz99g4V09DpjpCF.mat
|               Tjq03bwvsZigEt9NWqMNJE.mat
|               TSGsHfCjFQnYfkncyUaX0E.mat
|               tufejRfJWfyDFUJExIWtoC.mat
|               TVJWs6xdAqsYgxGLCl3ar.mat
|               tVxqKH9D9CZWv1g6GEWI8.mat
|               twKLIaC4PbVLl7p5lfBKdE.mat
|               ty50IhocwMSzAaUgQxR1rF.mat
|               U6s7Wq15CTubAdFyQTRzQB.mat
|               ufcxSJj9GIq2BA2NdQ7z0C.mat
|               uj3Xcllkrf4ohkNRkBJJ4D.mat
|               ujzAVGvDm1frDtf0wuPTiH.mat
|               uK8YyrHDxzXW2iRygueBdH.mat
|               ulZrsFvnQwVyuYNnSnoCAG.mat
|               Um4HzflI7sNHtVdQBvUXLF.mat
|               UmZJqxM0VXhRMe6fQzVIHH.mat
|               UVHw6tjq6sSZAYu8mzfI1E.mat
|               uympSaAXeK05ZN79Hg1oxE.mat
|               UziKeTWspMksCWqNKzIznD.mat
|               v6vtKHBNhEFidUEuLzQ06.mat
|               VcK0NNwdoiLAPJeBp5hGuG.mat
|               veO68YZdiI7UKilfqynH4D.mat
|               VhLgbzR4rahiVL9iTvaBHE.mat
|               VjkTcICAMKmVbCIC8PI8fG.mat
|               VKUOvrywuMF3dl4FvgTCSF.mat
|               vmYR1d4Ro1b7zGa1A3NLDE.mat
|               VUFI0mSAhxKFtnqVJluPyC.mat
|               VVnJpzFJjFGDFB3SJMH3YC.mat
|               VvtEv6ck0ONHBRvF3AvoMB.mat
|               vxJtlkWpgzdZNHg7ARaHIE.mat
|               vYKkIkC7akLoUUnBojLSDC.mat
|               W0KmoMTUq4gTz5QDp2gnPB.mat
|               W1xo2ApYycxqHYFaGTtp9G.mat
|               wBE3sbGiSun4ZvQ458LBFC.mat
|               WCxfi9cfF3ZPJ8p7ekeEVC.mat
|               wER0pCaCCBiUahCn27d0KF.mat
|               WEvTzg6tDGYgB8FGzKR5tC.mat
|               Wgjbv71qFSWjByY22O2QB.mat
|               wnwMhHxoqhwYhSh4OrNJL.mat
|               wOATd53tvDejEMfOJgmqXH.mat
|               WpOv6RLIO7evLWUTE6LzpF.mat
|               wQXiGV2SnJouNjxrL3UmYH.mat
|               wRdFZ1nZXV3yjPAIMSnd6B.mat
|               wvtG6PQIUCj7pjytSoHALG.mat
|               WVvWk1a9pRVUtyRleqKywB.mat
|               Wx6tbZ8yHqsXGl0IpNGAHE.mat
|               WxJWALTKgJDmfDHldA2VEB.mat
|               wyjeOWxf63wHi1ZjJn1TjF.mat
|               x2vLSfrDvgl8mZ6baj5eRD.mat
|               X6isf5g9YK82E9LK6gdufB.mat
|               X9n0s0ijvTfnmYFemLLF4D.mat
|               Xbo2IGM4NxFJzptFRODA3D.mat
|               XDEtKfUkjVQ8msYFJGR9RF.mat
|               XEEryy8QihI72gUzRsDhTG.mat
|               XiNCBy4NG3W4Yvhd0qwmOD.mat
|               xiqJQ7fQ43kJrCWE6y3yUH.mat
|               XjtlzGqJAN8IQdyXM85xgB.mat
|               XkJlL5SSpZMXx1ggFbIYm.mat
|               XoR3eojArBc8t5g2HcTB2C.mat
|               xPIwawUnSNNu6uToC3KeLC.mat
|               Xq1k2uXdt3zjrCv5eKo1kB.mat
|               XruDkw6o50vbiyqf4H0uq.mat
|               xxq3HcRluQJIgJNWX10UgB.mat
|               XxwJAW6edz123onSGr5tPC.mat
|               xZIGZ9mREr1WvjDHydH0oG.mat
|               xZRHzhsbw4hKiBGT21jQNG.mat
|               xzSb4o8vFCPrykPpsEN9qB.mat
|               Y30s4RX9VtPdBA328hktqF.mat
|               y3X4mg7YfbodlnsT3VLrQH.mat
|               Y7zIL2KjbRaGo1KyYGYgwF.mat
|               YBjcZHmKXxrfVfancTMaGB.mat
|               YH3KCBAg0N6nf1Zlk8cmlH.mat
|               yJEdWbtfWVmD1DBglVdwTC.mat
|               YjqJrDir0JRiAjir8Rxd4C.mat
|               YJrYurewaQJHrgpSrgYtUH.mat
|               yNXu6wbiOyHq9os8tqfuvD.mat
|               yreD2hB3JDPqKSc5wTTEWF.mat
|               yrmQxqSDKFcL2HkXoH0F2F.mat
|               YXN4awwz6iRlbABVraO29D.mat
|               Z0ZAZhmW0iA5twO5kMPEeH.mat
|               Z9XOiyAWxsmUmXQYse2mtB.mat
|               zhceeNFqMj5YYF3r8mez7.mat
|               ZhZKtJXnCucco3hR5GAWTB.mat
|               ZiF5n7RzHGXmyvLmRZEt0G.mat
|               zjIPxqjDNPffT8a1vwivNF.mat
|               zoJtBSyGBFryTy0gV3eoeH.mat
|               ZSuzjAw65TvYyVQkNr8JiE.mat
|               Zu0KAqqcKPB30Rkmrt7y8E.mat
|               zXNFl2FvYmM9BMVNbj5UPE.mat
|               ZZVpTJZPlnVOo6txlKWpJE.mat
|
\---System-Analysis and Controller-Designn Workspace
        ideas.txt
        Planning etc.txt
        system_analisys_and_controller_design.m





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