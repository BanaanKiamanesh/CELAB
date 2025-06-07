# CELAB-Practice

## Things to Consider

1. Everything is only developed and tested on **MATLAB R2024b**.
2. **BB** is an abbreviation for **Black Box Model** provided by the prof(all can be found in the *BB Model* directory).
3. If the BB does not work, go to the simulink icon of the BB, click on the little arrow on the left bottom side of it, inside, right click on the `model` and select `Explore` and in the freshly opened `Model Explorer` window, click `browse` and find the `.slxp` file associated with the BB. Apply the changes and save the file. This should fix the issue.
4. For each section, the only thing you need to do is to run the **Simulink** file usually named as `Model.slx`. It will automatically do the design, simulation, plotting and data saving(if it's supposed to).
5. When doing the validation on the simplified state-space/transfer function model, notice that the model output is in `rad` and not `deg` so the only conversion needed is to convert input to `rad`(LABs 0, 1, 2).
6. When doing the **Discrete Time Simulations**, is you choose to use a fixed-step solver, the step-size of the solver must be smaller than the controller sampling time, in order to properly simulate how the continuous-time plant dynamics evolve in between each sampling time. Since the min sampling time in all the labs was 1ms, then the best thing to do in all cases, is to set max sampling time of the solver to 1ms.
7. MATLAB's `c2d` function doesn't support the **Forward Euler Discretization** method. For the purpose, `c2d_euler` is used. More info can be found [here](https://github.com/tamaskis/c2d_euler-MATLAB/tree/main) or [here](https://www.mathworks.com/matlabcentral/fileexchange/90267-euler-c2d-transformations-c2d_euler).
