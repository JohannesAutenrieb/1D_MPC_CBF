# 1D_MPC_CBF

### Description
The 1D_MPC_CBF repository houses a collection of MATLAB scripts written to study differnt state-constrained controllers. With a focus on a class of 1D dynamical systems, this collection of MATLAB scripts intends to compare and analyze the performance of different state-constrained controllers utilizing control barrier functions and model predictive control.

#### Content

The following state-constrained controllers are implemented and can be compared via separate time simulations:

- **MPC controllers with classical state constraints
- **MPC controllers with discrete control barrier functions
- **Pointwise CLF-CBF-QP Controller

After each simulation run, a plot with results is given out. An example of such a plot is given here:

<p align=center>
<img src="https://github.com/JohannesAutenrieb/TeamACranfieldUAVSwarm/blob/master/img/GUI_MISSION_OVERVIEW.png" alt="MISSION_GUI" height=500px>
</p>


## Dependencies

The scripts use external libraries, which need to be installed.

- **YAMLIP
- **Export_fiq

Further, the following MATLAB toolboxes are needed:

- **Optimization Toolbox

**The software was tested with MATLAB 2020b under Windows 11 Home.*-** 


### License

The contents of this repository are covered under the [MIT License](LICENSE).


### References

We kindly acknowledge the following papers, which have been the foundation of the here presented scripts:
----------
* [[1] J. Zeng, B. Zhang, and K. Sreenath, “Safety-Critical Model Predictive Control with Discrete-Time Control
Barrier Function,” in 2021 American Control Conference (ACC), May 2021, pp. 3882–3889.](https://ieeexplore.ieee.org/document/9483029)
* [[2] A. D. Ames, X. Xu, J. W. Grizzle and P. Tabuada, "Control Barrier Function Based Quadratic Programs for Safety Critical Systems," in IEEE Transactions on Automatic Control, vol. 62, no. 8, pp. 3861-3876.](https://ieeexplore.ieee.org/document/7782377)
