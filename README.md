# 1D_MPC_CBF

## Description
The 1D_MPC_CBF repository houses a collection of MATLAB scripts written to study different state-constrained controllers. With a focus on a class of 1D dynamical systems, this collection of MATLAB scripts intends to compare and analyze the performance of different state-constrained controllers utilizing control barrier functions and model predictive control.

## Concept of Control Barrier Functions

We consider the following control affine system:

$$\\begin{equation}
    \dot{x}= f(x) + g(x)u
\\end{equation}$$

 A closed set $\mathcal{C} \in \mathbf{R}^n$, which we consider as a safe set, is defined in the following form:
  
  $$\\begin{equation}
      \mathcal{C} = \{ x \in \mathbf{R}^n : h(x) \geq 0 \}
  \\end{equation}$$
  
with $h: \mathbf{R}^n \times \mathbf{R}^p \to \mathbf{R}$ being a  continuously differentiable function, called  control barrier function (CBF).
  
<p align=center>
<img src="https://github.com/JohannesAutenrieb/1D_MPC_CBF/blob/main/Images/CBF_Function_Plot.png" alt="CBF_Function_Plot" height=300px>
</p>
  
The CBF can ensure for the presented control affine system that for any initial condition $x_0 := x(t_0) \in \mathcal{C}$, that $x(t)$ stays within $\mathcal{C}$ for any $t$, if there exists an extended class $\mathcal{K}$ functions $\alpha$ such that for all $x \in Int(\mathcal{C})$

$$\\begin{equation}
    \sup_{u \in U} [L_f B(x) + L_g B(x) u  + \alpha(h(x))] \geq 0
\\end{equation}$$

Where $\alpha(h(x)$ often chosen to be

$$\\begin{equation}
    \alpha(h(x)) = \gamma h(x)
\\end{equation}$$

with $\gamma$ being $\gamma > 0$.

## Integrated Controllers

The following state-constrained controllers are implemented and can be compared via separate time simulations:

### MPC controllers with classical state constraints

  $$\\begin{aligned}
  \\min\_{u_{t: t+N-1 \\mid t}} \\quad & \\frac{1}{2} x\_N^T P x_N+\\sum\_{k=0}^{N-1} \\frac{1}{2} x\_k^T Q x\_k+\\frac{1}{2} u_k^T R u\_k\\\\
  \\textrm{s.t.} \\quad 
   & x\_{t+k+1 \\mid t}=f\\left(x\_{t+k \\mid t}, u\_{t+k \\mid t}\\right), \\quad k=0, . ., N-1,\\\\
   & x\_{\\min } \\leq x\_{t+k \\mid t} \\leq x\_{\\max }, \\quad k=0, \\ldots, N-1,\\\\
   & u\_{\\min } \\leq u\_{t+k \\mid t} \\leq u\_{\\max }, \\quad k=0, \\ldots, N-1,   \\\\
   & x\_{t \\mid t}=x\_t,   \\\\
   & \\Delta h\\left(x\_{t+k \\mid t}, u\_{t+k \\mid t}\\right) \\geq-\\gamma h\\left(x\_{t+k \\mid t}\\right), \\quad k=0, \\ldots, N-1 \\\\
  \\end{aligned}$$

### MPC controllers with discrete control barrier functions


  $$\\begin{aligned}
  \\min\_{u_{t: t+N-1 \\mid t}} \\quad & \\frac{1}{2} x\_N^T P x_N+\\sum\_{k=0}^{N-1} \\frac{1}{2} x\_k^T Q x\_k+\\frac{1}{2} u_k^T R u\_k\\\\
  \\textrm{s.t.} \\quad 
   & x\_{t+k+1 \\mid t}=f\\left(x\_{t+k \\mid t}, u\_{t+k \\mid t}\\right), \\quad k=0, . ., N-1,\\\\
   & \\Delta h\\left(x\_{t+k \\mid t}, u\_{t+k \\mid t}\\right) \\geq-\\gamma h\\left(x\_{t+k \\mid t}\\right), \\quad k=0, \\ldots, N-1 \\\\
   & u\_{\\min } \\leq u\_{t+k \\mid t} \\leq u\_{\\max }, \\quad k=0, \\ldots, N-1,   \\\\
   & x\_{t \\mid t}=x\_t,   \\\\
  \\end{aligned}$$
  
### Pointwise Controller and CBF-QP Safety Filter

A linear controller of the following form is defined:

$$\\begin{align}
u_d = K_1 x + K_2 x_d
\\end{align}$$

The linear controller is tuned regarding the desired control performance but alone cannot generate safe commands. Therefore the following CLF-QP safety filter is used to adapt $u_d$ such that $x(t)$ stays within $\mathcal{C}$ for any $t$.

$$\\begin{align}
&\min_{u \in \mathcal{U}}
\begin{aligned}[t]
  &\|u - u_d\|_2
\end{aligned} \\
&\text{s.t.} \notag \\
& L_f h(x) + L_g h(x)u - \gamma h(x) \leq 0, \notag
\\end{align}$$

After each simulation run, a plot with results is given out. An example of such a plot is given here:

<p align=center>
<img src="https://github.com/JohannesAutenrieb/1D_MPC_CBF/blob/main/Images/Example_Simulation_Results.png" alt="MISSION_GUI" height=1000px>
</p>

## Dependencies

The scripts use external libraries, which need to be installed.
* [YAMLIP](https://github.com/yalmip/YALMIP)
* [Export_fig](https://github.com/altmany/export_fig)


Further, the following MATLAB toolboxes are needed:
* [Optimization Toolbox](https://www.mathworks.com/products/optimization.html)


**The software was tested with MATLAB 2020b under Windows 11 Home.** 


## License

The contents of this repository are covered under the [MIT License](LICENSE).


## References

We kindly acknowledge the following papers, which have been the foundation of the here presented scripts:

* [[1] J. Zeng, B. Zhang, and K. Sreenath, “Safety-Critical Model Predictive Control with Discrete-Time Control
Barrier Function,” in 2021 American Control Conference (ACC), May 2021, pp. 3882–3889.](https://ieeexplore.ieee.org/document/9483029)
* [[2] A. D. Ames, X. Xu, J. W. Grizzle and P. Tabuada, "Control Barrier Function Based Quadratic Programs for Safety Critical Systems," in IEEE Transactions on Automatic Control, vol. 62, no. 8, pp. 3861-3876.](https://ieeexplore.ieee.org/document/7782377)
