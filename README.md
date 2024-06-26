# Biped-Motor-Control
Optimal Control System for Motor Control of a Bipedal Robot

<p align="center">
  <img src="mediafolder/jaadlity.png" alt="Stick Fig" width="400"/>
</p>


# Death on Two Legs, an Optimal Control Simulation

**Authors:** A. Powell, J. Smith, T. Christensen, W. Lewis

## Abstract

We attempt at modelling how to move a biped across a flat surface. Given a point of connection and 2 legs acting as double-pendulums, we try to minimize the distance between the robots final state and a given point (desired destination) using the angles of each leg as control.

<!--## Background/Motivation  optional do later-->

## Modeling

### Initial Modelling

To model the robot's movement, we are treating the legs as double pendulums while in air but inverted double pendulums while on the ground. A $\theta$ parameter represents the angles of the "thigh" pendulums, and a $\phi$ parameter represents the angle of the "calf" pendulums. Our state $\vec{x}$, represents the position of the top of the legs where they connect (see figure above).

Our model tries to minimize the amount the legs move, and the time spent moving for $\vec{x}$ to reach a final state. We constrain our control to possible angles similar to the standard human range of motion: 

$-10\degree \leq \theta_i \leq 90\degree$
$-90\degree + \theta_i \leq \phi_i \leq 0\degree + \theta_i$

##### In the interest of generating a problem we can solve, we decide to abstract it from real world physics, seeing if we can encourage similar behavior mathematically. In other words, we do not consider things such as friction or mass but instead have rules such as the foot being on the ground should fix it in place or neither foot on the ground causes a negative velocity in the y direction.

Given the constraints in the previous section, we have devised the following cost functional, with our desired final position as $y^*$.

$$\int_0^{t_f}(\alpha_3 u_1)^2 + (\alpha_3 u_2)^2 + (\alpha_4 u_3)^2 + (\alpha_4 u_4)^2 + (\alpha(y - y^*)^4 dt$$

Since our control is defined on a compact set one might think the optimal control would end up being bang-bang but at any discrete timestep we want to avoid extreme motion in the legs. To achieve this we normalize the control by the desired maximum amount of motion at a timestep (such as 5 degrees of change from one frame to another) and then square this term leading to the $(\alpha_3 u_1)^2$ and similar terms in the cost function.

<p align="center">
  <img src="mediafolder/CostFig.png" alt="Stick Fig" width="400"/>
</p>

We then call the following our state:

$x_i' = l_2(\phi_i')\cos\phi_i + l1(\theta_i')\cos\theta_i$
$y_i' = l_2(\phi_i')\sin\phi_i + l_1(\theta_i')\sin\theta_i$

This comes from finding the position of the top point of a single double pendulum and then taking the derivative with respect to time to try and find the force it exerts on the point $\vec{x}$. Also note the derivative terms, we handle these by adding them to our control, making it 8-dimensional. 

Then define the following condition:

$c_i = y - (l_1\cos(\theta_i) + l_2\cos(\phi_i)) \leq 0$

This condition is supposed to tell use whether or not a foot (end of a double pendulum) is on the ground ($y=0$). Therefore, our state equation comes out to

$\vec{x}' = [x' y'] $

$x' =$ $x_1' + x_2'$ if $c_1$ and $c_2$; $x_1'$ if $c_1$; $x_2'$ if $c_2$; $0$ otherwise

$y' =$ $y_1' + y_2'$ if $c_1$ and $c_2$; $y_1'$ if $c_1$; $y_2'$ if $c_2$; $-\alpha_2$ otherwise

### Deriving a system of ODEs to solve

Using pontyagrins' maximum principle modified with KKT conditions for our inequality constraings we get the following modified lagrangian:

$\mathscr{L} = p_1\cdot x' + p_2\cdot y' - (\alpha_3 u_1)^2 - (\alpha_3 u_2)^2 - (\alpha_4 u_3)^2 - (\alpha_4 u_4)^2 - (\alpha(y - y^*)^4$

$- \mu_1(\phi_1 - \theta_1) - \mu_2(-\frac{\pi}{2} + \theta_1 - \phi_1) - \mu_3(-\frac{\pi}{18} - \theta_1) - \mu_4(-\frac{\pi}{2} + \theta_1)$

$- \mu_5(\phi_2 - \theta_2) - \mu_6(-\frac{\pi}{2} + \theta_2 - \phi_2) - \mu_7(-\frac{\pi}{18} - \theta_2) - \mu_8(-\frac{\pi}{2} + \theta_2),$

This then gives us the costate evolution of:

$\vec{p'} = [-\frac{DL}{Dx},  -\frac{DL}{Dy}]= [0, 4(x_1(y-y^*)^3]$

$\vec{p'}(t_f) = -\frac{\Delta\phi}{\Delta\vec{x}(t_f)} = \vec{0}$

From $\frac{D\mathscr{L}}{D\vec{u}} = \vec{0}$ we get the following bang-bang solutions and analytical solutions:

$\tilde{\theta}_1' = \alpha_3\dot\[1,  \text{if } p_1(\ell_1 \cos\theta_1) + p_2(\ell_2\sin\theta_1) < 0; -1, \text{if } p_1(\ell_1 \cos\theta_1) + p_2(\ell_2\sin\theta_1) > 0,\]$

$\tilde{\theta}_2' = \alpha_3\dot\[1,  \text{if } p_1(\ell_1 \cos\theta_2) + p_2(\ell_2\sin\theta_2) < 0; -1, \text{if } p_1(\ell_1 \cos\theta_2) + p_2(\ell_2\sin\theta_2) > 0,\]$

$\tilde{\phi}_1' = \alpha_3\dot\[1,  \text{if } p_1(\ell_1 \cos\phi_1) + p_2(\ell_2\sin\phi_1) < 0; -1, \text{if } p_1(\ell_1 \cos\phi_1) + p_2(\ell_2\sin\phi_1) > 0,\]$

$\tilde{\phi}_2' = \alpha_3\dot\[1,  \text{if } p_1(\ell_1 \cos\phi_2) + p_2(\ell_2\sin\phi_2) < 0; -1, \text{if } p_1(\ell_1 \cos\phi_2) + p_2(\ell_2\sin\phi_2) > 0,\]$

$\tilde{\phi}_2' = \alpha_3\dot\[1,  \text{if } p_1(\ell_1 \cos\phi_2) + p_2(\ell_2\sin\phi_2) < 0; -1, \text{if } p_1(\ell_1 \cos\phi_2) + p_2(\ell_2\sin\phi_2) > 0,\]$

<p align="center">
  <img src="mediafolder/tex1.png" alt="" width="500"/>
  <img src="mediafolder/tex2.png" alt="" width="500"/>
  <img src="mediafolder/tex4.png" alt="" width="500"/>
  <img src="mediafolder/tex3.png" alt="" width="500"/>
</p>

We can also use this equation to get ODEs for the derivative terms:

<p align="center">
  <img src="mediafolder/ode1.png" alt="" width="500"/>
  <img src="mediafolder/ode2.png" alt="" width="500"/>
  <img src="mediafolder/ode4.png" alt="" width="500"/>
  <img src="mediafolder/ode3.png" alt="" width="500"/>
</p>

### Code implementation and solution

We used scipy.integrate's solve_bvp function to solve these optimal control porblems in python. please see JaAdLiTy.py for the code.

#### Model Parameters and Equations

The parameters and equations governing motion in this updated model are listed in the table below.

| Parameter | Description | Tested Values |
|-----------|-------------------------|----------------|
| $\alpha_1$ | Height Constraint on the state | 2, 5, 10 |
| $\alpha_2$ | Freefall velocity ~mg | 9.81 |
| $\alpha_3$ | Penalty for thigh movement | $\frac{180}{5\pi}$ |
| $\alpha_3$ | Penalty for calf movement | $\frac{180}{5\pi}-\frac{180}{10\pi}$ |
| $\beta$ | Linearization for cosine | $0.63598-0.67561$ |

## Results

<p align="center">
  <img src="mediafolder/kneels.gif" alt="Stick Fig" width="400"/><img src="mediafolder/instability.gif" alt="Stick Fig" width="400"/>
</p>
As seen above we did not get the results we were hoping for. We struggled with numerical instability and overflow represented on the right gif above.  We also had a hard time motivating movement in the y-axis as shown by the figure on the left.  The model went through many iterations before we found a stable one. In order to get rid of overflow we use a logarithm to split some divisions and then exponentiated the results.  This worked suprising well and we started to see smooth/understandable results.  Unfortunately in all of our experiments we did not succeed in getting the stickman to walk as for why we will elaborate further in the analysis section.  Note the next two figures:

<p align="center">
  <img src="mediafolder/badstand.gif" alt="Stick Fig" width="400"/><img src="mediafolder/Figure_1.png" alt="Stick Fig" width="400"/>
</p>
<p align="center">
  <img src="mediafolder/badstand2.gif" alt="Stick Fig" width="400"/><img src="mediafolder/Figure_2.png" alt="Stick Fig" width="400"/>
</p>

While the syncronization between the two gifs may be off they are important because each one was solved with a different ODE.  One was solved using the bang-bang solution for the derivatives and the analytical solution for the rest of the control as seen above, the second one was solved by solving for the control using the ODEs found above.  This is important as it suggests a few things.  First it suggests that the analytical solutions and bang-bang switching times found are correct since a numerical solver gave the same control (see the plots of the control to the right).  Secondly it suggests that our stable code implementation probably doesn't have too much wrong with it and the resons we are not getting results is a problem with our state or cost functional we chose.  While this is disappointing it does at least define what is wrong and what could be fixed in future models.


<p align="center">
  <img src="mediafolder/yoga.gif" alt="Stick Fig" width="400"/><img src="mediafolder/controls(yoga).png" alt="Stick Fig" width="400"/>
</p>
<p align="center">
  <img src="mediafolder/kneelskinda.gif" alt="Stick Fig" width="400"/><img src="mediafolder/controls(kneelskinda).png" alt="Stick Fig" width="400"/>
</p>

The above two figures tell us more about some issues we might be having.  The top figure where our stick figure starts hot yoga demonstrates that depending on the conditions the inequality strengths are not costly enough to be enforced.  In the lower figure the stick figure finds some floating equilibrium where it's not standing on 0 as we desire but lower this suggests that the state is not evolving properly in the y direction, the velocity in y is somehow negative even though the conditions for the feet to be on the ground are cleared.

## Analysis

Due to the complicated nature of the problem, our model currently does not produce the desired result when applied via Python. A major problem is defining the evolution of the control when the foot is not on the ground.

With continual improvements to one of the many parameters of the problem, we foresee success with this project. For example, one improvement that we can make to this is changing the ODE that was derived from the cost functions and state equations by making minor changes to the behavior of the cost function or state. Another improvement would be to solve for one step and then feed the final conditions of taking one step into the initial conditions of the next step, thereby discretizing the walk.

From now, our current plan is to iterate on the equations we derived and continually update the code until we determine the optimal control for the bipedal robot. Given we can solve this issue, it can be the basis for more exciting and complicated problems in the future such as different terrain or changing parameters and costs to encourage/allow jumping.


#### Learning and Future Directions

The optimization of physical systems can become extremely difficult. Abstracting real world physics into mathematical rules motivated by cost is helpful in order to make this problem solvable in the context of our curriculum.  In the future we hope to redefine the state equation to help motivate 'natural' movement while a leg is not on the ground. We also hope to refine our parameters and the cost function, such as adding a cost for holding a leg in the air for too long. In addition to fixing the models interactions with the floor we hope that these changes will give us a model closer to our desired outcome.
