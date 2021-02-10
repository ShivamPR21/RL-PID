## Cartpole-PID  
  
> The original `Cartpole-v1` environment is adapted as `Cartpole-PID` to use `Proportional-Integral-and-Derivative`  control to make system stable and balancing the pole to center with reducing translational drift.  

### Actions  
  
| S.No | Action        | Type    | Name              |  
|-----| :-----------: |:-------:| -----------------:|  
|1    | K<sub>P</sub> | Box(1D) | Proportional gain |  
|2    | K<sub>D</sub> | Box(1D) | Derivative gain   |  
|3    | K<sub>I</sub> | Box(1D) | Integral gain     |  
 
 
### State

| S.No | State        | Name         | Range              |  
|-----| :-----------: | :----------: | -----------------: |  
|1    | $X$           | Position     | $[-2.4, 2.4]$      |  
|2    | $\dot{X}$     | Velocity     | $[-\infty, \infty]$|  
|3    | $\ddot{X}$    | Acceleration | $[-\infty, \infty]$|
|-----| ------------- | ------------ | ------------------ |
|4    | $\theta$        | Angle     | $[-14\degree, 14\degree]$|  
|5    | $\dot{\theta}$  | Angiular Velocity | $[-\infty, \infty]$|  
|6    | $\ddot{\theta}$ | Angular Acceleration | $[-\infty, \infty]$|  

> Stable state is consider to be $X_0=0, \dot X_0=0, \theta_0=0\degree, \dot \theta_0=0\degree$

### Differential PID  
  
Each action updates the gains of `PID-control` for that specific time step. The environment uses  
differential form of PID to get the force value updates from `PID-framework` for `sampling time   
step` $\tau$ and applies to the cartpole physics as form of `force to sliding block` $f$.

$\delta_{e_t} = \theta_t - \theta_{target}$
$\dot\delta_{e_t} = \frac{\delta_{e_t} - \delta_{e_{t-1}}}{\tau}$
$\ddot\delta_{e_t} = \frac{\dot\delta_{e_t} - \dot\delta_{e_{t-1}}}{\tau^2}$

$\dot\delta_u = K_P\dot\delta_{e_t} + K_I\delta_{e_t} + K_D\ddot\delta_{e_t}$
$u_t = u_{t-1}  + \dot\delta_u*\tau$

> Here $u_t$ is mapped directly to `force on sliding block` $f$ and after that the original Cartpole-v1 physics takes on.

### Noise modelled in state estimation

The noise is modelled as a gaussian noise with amplitude of 1/100th of the estimated state values i.e.
| S.No | State          | Noise                        |  
|-----| :-------------: | :--------------------------: |  
|1    | $X$             | $(2N(0, 1)-1)*X$             |  
|2    | $\dot{X}$       | $(2N(0, 1)-1)*\dot{X}$       |  
|3    | $\ddot{X}$      | $(2N(0, 1)-1)*\ddot{X}$      |
|-----| --------------- | ---------------------------- |
|4    | $\theta$        | $(2N(0, 1)-1)*\theta$        |  
|5    | $\dot{\theta}$  | $(2N(0, 1)-1)*\dot{\theta}$  |  
|6    | $\ddot{\theta}$ | $(2N(0, 1)-1)*\ddot{\theta}$ |

### Rewards

The `Cartpole-PID` uses a continuous reward function to get a dense reward set given as follows:-
$r_x = \exp^{\frac{-(X_t - X_0)^2}{2}} + \exp^{\frac{-(\dot{X}_t - \dot{X}_0)^2}{2}}$
$r_\theta = \exp^{\frac{-(\theta_t - \theta_0)^2}{0.02}} + \exp^{\frac{-(\dot{\theta}_t - \dot{\theta}_0)^2}{0.02}}$

$R = 0.25*r_x + 0.75*r_\theta$
