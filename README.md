# Lyapunov_Control_Ecosystem
Lyapunov Control of Predator_Prey_Food Ecosystem


![funny_picture](images/1461050888149073890.jpg)

---

## Problem Description

We simulate and control a simplified ecosystem using a dynamical systems framework. The ecosystem consists of three interacting populations:

- **Food (F)**: A renewable resource.  
- **Prey (P)**: Herbivores that consume food.  
- **Predators (R)**: Carnivores that consume prey.  

Our objective is to control this system using feedback such that the state $(F, P, R)$ converges to a desired target equilibrium.

---

## State Space Description

The system's state is described by the vector:

$$
x = \begin{bmatrix} f \\ 
p \\ 
r \end{bmatrix}
$$

Where:
- $f$: amount of food,
- $p$: number of prey,
- $r$: number of predators.

---

## Action Space Description

The control action vector $u \in \mathbb{R}^3$ is defined as:

$$
a = \begin{bmatrix} a_1 \\ 
a_2 \\ 
a_3 \end{bmatrix}
$$

Where:
- $a_1$: control input to influence food supply,
- $a_2$: control input to influence prey population,
- $a_3$: control input to influence predator population.

---

## System Dynamics

The dynamics are modeled as a set of nonlinear differential equations:

$$
\begin{aligned}
\frac{df}{dt} &= kf - \alpha p + a_1 \\
\frac{dp}{dt} &= \beta fp - \gamma r + a_2 \\
\frac{dr}{dt} &= \delta pr - \mu r + a_3 \\
\end{aligned}
$$

Where:
- $k$: food influx rate,
- $\alpha$: rate of food consumption by prey,
- $\beta$: prey growth efficiency due to food,
- $\gamma$: predation rate,
- $\delta$: predator growth efficiency due to prey,
- $\mu$: predator death rate.

---

## Lyapunov-Based Control

To stabilize the system to a desired target state ![target](images/equation.jpg), we employ Lyapunov-based feedback control. The Lyapunov function is chosen as a quadratic energy function:

![energy](images/lyap.jpg)

In our implementation, we use the **identity matrix** for $Q$, which simplifies the Lyapunov function to:

![energy1](images/eng1.jpg)


---

### Properties of the Lyapunov Function

1. **Positive Semi-Definite**:  

   ![psd](images/psd.jpg)

   with equality if and only if $x = x^*$.

2. **Time Derivative of the Lyapunov Function**:

   Let $e = x - x^*$. Then the derivative is:

   ![V_dot](images/V_dot.jpg)
   

   The controller is designed so that $\dot{V}(x) \leq 0$, which ensures stability and convergence to the equilibrium point.

---

## Controller and Class Structure

The system is implemented using a modular, class-based structure with clear responsibilities:

---

### `Plant` and `Ecosystem`

- `Plant`: An abstract class defining the structure of a dynamical system.
- `Ecosystem`: Inherits from `Plant`. Implements the specific nonlinear dynamics:


## Simulation results
### No control applied
![Demo 1](https://github.com/CamelPoem32/Lyapunov_Control_Ecosystem/blob/master/animations/anim_death_control.gif)


### Control applied
![Demo 2](https://github.com/CamelPoem32/Lyapunov_Control_Ecosystem/blob/master/animations/animation_up_up_up.gif)


### Control applied
![Demo 3](https://github.com/CamelPoem32/Lyapunov_Control_Ecosystem/blob/master/animations/animation_down_down_down.gif)

## Phase Portrait
<video src="https://github.com/CamelPoem32/Lyapunov_Control_Ecosystem/blob/master/animations/Phase_Portrait.mp4"></video>