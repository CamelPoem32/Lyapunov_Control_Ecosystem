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



# Lyapunov-based Control of a Predator-Prey Ecosystem

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
x = \begin{bmatrix} f \\ p \\ r \end{bmatrix}
$$

Where:
- $f$: amount of food,
- $p$: number of prey,
- $r$: number of predators.

---

## Action Space Description

The control action vector $u \in \mathbb{R}^3$ is defined as:

$$
u = \begin{bmatrix} u_1 \\ u_2 \\ u_3 \end{bmatrix}
$$

Where:
- $u_1$: control input to influence food supply,
- $u_2$: control input to influence prey population,
- $u_3$: control input to influence predator population.

---

## System Dynamics

The dynamics are modeled as a set of nonlinear differential equations:

$$
\begin{aligned}
\frac{df}{dt} &= kf - \alpha p + u_1 \\
\frac{dp}{dt} &= \beta fp - \gamma r + u_2 \\
\frac{dr}{dt} &= \delta pr - \mu r + u_3 \\
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

To stabilize the system to a desired target state $x^* = [f^*, p^*, r^*]^T$, we employ Lyapunov-based feedback control. The Lyapunov function is chosen as a quadratic energy function:

$$
V(x) = (x - x^*)^T Q (x - x^*)
$$

In our implementation, we use the **identity matrix** for $Q$, which simplifies the Lyapunov function to:

$$
V(x) = (f - f^*)^2 + (p - p^*)^2 + (r - r^*)^2
$$

---

### Properties of the Lyapunov Function

1. **Positive Semi-Definite**:  
   $$
   V(x) \geq 0 \quad \forall x
   $$
   with equality if and only if $x = x^*$.

2. **Time Derivative of the Lyapunov Function**:

   Let $e = x - x^*$. Then the derivative is:

   $$
   \dot{V}(x) = 2(f - f^*)\dot{f} + 2(p - p^*)\dot{p} + 2(r - r^*)\dot{r}
   $$

   The controller is designed so that $\dot{V}(x) \leq 0$, which ensures stability and convergence to the equilibrium point.

---

## Controller and Class Structure

The system is implemented using a modular, class-based structure with clear responsibilities:

---

### `Plant` and `Ecosystem`

- `Plant`: An abstract class defining the structure of a dynamical system.
- `Ecosystem`: Inherits from `Plant`. Implements the specific nonlinear dynamics:

```python
df = self.k * f - self.α * p + u1
dp = self.β * f * p - self.γ * r + u2
dr = self.δ * p * r - self.μ * r + u3
```

# Lyapunov-Based Control of a Predator-Prey Ecosystem

---

## Lyapunov Function Definition

We define the Lyapunov function as a simple sum of squared errors:

$$
V = \frac{1}{2}(x_1^2 + x_2^2 + x_3^2) = \frac{1}{2}(f^2 + p^2 + r^2)
$$

where \( x = \begin{bmatrix} f \\ p \\ r \end{bmatrix} \) is the current state of the system.

---

## Lyapunov Function Derivative

To analyze system stability, we compute the time derivative of \( V \):

$$
\dot{V} = f \dot{f} + p \dot{p} + r \dot{r}
$$

Substitute in the system dynamics (without control yet):

$$
\begin{aligned}
\dot{f} &= kf - \alpha p \\
\dot{p} &= \beta f p - \gamma r \\
\dot{r} &= \delta p r - \mu r \\
\end{aligned}
$$

Then:

$$
\begin{aligned}
\dot{V} &= f(kf - \alpha p) + p(\beta fp - \gamma r) + r(\delta pr - \mu r) \\
&= f^2 k - f p \alpha + \beta f p^2 - \gamma p r + \delta p r^2 - \mu r^2 \\
\end{aligned}
$$

---

## Reformulation for Nonzero Target State

Let:

$$
\tilde{f} = f - f^*, \quad \tilde{p} = p - p^*, \quad \tilde{r} = r - r^*
$$

Define Lyapunov function for deviation from the target:

$$
V = \frac{1}{2}(\tilde{f}^2 + \tilde{p}^2 + \tilde{r}^2)
$$

Then the time derivative becomes:

$$
\dot{V} = \tilde{f} \dot{f} + \tilde{p} \dot{p} + \tilde{r} \dot{r}
$$

With control included:

$$
\begin{aligned}
\dot{f} &= kf - \alpha p + u_1 \\
\dot{p} &= \beta f p - \gamma r + u_2 \\
\dot{r} &= \delta p r - \mu r + u_3 \\
\end{aligned}
$$

Now expand the derivative:

$$
\begin{aligned}
\dot{V} &= \tilde{f}(kf - \alpha p + u_1) + \tilde{p}(\beta f p - \gamma r + u_2) + \tilde{r}(\delta p r - \mu r + u_3) \\
&= \tilde{f} k f - \tilde{f} \alpha p + \tilde{f} u_1 + \tilde{p} \beta f p - \tilde{p} \gamma r + \tilde{p} u_2 + \tilde{r} \delta p r - \tilde{r} \mu r + \tilde{r} u_3
\end{aligned}
$$

Group by control inputs:

$$
\dot{V} = \text{nonlinear terms} + \tilde{f} u_1 + \tilde{p} u_2 + \tilde{r} u_3
$$

To ensure \( \dot{V} \leq 0 \), we choose control actions that cancel out the positive components.

---

## Control Law Derivation

From the theoretical derivation and code, control actions are:

```python
f, p, r = state[:3]
f_goal, p_goal, r_goal = state[:3] - target[:3]

a1 = -f * self.k + p * self.α - f_goal * self.k
a2 = -p * f * self.β + r * self.γ - p_goal * self.β
a3 = -r * p * self.δ + r * self.μ - r_goal * self.μ
```

## Detailed Expansion of Lyapunov Derivative

We previously wrote the derivative of the Lyapunov function as:

$$
\dot{V} = \tilde{f}(kf - \alpha p + u_1) + \tilde{p}(\beta f p - \gamma r + u_2) + \tilde{r}(\delta p r - \mu r + u_3)
$$

### Step 1: Expand all nonlinear terms

Distribute each term:

$$
\begin{aligned}
\dot{V} &= \tilde{f} \cdot kf - \tilde{f} \cdot \alpha p + \tilde{f} \cdot u_1 \\
&\quad + \tilde{p} \cdot \beta f p - \tilde{p} \cdot \gamma r + \tilde{p} \cdot u_2 \\
&\quad + \tilde{r} \cdot \delta p r - \tilde{r} \cdot \mu r + \tilde{r} \cdot u_3 \\
&= \text{(nonlinear dynamics)} + \tilde{f} u_1 + \tilde{p} u_2 + \tilde{r} u_3
\end{aligned}
$$

Now label and group the **nonlinear dynamics**:

- $ \tilde{f} \cdot kf $ is nonlinear (bilinear in $ \tilde{f}, f $),
- $ -\tilde{f} \cdot \alpha p $,
- $ \tilde{p} \cdot \beta f p $,
- $ -\tilde{p} \cdot \gamma r $,
- $ \tilde{r} \cdot \delta p r $,
- $ -\tilde{r} \cdot \mu r $

So:

$$
\begin{aligned}
\dot{V} &= \underbrace{\tilde{f}(kf - \alpha p) + \tilde{p}(\beta f p - \gamma r) + \tilde{r}(\delta p r - \mu r)}_{\text{Nonlinear system dynamics}} + \tilde{f} u_1 + \tilde{p} u_2 + \tilde{r} u_3
\end{aligned}
$$

---

## Step 2: Define control actions to cancel nonlinear terms

We now apply **feedback linearization** — design control inputs \( u_i \) that cancel the nonlinear parts:

$$
\begin{aligned}
u_1 &= -kf + \alpha p - \tilde{f} k \\
u_2 &= -\beta f p + \gamma r - \tilde{p} \beta \\
u_3 &= -\delta p r + \mu r - \tilde{r} \mu
\end{aligned}
$$

Now substitute into \( \dot{V} \):

$$
\dot{V} = \tilde{f} u_1 + \tilde{p} u_2 + \tilde{r} u_3 + \text{nonlinear terms}
$$

But the nonlinear terms cancel **exactly** with the first part of each \( u_i \), so only the last damping terms remain:

$$
\begin{aligned}
\dot{V} &= \tilde{f} (-\tilde{f} k) + \tilde{p} (-\tilde{p} \beta) + \tilde{r} (-\tilde{r} \mu) \\
&= -k \tilde{f}^2 - \beta \tilde{p}^2 - \mu \tilde{r}^2
\end{aligned}
$$

---

## Final Result

The time derivative of the Lyapunov function becomes:

$$
\dot{V} = -k \tilde{f}^2 - \beta \tilde{p}^2 - \mu \tilde{r}^2 \leq 0
$$

This is a **negative semi-definite** function. Each term is quadratic in the deviation and scaled by positive constants.

✅ Therefore, the system is globally stable under this control law.

---

## Interpretation: Feedback Linearization

This approach is a classic **feedback linearization** technique:

- The original nonlinear system dynamics are canceled exactly by the first term in each control input.
- The remaining control part adds a **linear damping** based on the distance to the goal.
- As a result, the system behaves like a stable linear system around the target state.

This technique ensures that the Lyapunov function decreases over time, proving **asymptotic convergence to the target state**.

---



## Simulation results
### No control applied
![Demo 1](https://github.com/CamelPoem32/Lyapunov_Control_Ecosystem/blob/master/animations/anim_death_control.gif)


### Control applied
![Demo 2](https://github.com/CamelPoem32/Lyapunov_Control_Ecosystem/blob/master/animations/animation_up_up_up.gif)


### Control applied
![Demo 3](https://github.com/CamelPoem32/Lyapunov_Control_Ecosystem/blob/master/animations/animation_down_down_down.gif)

## Phase Portrait
![Phase Portrait](https://github.com/CamelPoem32/Lyapunov_Control_Ecosystem/blob/master/animations/Phase_Portrait.gif)