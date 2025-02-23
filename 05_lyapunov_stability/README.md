# 1. Lyapunov Stability: Fundamental Concepts

## 1.1 Definitions

### Equilibrium Point

For a nonlinear system
\[
\dot{x} = f(x,t)
\]
with state vector \( x \in \mathbb{R}^n \), an equilibrium point \( x_e \) satisfies
\[
f(x_e) = 0.
\]
(Often the origin \( x = 0 \) is chosen.)

### Stability in the Sense of Lyapunov

The equilibrium \( x_e \) is **Lyapunov stable** if, for every \( \varepsilon > 0 \), there exists a \( \delta > 0 \) such that
\[
\|x(0)-x_e\| < \delta \quad \Longrightarrow \quad \|x(t)-x_e\| < \varepsilon,\quad \forall\, t \ge 0.
\]
This means that trajectories starting within a \( \delta \)-neighborhood remain within an \( \varepsilon \)-neighborhood for all future time.

### Asymptotic Stability

The equilibrium is **asymptotically stable** if it is Lyapunov stable and, in addition,
\[
\lim_{t \to \infty} x(t) = x_e.
\]

---

# 2. Approaches to Lyapunov Stability Analysis

## 2.1 Indirect (Linearization) Method

### Idea

Linearize the nonlinear system around the equilibrium using a Taylor series expansion:
\[
\dot{x} = f(x) \approx Ax + G(x),
\]
where 
\[
A = \left.\frac{\partial f}{\partial x}\right|_{x=x_e}
\]
and \( G(x) \) contains higher–order terms that vanish as \( x \to x_e \).

### Stability Condition

The nonlinear system is locally asymptotically stable if the linearized system
\[
\dot{x} = Ax
\]
is asymptotically stable (i.e., all eigenvalues of \( A \) have negative real parts).

### Limitations

- The method is valid only for initial conditions close to the equilibrium.
- If some eigenvalues of \( A \) are zero, conclusions on stability become inconclusive.

## 2.2 Direct (Lyapunov Function) Method

### Lyapunov Function Candidate

Select a scalar, continuously differentiable function \( V(x) \) that satisfies:

- **Positive Definiteness:**
  \[
  V(0) = 0 \quad \text{and} \quad V(x) > 0 \quad \text{for } x \ne 0.
  \]
- **Negative Definiteness of its Time Derivative:**
  \[
  \dot{V}(x) = \frac{\partial V}{\partial x}\,f(x) < 0, \quad \forall\, x \ne 0.
  \]

### Conclusion

If such a function exists, the equilibrium \( x = 0 \) is asymptotically stable.  
*Note:* Finding a suitable Lyapunov candidate is generally non–systematic.

---

# 3. Examples of Lyapunov Stability Analysis

## 3.1 Nonlinear System Example (Indirect vs. Direct)

### Indirect Method Example

Consider a nonlinear system (with states \( x_1, x_2 \)) whose linearization yields
\[
\dot{x} = Ax, \quad \text{with} \quad A=\begin{bmatrix}0 & -1\\1 & 0\end{bmatrix}.
\]
The eigenvalues are purely imaginary (\( \lambda = \pm j\omega \)); therefore, linearization does not conclude asymptotic stability.

### Direct Method Example

For the same system, choose a Lyapunov candidate
\[
V(x) = \frac{1}{2}\left(x_1^2 + x_2^2\right),
\]
with time derivative
\[
\dot{V}(x) = x_1\dot{x}_1 + x_2\dot{x}_2.
\]
After substituting the dynamics, if one obtains
\[
\dot{V}(x) = (a-b)x_1^2 x_2,
\]
then by selecting parameters (e.g., \( a < b \)) so that \( \dot{V}(x) < 0 \) for all \( x \ne 0 \), the equilibrium is asymptotically stable.

## 3.2 Stability Analysis of Linear Systems

### Quadratic Lyapunov Function

For a linear system \( \dot{x} = Ax \), choose
\[
V(x) = x^T P x,
\]
with \( P > 0 \) (a symmetric, positive definite matrix).

### Lyapunov Equation

The time derivative is
\[
\dot{V}(x) = x^T \left(A^T P + P A\right)x.
\]
If we can find \( P \) such that
\[
A^T P + P A = -Q, \quad Q > 0,
\]
then
\[
\dot{V}(x) \le -x^T Q x < 0 \quad \text{for } x \ne 0.
\]

*Practical Note:* MATLAB’s `lyap` function can be used to solve this matrix equation.

---

# 4. Lyapunov-Based Control of Robotic Systems

Lyapunov methods are also used to design controllers that guarantee stability.

## 4.1 Single–Link Manipulator Example

### Manipulator Dynamics

A typical single–link manipulator can be modeled by:
\[
m l^2\,\ddot{q} + K\,\dot{q} + mgl\,\cos(q) = \tau,
\]
where:
- \( q \): Joint angle.
- \( m \), \( l \): Mass and length.
- \( g \): Gravitational acceleration.
- \( K \): Damping or friction coefficient.
- \( \tau \): Control torque.

### Tracking Objective

Let \( q_d \) be the desired trajectory. Define the tracking error:
\[
e = q_d - q,\quad \dot{e} = \dot{q}_d - \dot{q}.
\]

### Control Law Design

A feedback linearization–based controller might be:
\[
\tau = m l^2\,\ddot{q}_d + K_D\,\dot{e} + K_P\,e + K\,\dot{q} + mgl\,\cos(q),
\]
where \( K_P \) and \( K_D \) are positive gains.

### Closed–Loop Error Dynamics

Substituting the control law into the manipulator dynamics yields:
\[
m l^2\,\ddot{e} + K_D\,\dot{e} + K_P\,e = 0.
\]
Choosing \( K_P, K_D > 0 \) guarantees that \( e(t) \to 0 \) as \( t \to \infty \).

## 4.2 Multi–DOF Manipulator Example

### Robot Dynamics

For an \( n \)-DOF manipulator:
\[
M(\theta)\,\ddot{\theta} + C(\theta,\dot{\theta})\,\dot{\theta} + G(\theta) = \tau,
\]
where \( \theta \) is the joint angle vector.

### Control Law

A common feedback linearization control law is:
\[
\tau = M(\theta)\left(\ddot{\theta}_d + K_D\left(\dot{\theta}_d - \dot{\theta}\right) + K_P\left(\theta_d - \theta\right)\right) + C(\theta,\dot{\theta})\,\dot{\theta} + G(\theta),
\]
where \( \theta_d \) is the desired joint trajectory and \( K_P, K_D \) are positive–definite gain matrices.

### Closed–Loop Error Dynamics

The tracking error \( e = \theta_d - \theta \) then satisfies:
\[
\ddot{e} + K_D\,\dot{e} + K_P\,e = 0,
\]
ensuring asymptotic stability.

### Lyapunov Candidate for Analysis

A typical Lyapunov function candidate for the closed–loop system is:
\[
V(e,\dot{e}) = \frac{1}{2}\dot{e}^T M(\theta) \dot{e} + \frac{1}{2}e^T K_P e.
\]
Under standard properties (including the skew-symmetry of \( \dot{M} - 2C \)), one can show:
\[
\dot{V}(e,\dot{e}) = -\dot{e}^T K_D \dot{e} \le 0.
\]
This confirms asymptotic stability of the equilibrium \( e=0,\,\dot{e}=0 \).
