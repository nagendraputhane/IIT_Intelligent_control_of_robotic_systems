# 1. Overview of Feedback Linearization

**Definition:**  
Feedback linearization is a control strategy in which a nonlinear system is transformed into a (closed–loop) linear system by applying a suitably designed control law. When this transformation is successful, the resulting error dynamics become linear and stable.

**Core Idea:**  
For a nonlinear system (e.g., the full robot dynamics), if one can design a control input \( u \) such that the closed–loop dynamics become linear (and stable), then classical linear control methods can be applied to achieve tracking or stabilization.

---

# 2. Feedback Linearization for Dynamics-Based Control

## 2.1 General Nonlinear Robotic Dynamics

A typical \( n \)-DOF robot manipulator is described by:
\[
\tau = M(\theta)\,\ddot{\theta} + C(\theta,\dot{\theta})\,\dot{\theta} + G(\theta)
\]
where:

- \(\tau\): Joint torque vector (\(n \times 1\))
- \(\theta\), \(\dot{\theta}\), \(\ddot{\theta}\): Joint angles, velocities, and accelerations respectively
- \(M(\theta)\): Inertia (mass) matrix
- \(C(\theta,\dot{\theta})\): Coriolis and centrifugal forces matrix
- \(G(\theta)\): Gravity vector

## 2.2 Designing a Feedback Linearizing Controller

### Partitioning the Dynamics

Write the dynamics in the form:
\[
\tau = \underbrace{M(\theta)}_{\alpha(\theta)}\,\ddot{\theta} + \underbrace{\left(C(\theta,\dot{\theta})\,\dot{\theta} + G(\theta)\right)}_{\beta(\theta,\dot{\theta})}
\]

### Control Law

Choose the control input as:
\[
\tau = M(\theta)\,\tau_d + C(\theta,\dot{\theta})\,\dot{\theta} + G(\theta)
\]
where the servo law (or virtual input) is defined as:
\[
\tau_d = \ddot{\theta}_d + K_D\,\dot{e} + K_P\,e
\]
with:
- \( e = \theta_d - \theta \) (tracking error)
- \( \dot{e} = \dot{\theta}_d - \dot{\theta} \)
- \( K_P \) and \( K_D \) are positive–definite gain matrices.

### Closed–Loop Error Dynamics

Substituting the control law into the robot dynamics yields:
\[
M(\theta)\left(\ddot{\theta}_d + K_D\,\dot{e} + K_P\,e\right) + C(\theta,\dot{\theta})\,\dot{\theta} + G(\theta) = M(\theta)\,\ddot{\theta} + C(\theta,\dot{\theta})\,\dot{\theta} + G(\theta)
\]
After cancellation and simplification, the resulting error dynamics become:
\[
\ddot{e} + K_D\,\dot{e} + K_P\,e = 0
\]
which is a standard linear second–order system. For positive gains, \( e(t) \to 0 \) as \( t \to \infty \).

---

# 2.3 Example: Single–Link Manipulator (Dynamics)

Consider a single–link manipulator with dynamics given by:
\[
m l^2\,\ddot{q} + B\,\dot{q} + mgl\,\sin(q) = \tau
\]
where:
- \( q \): Joint angle (generalized coordinate)
- \( m \), \( l \), \( g \): Mass, length, gravitational acceleration (e.g., \( m=1\,\text{kg} \), \( l=1\,\text{m} \), \( g=10\,\text{m/s}^2 \))
- \( B \): Damping (or terms representing Coriolis/centrifugal effects)

### Design Steps

1. **Define the Tracking Error:**  
   \[
   e = q_d - q
   \]
   where \( q_d \) is the desired joint angle.

2. **Select the Control Law:**  
   Choose:
   \[
   \tau = m l^2\,\ddot{q}_d + K_D\,e' + K_P\,e + B\,\dot{q} + mgl\,\sin(q)
   \]
   where \( \ddot{q}_d \) and \( e' \) denote the desired acceleration and the derivative of the error, respectively.

3. **Resulting Closed–Loop Dynamics:**  
   Substituting into the dynamic equation yields:
   \[
   m l^2\,\left(\ddot{q}_d - \ddot{q}\right) + B\,\left(q'_d - \dot{q}\right) + K_D\,e' + K_P\,e = 0
   \]
   which simplifies to:
   \[
   e'' + K_D\,e' + K_P\,e = 0
   \]
   a linear, stable second–order differential equation.

---

# 3. Feedback Linearization for Kinematics-Based Control

## 3.1 Differential Kinematics

- **Forward Kinematics:**  
  Given by:
  \[
  x = f(\theta)
  \]
- **Differential Kinematics:**  
  Taking the time derivative, we have:
  \[
  \dot{x} = J(\theta)\,\dot{\theta}
  \]
  where \( J(\theta) \) is the Jacobian matrix.

## 3.2 Designing a Kinematic Feedback Controller

- **Desired End–Effector Trajectory:**  
  Let \( x_d \) be the desired Cartesian position. Define the tracking error:
  \[
  e = x_d - x
  \]

- **Control Law in Joint Space:**  
  Choose the joint velocity as:
  \[
  \dot{\theta} = J^{-1}(\theta)\,\left(\dot{x}_d + K_P\,e\right)
  \]
  where \( K_P \) is a positive–definite gain matrix.

- **Closed–Loop Kinematics:**  
  Substituting into the differential kinematics yields:
  \[
  \dot{x} = J(\theta)\,J^{-1}(\theta)\,\left(\dot{x}_d + K_P\,e\right) = \dot{x}_d + K_P\,e
  \]
  Thus, the error dynamics become:
  \[
  \dot{e} + K_P\,e = 0
  \]
  which is linear and stable for \( K_P > 0 \).

---

# 4. Feedback Linearization for Multi–DOF Dynamic Control

For an \( n \)-DOF robot, the full dynamics are:
\[
\tau = M(\theta)\,\ddot{\theta} + C(\theta,\dot{\theta})\,\dot{\theta} + G(\theta)
\]
The control law is chosen as:
\[
\tau = M(\theta)\,\left(\ddot{\theta}_d + K_D\,\dot{e} + K_P\,e\right) + C(\theta,\dot{\theta})\,\dot{\theta} + G(\theta)
\]
with tracking error \( e = \theta_d - \theta \). This guarantees that:
\[
\ddot{e} + K_D\,\dot{e} + K_P\,e = 0
\]
which is linear and stable.
