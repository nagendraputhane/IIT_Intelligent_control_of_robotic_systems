# 1. Introduction to Backstepping Control

**Purpose:**  
Backstepping is a recursive design methodology developed (around 1990) for stabilizing nonlinear systems in strict feedback form. Its goal is to design a control law that stabilizes the origin (or forces tracking) by breaking the system into simpler subsystems.

**Key Idea:**  
In a multi–state nonlinear system, one designs virtual controls for subsystems in a recursive manner. The process starts by stabilizing a lower–order subsystem (using a “virtual control”) and then designing the actual control input so that the next subsystem follows the virtual control signal.

---

# 2. Backstepping for a Second–Order Nonlinear System

## 2.1 System in Strict Feedback Form

Consider a second–order nonlinear system:

- **Subsystem 1 (State \(x_1\)):**
  \[
  \dot{x}_1 = f_1(x_1) + g_1(x_1)\,x_2
  \]
  
- **Subsystem 2 (State \(x_2\)):**
  \[
  \dot{x}_2 = f_2(x_1, x_2) + u
  \]

Here, \(x_2\) is treated as a virtual control for the first subsystem. The design proceeds in two steps:

### Virtual Control Design

- **Step 1:**  
  Choose a desired virtual control \(x_{2d}\) to stabilize the first subsystem. Define the tracking error:
  \[
  e_1 = x_1 - x_{1d}
  \]
  where \(x_{1d}\) is the desired value for \(x_1\).

- **Step 2:**  
  A common choice is:
  \[
  x_{2d} = -K_1 e_1 + \dot{x}_{1d}
  \]
  which makes the error dynamics for subsystem 1 approximately:
  \[
  \dot{e}_1 = -K_1 e_1
  \]
  ensuring that \(e_1 \to 0\) for positive \(K_1\).

### Actual Control Design

- **Step 1:**  
  Design the control input \(u\) so that the second subsystem forces \(x_2\) to track \(x_{2d}\). Define the error:
  \[
  e_2 = x_2 - x_{2d}
  \]

- **Step 2:**  
  Choose \(u\) such that:
  \[
  \dot{e}_2 = -K_2 e_2
  \]
  ensuring that \(e_2 \to 0\) for a positive \(K_2\). With \(e_2 \to 0\), the virtual control is effectively implemented, and the overall system is stabilized.

---

## 2.2 Example: Single Link Manipulator

Given the dynamics:
\[
\begin{cases}
\dot{x}_1 = x_2, \\
\dot{x}_2 = -10\,\sin(x_1) + u,
\end{cases}
\]
the design proceeds as follows:

### Subsystem 1

- **Error Definition:**  
  \[
  e_1 = x_1 - x_{1d}
  \]

- **Virtual Control:**  
  \[
  x_{2d} = -K_1 e_1 + \dot{x}_{1d}
  \]
  
- **Resulting Error Dynamics:**  
  \[
  \dot{e}_1 = -K_1 e_1
  \]

### Subsystem 2

- **Error Definition:**  
  \[
  e_2 = x_2 - x_{2d}
  \]

- **Control Law:**  
  \[
  u = -10\,\sin(x_1) - K_2 e_2 + \dot{x}_{2d}
  \]
  
- **Resulting Error Dynamics:**  
  \[
  \dot{e}_2 = -K_2 e_2
  \]

Thus, with \(K_1, K_2 > 0\), both error signals decay to zero, and the single–link manipulator is stabilized.

---

# 3. Integrator Backstepping

Integrator backstepping is a special case of backstepping applied to systems where the control input appears through an integrator (or a chain of integrators). In this method, each subsystem is “augmented” with an additional integrator, and Lyapunov functions (or Liapunov function candidates) are defined for each subsystem.

## 3.1 General Idea and Setup

### System Description

Consider a nonlinear system with an integrator chain:
\[
\dot{x} = f(x) + g(x)\,\lambda, \quad f(0)=0, \qquad \dot{\lambda} = u,
\]
where \(\lambda\) is a virtual control input, and \(u\) is the actual control input.

### Design Steps

1. **Virtual Control Selection:**  
   Choose a stabilizing virtual control law \(\alpha(x)\) so that if \(\lambda = \alpha(x)\), the system
   \[
   \dot{x} = f(x) + g(x)\,\alpha(x)
   \]
   is stable.

2. **Error Definition:**  
   Define an error between the actual virtual control and its desired value:
   \[
   z = \lambda - \alpha(x)
   \]

3. **Lyapunov Function Candidate:**  
   Start with a candidate for the \(x\)–subsystem, e.g., 
   \[
   V(x)
   \]
   and augment it:
   \[
   V_A(x, z) = V(x) + \frac{1}{2}z^2.
   \]

4. **Control Law Design:**  
   Differentiate \(V_A(x, z)\) along the trajectories and choose \(u\) such that the derivative \(\dot{V}_A\) becomes negative definite. This involves:
   - Computing
     \[
     \dot{V}_A = \dot{V}(x) + z\,\dot{z},
     \]
   - Substituting for \(\dot{z}\) (which involves \(u\) and the derivative of \(\alpha(x)\)), and then
   - Selecting \(u\) to cancel undesirable terms.

---

## 3.2 Example of Integrator Backstepping

Consider a two–state system:

- **Subsystem 1:**
  \[
  \dot{x}_1 = -x_1^3 + x_2,
  \]
  
- **Subsystem 2:**
  \[
  \dot{x}_2 = x_2^3 + u.
  \]

### Design Steps

1. **Step 1:**  
   Treat \(x_2\) as the control input for subsystem 1.

2. **Step 2:**  
   Define the desired virtual control for \(x_2\) as:
   \[
   \alpha(x_1) = -K_1 x_1,
   \]
   and define the error:
   \[
   z = x_2 - \alpha(x_1).
   \]

3. **Step 3:**  
   Choose a Lyapunov candidate for \(x_1\):
   \[
   V(x_1) = \frac{1}{2} x_1^2.
   \]
   Then, define an augmented Lyapunov function:
   \[
   V_A(x_1, z) = V(x_1) + \frac{1}{2} z^2.
   \]

4. **Step 4:**  
   Differentiate \(V_A(x_1, z)\) with respect to time. The derivative involves:
   - Terms from \(\dot{x}_1\),
   - The derivative of \(\alpha(x_1)\) (which brings in \(\dot{x}_1\)),
   - And the dynamics of \(z\):
     \[
     \dot{z} = \dot{x}_2 - \frac{d\alpha}{dx_1}\,\dot{x}_1.
     \]

5. **Step 5:**  
   Choose the actual control \(u\) to cancel the nonlinearities and to introduce a damping term (typically of the form \(-K_2 z\)), so that:
   \[
   \dot{V}_A \leq -W(x_1) - K_2 z^2,
   \]
   ensuring that \(V_A\) decreases over time.

### Example Control Law

One simplified form (for illustration) might be:
\[
u = -K_2 z - \frac{d\alpha}{dx_1}\Bigl(-x_1^3 + x_2\Bigr) - x_2^3,
\]
where the derivative of \(\alpha(x_1)= -K_1 x_1\) is simply \(-K_1\). This control law is chosen so that the closed–loop error dynamics become stable.
