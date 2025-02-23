# 1. Kinematics

**Definition:**  
Kinematics is the study of motion without considering the forces that cause it. In robotics, it examines the relationship between joint variables (angles or displacements) and the Cartesian position and orientation of the end-effector.

## 1.1 Forward Kinematics

### Concept
- **Input:** Joint variables (e.g., angles θ for revolute joints, distances D for prismatic joints).
- **Output:** The position and orientation of the end-effector.
- There is a one-to-one mapping: Given a set of joint values, the end-effector’s pose is uniquely determined.

### Denavit–Hartenberg (DH) Parameters
To model a serial-link manipulator, four DH parameters are used for each joint:
- **\( a_i \) (link length):** Distance along \( X_i \) from the previous joint’s origin to the intersection with the current joint’s \( Z \) axis.
- **\( d_i \) (link offset):** Distance along \( Z_{i-1} \) from the previous origin to the intersection with the current \( X_i \) axis.
  - \( d_i \) is variable for prismatic joints and constant for revolute joints.
- **\( \alpha_i \) (link twist):** The angle between \( Z_{i-1} \) and \( Z_i \) measured about \( X_i \).
- **\( \theta_i \) (joint angle):** The angle between \( X_{i-1} \) and \( X_i \) measured about \( Z_{i-1} \).
  - \( \theta_i \) is variable for revolute joints.

### Homogeneous Transformation Matrix
Each joint’s contribution is represented by a 4×4 matrix combining rotation and translation:
- Rotate about \( Z \) by \( \theta \).
- Translate along \( Z \) by \( d \).
- Translate along \( X \) by \( a \).
- Rotate about \( X \) by \( \alpha \).

Multiplying these matrices for all joints yields the overall transformation \( T^0_n \) that relates the base frame to the end-effector frame. The last row `[0 0 0 1]` is used for scaling/simplification in homogeneous coordinates.

### DH Algorithm Steps
1. **Establish Joint Axes:** Label joint axes as \( Z_0, Z_1, \dots, Z_{n-1} \).
2. **Define Base Frame:** Place the origin \( O_0 \) and ensure the frame follows the right-hand rule.
3. **Assign \( X_i \) Axes:**
   - For intersecting axes: \( X_i \) is normal to the plane of \( Z_{i-1} \) and \( Z_i \).
   - For parallel axes: \( X_i \) is chosen along the common normal.
4. **Define \( Y_i \) Axes:** Use the right-hand rule to complete the frame.
5. **Construct the DH Table:** List \( a_i \), \( d_i \), \( \alpha_i \), and \( \theta_i \) for each joint.
6. **Compute Transformation Matrices:** Multiply them to obtain \( T^0_n \).

### Example
For a two-revolute joint nonplanar robot:
- Form two rows of the DH table (one for each joint).
- Calculate \( T^0_1 \) and \( T^1_2 \).
- The final transformation \( T^0_2 = T^0_1 \times T^1_2 \) gives the end-effector’s pose.
  - The last column of \( T^0_2 \) gives the position.
  - The upper 3×3 block gives the orientation (from which Euler angles like \( \alpha \), \( \beta \), and \( \gamma \) can be extracted).

## 1.2 Inverse Kinematics

### Concept
- **Input:** Desired end-effector pose (position and orientation).
- **Output:** Joint variables that achieve that pose.

### Methods
- **Direct/Algebraic Approach:**  
  Equate the homogeneous transformation matrix from forward kinematics with the desired pose and solve the nonlinear equations for joint variables.
  
- **Geometric Approach:**  
  Use the geometric relationships between links and joints to derive the necessary joint values.
  
- **Closed-Loop Inverse Kinematics (Jacobian-Based):**  
  Use a feedback control law involving the Jacobian matrix. For example:
  \[
  \dot{q} = J^{-1} \dot{x}_d + K_P (x_d - x)
  \]
  where:
  - \( \dot{x}_d \) is the desired end-effector velocity,
  - \( (x_d - x) \) is the error,
  - \( K_P \) is a gain matrix.

### Challenges
- Multiple solutions may exist for a given end-effector pose.
- Optimization criteria (e.g., minimizing energy or maximizing manipulability) may be used to select the best solution.

# 2. Statics

**Definition:**  
Statics is the study of forces in systems at equilibrium (no motion). In robotics, it relates the forces at the end-effector to the joint torques required to maintain equilibrium.

### Basic Relationship
\[
\tau = J^T \gamma
\]
where:
- \( \tau \) is the vector of joint torques.
- \( \gamma \) is the force (and moment) vector at the end-effector.
- \( J \) is the manipulator’s Jacobian matrix.

**Note:**  
This relationship is used to understand how forces applied by or to the end-effector translate into torques at the joints.

# 3. Dynamics

**Definition:**  
Dynamics is the study of motion taking into account the forces that cause it. In robotics, the dynamics equations model how applied torques and forces affect the joint accelerations and overall motion of the robot.

## 3.1 Dynamic Model

For an \( n \)-degree-of-freedom robot:
\[
\tau = M(\theta)\,\ddot{\theta} + V(\theta,\dot{\theta})\,\dot{\theta} + G(\theta)
\]
where:
- \( \tau \): Joint torque vector (\( n \times 1 \)).
- \( M(\theta) \): Inertia matrix (\( n \times n \)).
- \( V(\theta,\dot{\theta}) \): Coriolis and centrifugal matrix (\( n \times n \)).
- \( G(\theta) \): Gravity vector (\( n \times 1 \)).
- \( \dot{\theta} \) and \( \ddot{\theta} \): Joint velocity and acceleration vectors, respectively.

## 3.2 Forward vs. Inverse Dynamics

### Forward Dynamics
- **Input:** Joint torques \( \tau \).
- **Output:** The resulting joint accelerations \( \ddot{\theta} \) (and thus trajectories).
- Often used in simulations.

### Inverse Dynamics
- **Input:** Desired joint trajectories (positions, velocities, accelerations).
- **Output:** The required joint torques \( \tau \).
- Essential for model-based control of robotic systems.

# 4. Model-Based Manipulator Control

**Objective:**  
Use the dynamic model of the robot to compute control torques that ensure the robot follows a desired trajectory.

## 4.1 Control Law Formulation

### Partitioned Control Law
The dynamic equation may be partitioned as:
\[
\tau = \alpha(\theta)\,\ddot{\theta} + \beta(\theta,\dot{\theta})
\]
where:
- \( \alpha(\theta) \) is often the inertia matrix \( M(\theta) \).
- \( \beta(\theta,\dot{\theta}) \) includes Coriolis, centrifugal, gravity, and friction terms.

### Servo Control Law
A typical servo control law might be:
\[
\tau_d = \tau_{\text{ff}} + K_V\,\dot{e} + K_P\,e
\]
where:
- \( e = \theta_d - \theta \) is the tracking error.
- \( \dot{e} \) is the error in velocity.
- \( K_P \) and \( K_V \) are diagonal gain matrices.
- \( \tau_{\text{ff}} \) is the feedforward term derived from the dynamic model.

### Error Dynamics
Substituting the control law back into the dynamic model results in a second-order error differential equation:
\[
\ddot{e} + K_V\,\dot{e} + K_P\,e = 0
\]
With proper choice of gains, the error converges to zero as time tends to infinity, ensuring the robot reaches the desired trajectory.

## 4.2 Jacobian Matrix in Dynamics

### Relationship between End-Effector and Joint Velocities
\[
\dot{x} = J(q)\,\dot{q}
\]
where:
- \( \dot{x} \) is the vector combining linear and angular velocities of the end-effector.
- \( J(q) \) is the Jacobian matrix.

### Structure of the Jacobian
- **Position Jacobian:** Relates joint velocities to the end-effector’s linear velocity.
- **Orientation Jacobian:** Relates joint velocities to the angular velocity.
- For an \( n \)-DOF robot in Cartesian space, the Jacobian has a size of \( 6 \times n \).

### Example (SCARA Manipulator)
For a SCARA robot with 4 DOF, the Jacobian matrix would be \( 6 \times 4 \). Joint-specific columns are computed based on whether the joint is:
- **Revolute:** (using cross products like \( z_{i-1} \times (p - p_{i-1}) \))
- **Prismatic:** (directly using the joint’s axis vector)
