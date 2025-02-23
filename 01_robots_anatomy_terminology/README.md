# Robot Anatomy and Terminology

## 1. Robot Anatomy and Terminology

### Basic Components
A robotic system generally consists of:
- **Joints:** The moving connections between parts.
- **Links:** The rigid or flexible elements connecting joints.
- **Actuators:** Devices that cause motion (motors, smart actuators).
- **Sensors:** Components that provide feedback (position, force, etc.).

### Examples of Robotic Systems
- **Humanoid Robots:** Have straight (serial) rigid links and primarily electric (revolute) actuators.
- **Exoskeletons:** Often used for rehabilitation, composed of serial links and similar actuators.
- **Medical Robots:** May include special joints (e.g., prismatic) to facilitate precise movements in minimally invasive surgeries.

## 2. Types of Joints and Degrees of Freedom (DOF)

### Revolute Joint
- **Movement:** Rotational only.
- **DOF:** 1 (variable: Θ).

### Prismatic Joint
- **Movement:** Linear (translational) displacement.
- **DOF:** 1 (variable: D).

### Cylindrical Joint
- **Combination:** Includes one revolute and one prismatic motion.
- **DOF:** 2 (rotational and translational).

### Spherical Joint
- **Movement:** Provides rotation about three perpendicular axes (roll, pitch, yaw).
- **DOF:** 3.

### Articulated Robot
- **Characteristic:** All joints are revolute.
- **Example:** A robot with three revolute joints has configuration (R, R, R) giving 3 DOF.

### SCARA Robot
- **Configuration:** Typically includes two revolute joints, one prismatic joint, and an additional revolute joint for gripper orientation.
- **DOF:** 4 (Θ₁, Θ₂, D₃, Θ₄).

## 3. Work Volume (Workspace)

### Definition
The work volume is the three-dimensional region within which the robot’s end-effector (tool or gripper) can operate or position itself.

### Determining Work Volume
- **For a 2 DOF Robot:**
  - *Example:* With joint 1 (Θ₁ from 0° to 90°) and joint 2 (Θ₂ from 0° to 180°), the combined variations define a specific region in space.
- **For a 3 DOF Planar Robot:**
  - The workspace is obtained by varying Θ₁ (e.g., from 0° to 90°) and both Θ₂ and Θ₃ (e.g., from 0° to 180°).
  - Typically plotted using nested loops in Matlab to show the envelope of reachable positions.

### Work Volume for Different Robot Types
- **Cartesian Robots:**
  - Consist solely of prismatic joints.
  - Both side and top views yield a square area because movements are linear along each axis.
- **Cylindrical Robots:**
  - Combine revolute and prismatic joints.
  - Side view shows a square area (due to linear motion), while the top view is circular (due to the rotary component).

## 4. Robot Configurations in Terms of Independent Coordinates

- **Cartesian Robot:**
  - Uses only prismatic joints.
  - Configuration: Typically represented as (P₁, P₂, P₃).
- **Cylindrical Robot:**
  - Mix of revolute and prismatic joints.
  - *Example configuration:* (R, P, P) where the first joint is rotational (Θ) and the next two provide linear displacement.
- **Spherical and Articulated Robots:**
  - **Spherical:** May have configurations like (R, R, P) based on the specific joint arrangement.
  - **Articulated:** All joints are revolute, e.g., (R, R, R).
- **SCARA Robot Configuration:**
  - Typically a combination such as (Θ₁, Θ₂, D₃, Θ₄).

## 5. Actuators

### 5.1 Electric Actuators

#### DC Motors
- **Working Principle:**
  - A current-carrying conductor placed perpendicular to a magnetic field produces a turning force (torque).
  - Fleming’s Left-Hand Rule helps determine the direction of this force:
    - **Middle finger:** current direction
    - **Index finger:** magnetic field direction
    - **Thumb:** direction of force.
- **Control:**
  - Speed is controlled (examples given with RPM variations: 100, 50, 30, 80, 150 RPM).
  - Components include a power supply, driving circuitry (like an L293 H-bridge), and an interface (Arduino, NI boards).

### 5.2 Stepper Motors

- **Characteristics:**
  - Consist of a stator (fixed part) and a rotor (rotating part).
  - The rotor moves in discrete steps.
  - **Minimum Step Angle:** Approximately 1.8° per step, though increasing the number of stator windings can provide finer resolution.
- **Application:**
  - Used in systems where precise, incremental rotation is required (e.g., needling systems).

### 5.3 DC Servo Motors

- **Features:**
  - Include an inbuilt potentiometer for feedback to control the shaft angle precisely.
  - Often combined with gear trains to increase torque.
- **Application:**
  - Employed in systems where accurate angular positioning is essential, such as in a needling system for tissue penetration.

### 5.4 Smart Actuators

#### Shape Memory Alloy (SMA) Actuators
- **Composition:**
  - Typically made from nickel-titanium alloy (Nitinol).
  - Available in two forms: wire and spring.
- **Working Principle:**
  - Undergo a phase transformation when heated:
    - **High-temperature phase (austenite):** The material “remembers” its original shape.
    - **Low-temperature phase (martensite):** The material deforms.
  - Upon heating (e.g., by passing current, causing Joule heating) the actuator returns to its pre-set shape, causing contraction or bending.
- **Application Example:**
  - A needling system where the SMA wire actuator is attached to the lateral surface of the needle to induce bending.

#### Electrostatic Microactuators
- **Working Principle:**
  - Two parallel plates (electrodes) are placed close together. When a voltage is applied, opposite charges are induced, generating an attractive electrostatic force.
  - The force can be computed with an expression involving the dielectric constant, electrode area, applied voltage, and distance between the plates.
- **Application:**
  - Commonly used in micro-robotic systems.

## 6. Sensors

### 6.1 Optical Encoders

#### Absolute Optical Encoders
- **Structure:**
  - A disk with multiple concentric tracks.
  - Each track contains coded regions (black = opaque, transparent regions).
- **Working Principle:**
  - As the disk rotates, a light source and photo detectors read the pattern.
  - Each track encodes a bit (e.g., outer ring might represent 2⁰, the next 2¹, etc.).
  - The combined binary value from all tracks gives the exact angular position.
- **Angular Resolution:**
  - Calculated as:  
    `360° ÷ (2^n)`  
    where *n* is the number of tracks.  
    *Example:* With n = 13, the resolution is approximately 0.044°.

#### Incremental Optical Encoders
- **Structure:**
  - A disk with *n* radial lines.
- **Working Principle:**
  - The encoder generates pulses as the disk rotates.
  - The resolution is given by:  
    `360° ÷ n`
  - Two channels (A and B) are used to detect the direction of rotation:
    - If Channel A leads B, the rotation is in one direction; if it lags, the rotation is in the opposite direction.

### 6.2 Force Sensors

#### Four Sensor (Strain Gauge Based)
- **Working Principle:**
  - Strain gauges change resistance when deformed.
  - The resistance is given by:  
    `R = ρL/A`  
    where *ρ* is the resistivity, *L* is the length, and *A* is the cross-sectional area.
- **Signal Processing:**
  - A Wheatstone bridge is used to convert changes in resistance into a measurable voltage.
  - In a balanced bridge (no applied force), the output voltage is zero; any imbalance indicates an applied force.
- **Example:**
  - FSR402 is a simple force sensor (costing around 200 rupees) used in applications like exoskeletons for grasping objects.
