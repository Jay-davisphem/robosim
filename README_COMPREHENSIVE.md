# RoboSim: 3D Robot Arm Simulator

A comprehensive 3D robotics simulation system built with C++ and OpenGL that enables real-time visualization, control, and analysis of robotic manipulators using Denavit-Hartenberg parameters.

![Robot Simulation Demo](doc/demo.gif)

---

## 📋 Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [Technical Architecture](#technical-architecture)
- [Robotics Principles](#robotics-principles)
- [Graphics & Rendering](#graphics--rendering)
- [Installation & Setup](#installation--setup)
- [Running the Application](#running-the-application)
- [User Interface Guide](#user-interface-guide)
- [Configuration & Customization](#configuration--customization)
- [Understanding the Code](#understanding-the-code)
- [Mathematical Foundations](#mathematical-foundations)
- [Troubleshooting](#troubleshooting)
- [Future Enhancements](#future-enhancements)

---

## 🎯 Overview

**RoboSim** is an educational and research-oriented robotics simulation platform that provides:

- **Real-time 3D visualization** of robot manipulators
- **Interactive control** of joint angles and robot parameters
- **Forward kinematics** computation showing end-effector positions
- **Inverse kinematics** solving for target reaching
- **Jacobian matrix** calculation for velocity analysis
- **Live parameter editing** for robot design exploration

### Who Is This For?

- **Students** learning robotics and computer graphics
- **Researchers** prototyping robotic algorithms
- **Engineers** validating robot designs before physical implementation
- **Educators** teaching robotics concepts with visual demonstrations

---

## ✨ Key Features

### 🤖 Robotics Features
- **6-DOF Robot Arm Simulation**: Supports any robot configuration via Denavit-Hartenberg parameters
- **Real-time Forward Kinematics**: Instant calculation of end-effector position and orientation
- **Numerical Inverse Kinematics**: Click-to-reach functionality using Jacobian-based optimization
- **Jacobian Matrix Display**: Real-time calculation and visualization of the manipulator Jacobian
- **Joint Control**: Individual control of each joint with configurable limits
- **Parameter Editing**: Live modification of D-H parameters with instant visual feedback

### 🎨 Graphics Features
- **Modern OpenGL Rendering**: Hardware-accelerated 3D graphics with Phong shading
- **Procedural Mesh Generation**: Robot links automatically sized based on D-H parameters
- **Interactive 3D Camera**: Orbit, pan, and zoom controls for optimal viewing
- **Immediate Mode GUI**: User-friendly controls using Dear ImGui
- **Real-time Performance**: Smooth 60fps rendering with complex 3D scenes

### 🔧 Technical Features
- **Cross-platform**: Runs on Linux, Windows, and macOS
- **Modern C++17**: Clean, maintainable codebase with RAII and smart pointers
- **Modular Architecture**: Separated concerns for rendering, robotics, and user interface
- **CMake Build System**: Easy compilation and dependency management

---

## 🏗️ Technical Architecture

### System Components

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Application   │    │    Robot        │    │   Rendering     │
│   - Main Loop   │◄──►│   - Kinematics  │◄──►│   - OpenGL      │
│   - Event Loop  │    │   - Jacobian    │    │   - Shaders     │
│   - GUI         │    │   - IK Solver   │    │   - Camera      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│     ImGui       │    │   DH Params     │    │     Meshes      │
│   - Controls    │    │   - Link Data   │    │   - Geometry    │
│   - Sliders     │    │   - Joint Info  │    │   - Materials   │
│   - Displays    │    │   - File I/O    │    │   - Lighting    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### Libraries and Dependencies

| Library | Purpose | Version |
|---------|---------|---------|
| **OpenGL** | 3D graphics rendering | 3.3+ |
| **GLFW** | Window management and input | 3.3+ |
| **GLEW** | OpenGL extension loading | 2.2+ |
| **Dear ImGui** | Immediate mode GUI | Latest |
| **ImGuizmo** | 3D transformation gizmos | Latest |
| **Eigen3** | Linear algebra operations | 3.3+ |

---

## 🤖 Robotics Principles

### Denavit-Hartenberg Parameters

The robot's geometry is defined using the standard Denavit-Hartenberg (D-H) convention, which describes each joint and link using four parameters:

```
Joint i connects Link i-1 to Link i
```

| Parameter | Symbol | Description |
|-----------|--------|-------------|
| **Link Length** | `a_i` | Distance between joint axes (along x-axis) |
| **Link Twist** | `α_i` | Angle between joint axes (about x-axis) |
| **Link Offset** | `d_i` | Distance along previous joint axis to common normal |
| **Joint Angle** | `θ_i` | Angle about joint axis (variable for revolute joints) |

### Transformation Matrices

Each joint creates a homogeneous transformation matrix:

```
T_i = Rot_z(θ_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(α_i)
```

The complete forward kinematics chain:
```
T_0^n = T_0^1 * T_1^2 * ... * T_{n-1}^n
```

### Forward Kinematics Process

1. **Read D-H Parameters**: Load robot configuration from file
2. **Build Transformation Chain**: Create 4×4 matrices for each joint
3. **Compute End-Effector Pose**: Multiply transformation matrices
4. **Extract Position**: Get translation component from final matrix
5. **Update Visualization**: Render robot with new joint positions

### Inverse Kinematics Algorithm

The system uses a **Jacobian-based numerical method**:

1. **Define Target**: User clicks to set desired end-effector position
2. **Calculate Error**: Compute distance between current and target position
3. **Compute Jacobian**: Calculate 6×n velocity relationship matrix
4. **Solve for Joint Velocities**: Use pseudo-inverse method
5. **Update Joint Angles**: Integrate velocities to get new positions
6. **Iterate**: Repeat until convergence or timeout

```cpp
// Simplified IK iteration
Vector3f error = target - current_position;
MatrixXf jacobian = robot.getJacobian();
VectorXf joint_velocities = jacobian.pseudoInverse() * error;
joint_angles += joint_velocities * dt;
```

### Jacobian Matrix

The manipulator Jacobian (J) relates joint velocities to end-effector velocity:

```
ẋ = J(q) * q̇
```

Where:
- `ẋ` = End-effector velocity (6×1: linear + angular)
- `J(q)` = Jacobian matrix (6×n)
- `q̇` = Joint velocities (n×1)

**Applications:**
- Inverse kinematics solving
- Velocity control
- Singularity analysis
- Force/torque transformation

---

## 🎨 Graphics & Rendering

### Rendering Pipeline

1. **Geometry Generation**: Procedural creation of robot link meshes
2. **Vertex Processing**: Transform vertices using model-view-projection matrices
3. **Fragment Shading**: Apply Phong lighting model for realistic appearance
4. **Depth Testing**: Ensure proper occlusion of 3D objects
5. **GUI Overlay**: Render ImGui interface on top of 3D scene

### Mesh Generation System

The system procedurally generates 3D meshes based on robot parameters:

#### Cylinder Meshes (Robot Links)
```cpp
// Parametric cylinder generation
for (int i = 0; i <= segments; ++i) {
    float angle = 2.0f * M_PI * i / segments;
    float x = radius * cos(angle);
    float y = radius * sin(angle);
    // Generate vertices at both ends
    vertices.push_back(Vector3f(x, y, 0));      // Bottom
    vertices.push_back(Vector3f(x, y, height)); // Top
}
```

#### Box Meshes (Joint Housings)
- Procedural generation of rectangular prisms
- Automatic sizing based on joint requirements
- Optimized triangle count for performance

#### Sphere Meshes (End-Effector)
- UV-sphere generation using spherical coordinates
- Smooth normals for realistic lighting
- Configurable resolution for performance tuning

### Lighting Model

**Phong Shading Implementation:**
```glsl
// Fragment shader lighting calculation
vec3 ambient = ka * light_ambient;
vec3 diffuse = kd * max(dot(normal, light_dir), 0.0) * light_diffuse;
vec3 specular = ks * pow(max(dot(reflect_dir, view_dir), 0.0), shininess) * light_specular;
vec3 final_color = ambient + diffuse + specular;
```

### Camera System

**Orbital Camera Features:**
- Mouse-controlled rotation around robot
- Scroll wheel zoom with configurable limits
- Automatic centering on robot base
- Smooth interpolation for user comfort

---

## 🚀 Installation & Setup

### System Requirements

#### Minimum Requirements
- **OS**: Linux (Ubuntu 18.04+), Windows 10, or macOS 10.14+
- **CPU**: Intel i3 or equivalent
- **RAM**: 4GB
- **GPU**: OpenGL 3.3 compatible graphics card
- **Storage**: 500MB free space

#### Recommended Requirements
- **CPU**: Intel i5 or equivalent (for smooth real-time performance)
- **RAM**: 8GB (for large robot configurations)
- **GPU**: Dedicated graphics card with OpenGL 4.0+ support

### Prerequisites Installation

#### Ubuntu/Debian Linux
```bash
sudo apt update
sudo apt install build-essential cmake git
sudo apt install libgl1-mesa-dev libglfw3-dev libglew-dev libeigen3-dev
```

#### Fedora/CentOS/RHEL
```bash
sudo dnf install gcc-c++ cmake git
sudo dnf install mesa-libGL-devel glfw-devel glew-devel eigen3-devel
```

#### macOS (using Homebrew)
```bash
brew install cmake git glfw glew eigen
```

#### Windows (using vcpkg)
```bash
vcpkg install glfw3 glew eigen3
```

### Building from Source

#### Step 1: Clone Repository
```bash
git clone https://github.com/samilauronen/robosim.git
cd robosim
```

#### Step 2: Initialize Submodules
```bash
# Fix SSH to HTTPS for ImGuizmo (if needed)
git config --file=.gitmodules submodule.lib/ImGuizmo.url https://github.com/CedricGuillemet/ImGuizmo.git
git submodule sync
git submodule update --init --recursive
```

#### Step 3: Configure Build
```bash
mkdir build && cd build
cmake ..
```

**Common CMake Options:**
```bash
# Debug build with symbols
cmake -DCMAKE_BUILD_TYPE=Debug ..

# Release build with optimizations
cmake -DCMAKE_BUILD_TYPE=Release ..

# Specify custom install prefix
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
```

#### Step 4: Compile
```bash
# Build using all available CPU cores
make -j$(nproc)

# Or build with specific number of threads
make -j4
```

#### Step 5: Verify Installation
```bash
# Check if executable was created
ls -la ../robosim

# Test run (should open window)
cd ..
./robosim
```

### Troubleshooting Build Issues

#### Missing Dependencies
```bash
# Check what packages are missing
pkg-config --list-all | grep -E "(glfw|glew|eigen|opengl)"

# Install missing packages
sudo apt install [missing-package-dev]
```

#### CMake Configuration Errors
```bash
# Clear cache and reconfigure
rm -rf build/
mkdir build && cd build
cmake .. -DCMAKE_VERBOSE_MAKEFILE=ON
```

#### Submodule Issues
```bash
# Reset submodules
git submodule deinit --all
git submodule update --init --recursive
```

---

## 🎮 Running the Application

### Basic Execution
```bash
cd /path/to/robosim
./robosim
```

### Command Line Options
```bash
# Specify custom robot parameters file
./robosim --params custom_robot.txt

# Set initial window size
./robosim --width 1920 --height 1080

# Enable verbose logging
./robosim --verbose

# Show help
./robosim --help
```

### What You'll See

Upon launching, the application creates a window with:

1. **3D Viewport** (main area)
   - 3D robot visualization
   - Interactive camera controls
   - Target position marker (yellow sphere)

2. **Control Panel** (left side)
   - Joint angle sliders
   - Robot parameter inputs
   - Inverse kinematics controls

3. **Information Panel** (right side)
   - Current joint angles
   - End-effector position
   - Jacobian matrix display
   - Performance metrics

### Basic Controls

#### Mouse Controls
- **Left Click + Drag**: Rotate camera around robot
- **Right Click**: Set IK target position
- **Scroll Wheel**: Zoom in/out
- **Middle Click + Drag**: Pan camera

#### Keyboard Shortcuts
- **R**: Reset camera to default position
- **Space**: Reset robot to zero position
- **Esc**: Exit application
- **F1**: Toggle help overlay
- **F11**: Toggle fullscreen mode

---

## 🎛️ User Interface Guide

### Joint Control Panel

#### Joint Angle Sliders
Each robot joint has an individual control slider:

```
Joint 1 (Base):     [-180°] ====●==== [+180°]
Joint 2 (Shoulder): [-90°]  ==●====== [+90°]
Joint 3 (Elbow):    [-135°] ======●== [+135°]
...
```

**Features:**
- Real-time updates during dragging
- Configurable joint limits
- Angle display in degrees
- Reset to zero button for each joint

#### Preset Positions
Quick access to common robot configurations:
- **Home Position**: All joints at zero
- **Extended**: Robot fully extended
- **Folded**: Compact configuration
- **Custom**: User-defined positions

### Robot Parameters Panel

#### Denavit-Hartenberg Parameter Editor
Edit D-H parameters in real-time:

| Joint | a (mm) | α (deg) | d (mm) | θ (deg) |
|-------|--------|---------|--------|---------|
| 1 | [0.000] | [90.0] | [1000] | Variable |
| 2 | [700.0] | [0.0] | [0.000] | Variable |
| 3 | [500.0] | [-90.0] | [300] | Variable |
| ... | ... | ... | ... | ... |

**Real-time Updates:**
- Changes immediately affect 3D visualization
- Parameter validation prevents invalid configurations
- Undo/Redo functionality for parameter changes

### Information Display Panel

#### Current Status
```
End-Effector Position:
  X: 1.234 m
  Y: 0.567 m  
  Z: 0.890 m

Joint Angles:
  Joint 1: 45.2°
  Joint 2: -30.1°
  Joint 3: 60.8°
  ...

TCP Velocity:
  Linear:  [0.12, 0.05, 0.08] m/s
  Angular: [0.0, 0.0, 0.15] rad/s
```

#### Jacobian Matrix Display
Real-time 6×n Jacobian matrix visualization:
```
Manipulator Jacobian:
[ 0.123  0.456  0.789  0.012  0.345  0.678 ]
[ 0.234  0.567  0.890  0.123  0.456  0.789 ]
[ 0.345  0.678  0.901  0.234  0.567  0.890 ]
[ 0.456  0.789  0.012  0.345  0.678  0.901 ]
[ 0.567  0.890  0.123  0.456  0.789  0.012 ]
[ 0.678  0.901  0.234  0.567  0.890  0.123 ]
```

#### Performance Metrics
```
Rendering:
  FPS: 60.0
  Frame Time: 16.7ms
  
Computation:
  FK Time: 0.05ms
  IK Time: 2.3ms
  Jacobian: 0.8ms
```

### Inverse Kinematics Panel

#### Target Control
- **Manual Input**: Specify target coordinates directly
- **Mouse Picking**: Right-click in 3D viewport to set target
- **Preset Targets**: Common positions for testing

#### IK Solver Settings
```
Algorithm: Jacobian Transpose
Max Iterations: [100]
Convergence Threshold: [0.001] mm
Step Size: [0.1]
Timeout: [1000] ms

[✓] Show IK Path
[✓] Animate Solution
[ ] Avoid Joint Limits
```

#### Solution Status
```
IK Status: ✓ CONVERGED
Iterations: 23
Final Error: 0.0008 mm
Solve Time: 15.2 ms
```

---

## ⚙️ Configuration & Customization

### Robot Configuration Files

Robot parameters are defined in text files using D-H convention:

#### File Format (`params.txt`)
```
# Comments start with #
# Format: JointType a d alpha theta
# JointType: R (Revolute) or P (Prismatic)
# Angles in radians, distances in meters

# Joint  a     d    alpha    theta
  R      0     1    pi/2     x      # Base rotation
  R      0.7   0    0        x      # Shoulder  
  R      0.5   0.3  -pi/2    x      # Elbow
  R      0     0.3  -pi/2    x      # Wrist 1
  R      0.3   0    pi/2     x      # Wrist 2
  R      0.2   0    0        x      # Wrist 3
```

#### Parameter Descriptions
- **JointType**: 
  - `R` = Revolute (rotational) joint
  - `P` = Prismatic (linear) joint
- **a**: Link length (distance between joint axes)
- **d**: Link offset (distance along previous joint axis)
- **alpha**: Link twist (angle between joint axes)
- **theta**: Joint angle (marked as `x` for variable joints)

#### Mathematical Expressions
The parser supports mathematical expressions for angles:
```
pi/2     # 90 degrees
-pi/2    # -90 degrees  
pi/4     # 45 degrees
2*pi/3   # 120 degrees
```

### Custom Robot Examples

#### PUMA 560 Configuration
```
# PUMA 560-style robot arm
R  0      0     pi/2   x
R  0.43   0     0      x  
R  0.02   0.15  pi/2   x
R  0      0.43  -pi/2  x
R  0      0     pi/2   x
R  0      0     0      x
```

#### SCARA Robot Configuration
```
# SCARA (Selective Compliance Assembly Robot Arm)
R  0.35   0     0      x    # Link 1
R  0.25   0     0      x    # Link 2  
P  0      0     0      x    # Vertical slide
R  0      0     0      x    # End-effector rotation
```

#### Anthropomorphic Arm
```
# Human-like arm proportions
R  0      0.3   pi/2   x    # Shoulder yaw
R  0      0     pi/2   x    # Shoulder pitch  
R  0.3    0     0      x    # Upper arm
R  0.25   0     0      x    # Forearm
R  0      0     pi/2   x    # Wrist pitch
R  0      0.1   0      x    # Hand
```

### Rendering Configuration

#### Shader Customization
Modify lighting and appearance by editing shader files:

**Vertex Shader** (`src/rendering/shaders/simple_shader.vert`):
```glsl
#version 330 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec3 world_pos;
out vec3 world_normal;

void main() {
    world_pos = (model * vec4(position, 1.0)).xyz;
    world_normal = mat3(model) * normal;
    gl_Position = projection * view * vec4(world_pos, 1.0);
}
```

**Fragment Shader** (`src/rendering/shaders/simple_shader.frag`):
```glsl
#version 330 core

in vec3 world_pos;
in vec3 world_normal;

uniform vec3 light_pos;
uniform vec3 view_pos;
uniform vec3 light_color;
uniform vec3 object_color;

out vec4 frag_color;

void main() {
    // Phong lighting calculation
    vec3 ambient = 0.1 * light_color;
    
    vec3 norm = normalize(world_normal);
    vec3 light_dir = normalize(light_pos - world_pos);
    float diff = max(dot(norm, light_dir), 0.0);
    vec3 diffuse = diff * light_color;
    
    vec3 view_dir = normalize(view_pos - world_pos);
    vec3 reflect_dir = reflect(-light_dir, norm);
    float spec = pow(max(dot(view_dir, reflect_dir), 0.0), 32);
    vec3 specular = spec * light_color;
    
    vec3 result = (ambient + diffuse + specular) * object_color;
    frag_color = vec4(result, 1.0);
}
```

#### Material Properties
Customize robot appearance by modifying material constants:

```cpp
// In Application.cpp or separate materials file
struct Material {
    Vector3f ambient   = Vector3f(0.2f, 0.2f, 0.2f);   // Base color
    Vector3f diffuse   = Vector3f(0.8f, 0.6f, 0.4f);   // Surface color  
    Vector3f specular  = Vector3f(1.0f, 1.0f, 1.0f);   // Reflection color
    float shininess    = 32.0f;                          // Shininess factor
};

// Predefined materials
Material chrome = {
    Vector3f(0.25f, 0.25f, 0.25f),
    Vector3f(0.4f, 0.4f, 0.4f),  
    Vector3f(0.774597f, 0.774597f, 0.774597f),
    76.8f
};

Material gold = {
    Vector3f(0.24725f, 0.1995f, 0.0745f),
    Vector3f(0.75164f, 0.60648f, 0.22648f),
    Vector3f(0.628281f, 0.555802f, 0.366065f),
    51.2f
};
```

---

## 🧠 Understanding the Code

### Project Structure
```
robosim/
├── src/
│   ├── core/                    # Core robotics algorithms
│   │   ├── Application.cpp/hpp  # Main application loop
│   │   ├── Robot.cpp/hpp        # Robot kinematics & control
│   │   ├── DhParam.cpp/hpp      # D-H parameter handling
│   │   ├── InverseKinematics.*  # IK solving algorithms
│   │   ├── JointedLink.cpp/hpp  # Individual joint/link
│   │   └── PidController.*      # Control algorithms
│   ├── meshes/                  # 3D geometry generation
│   │   ├── Mesh.cpp/hpp         # Base mesh class
│   │   ├── BoxMesh.*            # Box/cube primitives
│   │   ├── CylinderMesh.*       # Cylindrical links
│   │   ├── SphereMesh.*         # Spherical primitives
│   │   └── JointedLinkMesh.*    # Complete link geometry
│   ├── rendering/               # Graphics & visualization
│   │   ├── Camera.cpp/hpp       # 3D camera controls
│   │   ├── Shader.hpp           # OpenGL shader management
│   │   └── shaders/             # GLSL shader files
│   ├── resources/               # Configuration files
│   │   └── params.txt           # Default robot parameters
│   └── Main.cpp                 # Program entry point
├── lib/                         # External dependencies
│   ├── Eigen/                   # Linear algebra library
│   ├── GLFW/                    # Window management
│   ├── GLEW/                    # OpenGL extensions
│   ├── imgui/                   # GUI library
│   └── ImGuizmo/                # 3D manipulation tools
├── build/                       # CMake build directory
├── doc/                         # Documentation & demos
└── CMakeLists.txt              # Build configuration
```

### Key Classes and Their Responsibilities

#### `Application` Class
**Purpose**: Main application controller and event loop manager

**Key Methods**:
```cpp
class Application {
public:
    void createWindow(int width, int height);  // Initialize graphics
    void run();                                // Main event loop
    
private:
    void update(float dt);                     // Update simulation
    void render();                             // Render 3D scene
    void renderGui();                          // Render GUI overlay
    void handleInput();                        // Process user input
    
    std::unique_ptr<Robot> robot_;             // Robot instance
    Camera camera_;                            // 3D camera
    std::vector<float> joint_angle_controls_;  // GUI sliders
};
```

**Responsibilities**:
- Window and OpenGL context management
- Main render loop coordination  
- User input processing
- GUI interface management
- Frame timing and performance monitoring

#### `Robot` Class
**Purpose**: Complete robot kinematics and control system

**Key Methods**:
```cpp
class Robot {
public:
    Robot(std::string dh_param_file, Vector3f location);
    
    // Kinematics
    Vector3f getTcpWorldPosition() const;
    MatrixXf getJacobian(const VectorXf& joint_angles) const;
    void setJointAngles(const VectorXf& angles);
    
    // Inverse Kinematics
    void setIkTarget(const Vector3f& target);
    bool solveInverseKinematics();
    
    // Visualization
    void render(const Matrix4f& view_matrix) const;
    
private:
    std::vector<std::unique_ptr<JointedLink>> links_;
    Affine3f worldToBase_;                    // Base transform
    Vector3f ik_target_;                      // IK target position
};
```

**Responsibilities**:
- Forward kinematics computation
- Jacobian matrix calculation
- Inverse kinematics solving
- Joint angle management
- 3D rendering coordination

#### `JointedLink` Class  
**Purpose**: Individual robot joint and link representation

**Key Methods**:
```cpp
class JointedLink {
public:
    JointedLink(const DhParam& params, int index);
    
    // Kinematics
    Affine3f getTransformationMatrix(float joint_angle) const;
    Vector3f getJointAxis() const;
    Vector3f getJointPosition(const Affine3f& base_transform) const;
    
    // Visualization  
    void render(const Affine3f& transform) const;
    
private:
    DhParam dh_params_;                       // D-H parameters
    std::unique_ptr<Mesh> link_mesh_;         // 3D geometry
    float current_angle_;                     // Current joint angle
    float min_angle_, max_angle_;             // Joint limits
};
```

**Responsibilities**:
- Individual joint transformation calculation
- Link geometry management
- Joint limit enforcement
- Single link rendering

#### `InverseKinematics` Namespace
**Purpose**: Numerical IK solving algorithms

**Key Functions**:
```cpp
namespace IK {
    struct IKSolution {
        VectorXf joint_angles;
        bool converged;
        int iterations;
        float final_error;
    };
    
    class SimpleIKSolver {
    public:
        static IKSolution solve(const Robot& robot, 
                              const Vector3f& target);
    private:
        static constexpr float DISTANCE_THRESHOLD = 0.001f;
        static constexpr uint64_t TIMEOUT_MICROS = 1000000;
    };
}
```

**Algorithm Overview**:
1. Calculate position error between current and target
2. Compute Jacobian matrix at current configuration
3. Use pseudo-inverse to find joint velocity direction
4. Update joint angles using small step size
5. Repeat until convergence or timeout

### Data Flow Architecture

```
User Input → GUI Controls → Robot State → Kinematics → Rendering
    ↓              ↓           ↓            ↓           ↓
Mouse/Keyboard → Sliders → Joint Angles → FK/IK → 3D Scene
    ↓              ↓           ↓            ↓           ↓
Events → ImGui → Robot Class → Math → OpenGL
```

**Detailed Flow**:
1. **Input Processing**: GLFW captures mouse/keyboard events
2. **GUI Updates**: ImGui processes interface interactions  
3. **State Changes**: Robot joint angles and parameters update
4. **Kinematics Computation**: Forward/inverse kinematics calculations
5. **Mesh Updates**: 3D geometry positioned based on joint states
6. **Rendering**: OpenGL draws updated scene to framebuffer
7. **Display**: Final image presented to user

---

## 📐 Mathematical Foundations

### Homogeneous Transformations

#### 4×4 Transformation Matrix Structure
```
T = [R | t]  = [r11 r12 r13 tx]
    [0 | 1]    [r21 r22 r23 ty]
               [r31 r32 r33 tz]
               [ 0   0   0   1]
```

Where:
- **R** (3×3): Rotation matrix
- **t** (3×1): Translation vector  
- **Bottom row**: Homogeneous coordinates

#### Denavit-Hartenberg Transformation
For joint i with parameters (a_i, α_i, d_i, θ_i):

```
T_i = Rot_z(θ_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(α_i)

    = [cos(θ_i) -sin(θ_i)cos(α_i)  sin(θ_i)sin(α_i)  a_i*cos(θ_i)]
      [sin(θ_i)  cos(θ_i)cos(α_i) -cos(θ_i)sin(α_i)  a_i*sin(θ_i)]
      [   0         sin(α_i)          cos(α_i)           d_i       ]
      [   0            0                 0               1         ]
```

### Forward Kinematics Mathematics

#### Complete Transformation Chain
```cpp
// Compute end-effector pose
Affine3f total_transform = worldToBase_;
for (int i = 0; i < num_joints; ++i) {
    Affine3f joint_transform = links_[i]->getTransformationMatrix(joint_angles[i]);
    total_transform = total_transform * joint_transform;
}
Vector3f tcp_position = total_transform.translation();
```

#### Matrix Multiplication Chain
```
T_0^n = T_0^1 * T_1^2 * T_2^3 * ... * T_{n-1}^n

Where each T_i^{i+1} is the D-H transformation for joint i+1
```

### Jacobian Matrix Computation

#### Geometric Jacobian Calculation
For each joint i contributing to end-effector motion:

**Linear Velocity Component**:
```cpp
Vector3f joint_position = getJointPosition(i);
Vector3f tcp_position = getTcpPosition();
Vector3f joint_axis = getJointAxis(i);

Vector3f linear_component = joint_axis.cross(tcp_position - joint_position);
jacobian.block(0, i, 3, 1) = linear_component;
```

**Angular Velocity Component**:
```cpp
Vector3f angular_component = joint_axis;
jacobian.block(3, i, 3, 1) = angular_component;
```

#### Complete Jacobian Structure
```
J = [J_v]  = [∂p/∂q1  ∂p/∂q2  ...  ∂p/∂qn]
    [J_ω]    [∂ω/∂q1  ∂ω/∂q2  ...  ∂ω/∂qn]

Where:
- J_v (3×n): Linear velocity Jacobian
- J_ω (3×n): Angular velocity Jacobian  
- p: End-effector position
- ω: End-effector angular velocity
- q: Joint angles vector
```

### Inverse Kinematics Mathematics

#### Jacobian Pseudo-Inverse Method
```cpp
// IK iteration step
Vector3f position_error = target - current_position;
Vector<float, 6> error_6d;
error_6d << position_error, Vector3f::Zero();  // Only position, no orientation

MatrixXf jacobian = robot.getJacobian();
MatrixXf jacobian_pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();

VectorXf joint_velocities = jacobian_pinv * error_6d;
VectorXf new_joint_angles = current_joint_angles + step_size * joint_velocities;
```

#### Convergence Criteria
```cpp
bool converged = (position_error.norm() < DISTANCE_THRESHOLD) ||
                (iterations >= MAX_ITERATIONS) ||
                (elapsed_time > TIMEOUT_MICROSECONDS);
```

#### Damped Least Squares (Alternative Method)
For better numerical stability near singularities:
```cpp
MatrixXf damped_jacobian = jacobian.transpose() * 
    (jacobian * jacobian.transpose() + lambda * MatrixXf::Identity(6, 6)).inverse();
VectorXf joint_velocities = damped_jacobian * error_6d;
```

### Singularity Analysis

#### Detecting Singularities
```cpp
float manipulability = sqrt((jacobian * jacobian.transpose()).determinant());
bool near_singularity = (manipulability < SINGULARITY_THRESHOLD);
```

#### Types of Singularities
1. **Boundary Singularities**: Robot fully extended or retracted
2. **Interior Singularities**: Special configurations within workspace
3. **Wrist Singularities**: Wrist axes align (common in 6-DOF arms)

---

## 🔧 Troubleshooting

### Common Build Issues

#### Missing Dependencies
**Problem**: CMake can't find required libraries
```
CMake Error: Could not find a package configuration file provided by "glfw3"
```

**Solution**:
```bash
# Ubuntu/Debian
sudo apt install libglfw3-dev libglew-dev libeigen3-dev

# Fedora/CentOS  
sudo dnf install glfw-devel glew-devel eigen3-devel

# macOS
brew install glfw glew eigen
```

#### Submodule Issues
**Problem**: ImGui or ImGuizmo directories are empty
```
fatal: clone of 'git@github.com:...' failed
```

**Solution**:
```bash
# Convert SSH to HTTPS
git config --file=.gitmodules submodule.lib/ImGuizmo.url https://github.com/CedricGuillemet/ImGuizmo.git
git submodule sync
git submodule update --init --recursive
```

#### CMake Version Issues
**Problem**: CMake version too old
```
CMake Error: CMake 3.26 or higher is required
```

**Solution**:
```bash
# Ubuntu: Install newer CMake from Kitware repository
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc | sudo apt-key add -
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ focal main'
sudo apt update && sudo apt install cmake
```

#### Compiler Issues
**Problem**: C++17 features not supported
```
error: 'auto' return type is a C++14 extension
```

**Solution**:
```bash
# Update GCC/Clang
sudo apt install gcc-9 g++-9  # or later version
export CC=gcc-9
export CXX=g++-9
cmake ..
```

### Runtime Issues

#### OpenGL Context Errors
**Problem**: Application crashes on startup
```
Failed to create OpenGL context
```

**Solutions**:
1. **Update Graphics Drivers**:
   ```bash
   # Ubuntu with proprietary drivers
   sudo ubuntu-drivers autoinstall
   
   # Intel graphics
   sudo apt install intel-media-va-driver-non-free
   ```

2. **Check OpenGL Support**:
   ```bash
   glxinfo | grep "OpenGL version"
   # Should show 3.3 or higher
   ```

3. **Force Software Rendering** (last resort):
   ```bash
   LIBGL_ALWAYS_SOFTWARE=1 ./robosim
   ```

#### Performance Issues

**Problem**: Low framerate or stuttering

**Solutions**:
1. **Reduce Mesh Resolution**:
   ```cpp
   // In mesh generation code, reduce segment counts
   const int CYLINDER_SEGMENTS = 12;  // Instead of 32
   const int SPHERE_SEGMENTS = 16;    // Instead of 32
   ```

2. **Disable VSync**:
   ```cpp
   // In Application.cpp
   glfwSwapInterval(0);  // Disable VSync
   ```

3. **Profile Performance**:
   ```bash
   # Use valgrind for memory profiling
   valgrind --tool=callgrind ./robosim
   
   # Use perf for CPU profiling
   perf record ./robosim
   perf report
   ```

#### Robot Configuration Issues

**Problem**: Robot appears distorted or joints don't move correctly

**Solutions**:
1. **Validate D-H Parameters**:
   ```bash
   # Check params.txt format
   # Ensure angles are in radians
   # Verify joint types (R for revolute, P for prismatic)
   ```

2. **Debug Joint Transformations**:
   ```cpp
   // Add debug output in Robot::updateForwardKinematics()
   std::cout << "Joint " << i << " transform:\n" 
             << transform.matrix() << std::endl;
   ```

3. **Reset to Known Working Configuration**:
   ```bash
   # Restore original params.txt
   git checkout src/resources/params.txt
   ```

#### GUI Issues

**Problem**: ImGui interface not responding or rendering incorrectly

**Solutions**:
1. **Reset ImGui Configuration**:
   ```bash
   rm imgui.ini  # Remove saved layout
   ./robosim     # Will regenerate default layout
   ```

2. **Check ImGui Version Compatibility**:
   ```bash
   # Update submodule to latest
   cd lib/imgui/imgui
   git pull origin master
   cd ../../..
   make clean && make
   ```

### Debugging Techniques

#### Enable Debug Output
```cpp
// In Robot.cpp, add debug prints
#define DEBUG_KINEMATICS
#ifdef DEBUG_KINEMATICS
    std::cout << "TCP Position: " << tcp_position.transpose() << std::endl;
    std::cout << "Jacobian determinant: " << jacobian.determinant() << std::endl;
#endif
```

#### Visual Debugging
```cpp
// Render coordinate frames for each joint
void Robot::renderDebugFrames() {
    for (int i = 0; i < links_.size(); ++i) {
        Affine3f joint_transform = getJointTransform(i);
        renderCoordinateFrame(joint_transform, 0.1f);  // 10cm axes
    }
}
```

#### Mathematical Validation
```cpp
// Verify Jacobian calculation using numerical differentiation
MatrixXf numericalJacobian(const VectorXf& joint_angles, float epsilon = 1e-6) {
    MatrixXf jacobian(6, joint_angles.size());
    Vector3f original_pos = getTcpPosition(joint_angles);
    
    for (int i = 0; i < joint_angles.size(); ++i) {
        VectorXf perturbed = joint_angles;
        perturbed[i] += epsilon;
        Vector3f new_pos = getTcpPosition(perturbed);
        Vector3f diff = (new_pos - original_pos) / epsilon;
        jacobian.block(0, i, 3, 1) = diff;
    }
    return jacobian;
}
```

---

## 🚀 Future Enhancements

### Short-term Improvements

#### User Interface Enhancements
- **Dark/Light Theme Toggle**: Modern UI appearance options
- **Customizable Layouts**: Dockable panels and saved workspace configurations
- **Animation Timeline**: Record and playback robot motions
- **Parameter Presets**: Library of common robot configurations

#### Visualization Improvements  
- **Multi-viewport Support**: Simultaneous front/side/top views
- **Trajectory Visualization**: Show end-effector path over time
- **Joint Limit Indicators**: Visual warnings for approaching limits
- **Collision Detection**: Real-time interference checking
- **Workspace Visualization**: Show reachable volume boundaries

### Medium-term Features

#### Advanced Kinematics
- **Prismatic Joint Support**: Linear joints for SCARA and Cartesian robots
- **Parallel Mechanisms**: Stewart platforms and Delta robots
- **Redundant Manipulators**: 7+ DOF arms with null-space optimization
- **Mobile Base Integration**: Combine arm with wheeled/tracked base

#### Control Algorithms
- **PID Joint Control**: Accurate trajectory following
- **Impedance Control**: Force-controlled interaction
- **Trajectory Planning**: Smooth path generation with via-points
- **Obstacle Avoidance**: Path planning around static obstacles

#### Simulation Features
- **Physics Integration**: Realistic dynamics using Bullet or similar
- **Force/Torque Sensors**: Simulated sensor feedback
- **Gripper Simulation**: End-effector tools and grasping
- **Environmental Interaction**: Objects to manipulate

### Long-term Vision

#### Multi-Robot Systems
- **Collaborative Robotics**: Multiple arms working together
- **Assembly Line Simulation**: Industrial automation scenarios  
- **Swarm Robotics**: Large numbers of coordinated robots
- **Human-Robot Collaboration**: Safe shared workspace simulation

#### Advanced Graphics
- **Physically Based Rendering**: Realistic materials and lighting
- **Virtual Reality Support**: Immersive robot programming
- **Augmented Reality**: Overlay simulation on real environments
- **Ray Tracing**: High-quality reflections and shadows

#### Integration Capabilities
- **ROS2 Interface**: Connect to Robot Operating System
- **CAD Import**: Load robot models from STEP/STL files
- **Real Robot Control**: Direct hardware interfacing
- **Machine Learning**: AI-based motion planning and control

#### Educational Features
- **Interactive Tutorials**: Guided learning experiences
- **Assessment Tools**: Automated grading for robotics courses
- **Simulation Library**: Pre-built scenarios for common tasks
- **Documentation Generator**: Automatic report creation

### Technical Architecture Improvements

#### Performance Optimization
- **GPU Acceleration**: CUDA/OpenCL for parallel kinematics
- **Level-of-Detail**: Adaptive mesh resolution based on distance
- **Frustum Culling**: Only render visible robot components
- **Instanced Rendering**: Efficient multi-robot visualization

#### Code Quality
- **Unit Testing**: Comprehensive test suite for all algorithms
- **Continuous Integration**: Automated builds and testing
- **Documentation**: Doxygen-generated API documentation
- **Internationalization**: Multi-language support

#### Cross-Platform Support
- **Mobile Platforms**: Android/iOS versions for education
- **Web Assembly**: Browser-based simulation
- **Cloud Computing**: Server-side simulation with web interface
- **Embedded Systems**: Lightweight version for microcontrollers

---

## 📚 Additional Resources

### Learning Materials

#### Robotics Textbooks
- **"Introduction to Robotics: Mechanics and Control"** by John J. Craig
- **"Robotics: Modelling, Planning and Control"** by Bruno Siciliano
- **"Modern Robotics"** by Kevin Lynch and Frank Park
- **"A Mathematical Introduction to Robotic Manipulation"** by Murray, Li, and Sastry

#### Online Courses
- **"Robotics Specialization"** - University of Pennsylvania (Coursera)
- **"Introduction to Robotics"** - Stanford University (edX)
- **"Robot Mechanics and Control"** - Seoul National University (Coursera)
- **"Modern Robotics"** - Northwestern University (Coursera)

#### Computer Graphics Resources
- **"Real-Time Rendering"** by Tomas Akenine-Möller
- **"OpenGL Programming Guide"** (Red Book)
- **"Learn OpenGL"** - learnopengl.com
- **"3D Math Primer for Graphics and Game Development"** by Fletcher Dunn

### Development Resources

#### Libraries and Tools
- **Eigen Documentation**: eigen.tuxfamily.org
- **Dear ImGui Examples**: github.com/ocornut/imgui
- **OpenGL Tutorials**: learnopengl.com
- **CMake Documentation**: cmake.org/documentation
- **GLFW Documentation**: glfw.org/documentation

#### Robotics Software
- **ROS (Robot Operating System)**: ros.org
- **MoveIt Motion Planning**: moveit.ros.org
- **Gazebo Simulator**: gazebosim.org
- **V-REP/CoppeliaSim**: coppeliarobotics.com

### Community and Support

#### Forums and Discussion
- **ROS Discourse**: discourse.ros.org
- **Reddit r/robotics**: reddit.com/r/robotics
- **Stack Overflow**: stackoverflow.com (tags: robotics, opengl, eigen)
- **GitHub Discussions**: Direct support in project repository

#### Professional Organizations
- **IEEE Robotics and Automation Society**: ieee-ras.org
- **International Federation of Robotics**: ifr.org
- **Association for Computing Machinery**: acm.org

---

## 📄 License and Contributing

### License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

### Contributing Guidelines

#### Bug Reports
1. Check existing issues before creating new ones
2. Provide detailed reproduction steps
3. Include system information and error messages
4. Add relevant screenshots or videos

#### Feature Requests
1. Describe the proposed functionality clearly
2. Explain the use case and benefits
3. Consider implementation complexity
4. Discuss with maintainers before major changes

#### Code Contributions
1. Fork the repository and create feature branches
2. Follow existing code style and conventions
3. Add unit tests for new functionality
4. Update documentation as needed
5. Submit pull requests with clear descriptions

#### Development Setup
```bash
# Fork and clone your fork
git clone https://github.com/yourusername/robosim.git
cd robosim

# Add upstream remote
git remote add upstream https://github.com/samilauronen/robosim.git

# Create feature branch
git checkout -b feature/your-feature-name

# Make changes, commit, and push
git add .
git commit -m "Add your feature description"
git push origin feature/your-feature-name

# Create pull request on GitHub
```

---

## 🙏 Acknowledgments

### Libraries and Dependencies
- **Eigen3**: High-performance linear algebra library
- **Dear ImGui**: Immediate mode graphical user interface
- **GLFW**: Multi-platform library for OpenGL applications
- **GLEW**: OpenGL Extension Wrangler Library
- **ImGuizmo**: 3D gizmo manipulation library

### Educational Inspiration
- Modern robotics courses from leading universities
- Open-source robotics community contributions
- Classical robotics textbooks and research papers

### Development Tools
- **CMake**: Cross-platform build system
- **Git**: Version control system
- **GitHub**: Repository hosting and collaboration
- **VS Code**: Development environment and debugging

---

*Happy robot simulation! 🤖*

For questions, suggestions, or support, please open an issue on the GitHub repository or contact the development team.
