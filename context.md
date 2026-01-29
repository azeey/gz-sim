# Mujoco Integration for Gazebo Sim

## Overview

The primary goal of this project is to integrate the Mujoco physics engine into Gazebo Sim. Work has proceeded along two different paths, resulting in two prototype implementations. The current and recommended path forward is the direct `gz-sim` system integration, which bypasses the `gz-physics` abstraction layer.

This document summarizes the state of both prototypes to provide context for future development.

---

## Mujoco API

The integration uses a recent version of the Mujoco API, which allows for programmatic scene generation and fine-grained control over the simulation. The headers are located in `~/ws/jetty/install/include/mujoco`.

The key available headers and their purposes are:

-   `mujoco.h`: The main header, providing core functions like `mj_step`, `mj_forward`, etc.
-   `mjspec.h`: The API for programmatically constructing a model specification (`mjSpec`). This is heavily used by the current `gz-sim` system to build scenes from the ECM.
-   `mjmodel.h`: Defines the compiled model structure (`mjModel`).
-   `mjdata.h`: Defines the simulation state and data structure (`mjData`), which is accessed at every step to sync states.
-   `mjrender.h` / `mjvisualize.h`: APIs for rendering and visualization, which could be used for debugging.
-   `mjplugin.h`: Header for developing Mujoco engine plugins.

The availability of these modern APIs, especially `mjspec.h`, is the key enabler for the direct `gz-sim` system approach, as it avoids the need to write MJCF XML files manually.


---

## Approach 1: `gz-physics` Plugin (Prototype)

- **Location:** `@mujoco/**`
- **Architecture:** This implementation follows the standard Gazebo methodology for integrating a new physics engine by creating a plugin for the `gz-physics` library. Gazebo's `Physics` system communicates with `gz-physics`, which in turn loads and interfaces with this Mujoco plugin.
- **Mechanism:** It works by parsing `sdf::World` and `sdf::Model` objects and dynamically building a corresponding Mujoco `mjSpec`. This spec is then compiled into an `mjModel`, and the simulation is advanced via `mj_step`.

### Current State & Limitations

This prototype successfully demonstrates the ability to load SDF files and step the simulation. However, as detailed in `@mujoco/README.md`, it is significantly incomplete and lacks most features required for a useful simulation:

-   **No Joints:** Joint creation, control, and state feedback are not implemented.
-   **No Contact Reporting:** The `GetContactsFromLastStepFeature` is missing, so contact data is unavailable.
-   **No External Forces:** Applying forces or torques to links is not supported.
-   **Incomplete Entity Management:** Many functions for querying and removing entities are stubbed out.
-   **Piecemeal construction:** Runtime construction of models, links and collisions is not supported.

**Conclusion:** This `gz-physics` plugin served as a valuable proof-of-concept for SDF-to-`mjSpec` translation but has been superseded by the more direct and capable `gz-sim` system approach.

---

## Approach 2: Direct `gz-sim` System (In Progress)

- **Location:** `@src/systems/physics/MujocoPhysics.cc`
- **Architecture:** This implementation bypasses `gz-physics` entirely. It is a native Gazebo `System` that interacts directly with the Entity Component Manager (ECM). This provides more direct control and avoids the limitations of the `gz-physics` abstraction layer.
- **Mechanism:**
    1.  **Scene Parsing:** On simulation start, or when a model is added/removed, the `RebuildModel` method is triggered. It traverses the ECM to build a kinematic tree of all models, links, and joints.
    2.  **Model Construction:** This tree is used to construct a Mujoco `mjSpec` programmatically, creating `mjsBody`, `mjsJoint`, and `mjsGeom` objects corresponding to Gazebo entities. Custom components (`MujocoBodyId`, `MujocoJointId`) are used to map Gazebo `Entity` IDs to Mujoco's integer-based IDs.
    3.  **Update Loop:** In each `Update`, the system performs a three-phase process:
        -   **Sync to Mujoco:** Applies user commands (e.g., `JointForceCmd`) from the ECM to the `mjData` structure.
        -   **Step:** Calls `mj_step()` to advance the physics simulation.
        -   **Sync from Mujoco:** Updates Gazebo components (e.g., `JointPosition`, `JointVelocity`, `Pose`) in the ECM with the new state from `mjData`.

### Current Progress

This approach is significantly more advanced and functional than the `gz-physics` plugin.

-   **What's Working:**
    -   **Scene Construction:** Building a valid `mjModel` from a complex Gazebo scene in the ECM, including models with `revolute`, `prismatic`, and `ball` joints.
    -   **Simulation Step:** The core `mj_step` loop is functional.
    -   **Force Control:** Applying joint forces via the `components::JointForceCmd` component.
    -   **State Feedback:** Syncing joint positions and velocities from Mujoco back to the ECM.

-   **What's In Progress / Partially Implemented:**
    -   **Pose Updates:** The logic to sync root link poses from Mujoco back to the ECM exists but is currently commented out in the `Update` method.

### Next Steps & Path Forward

Development should focus exclusively on this `gz-sim` system.

-   **Immediate TODOs:**
    1.  **Enable Pose Updates:** Uncomment and validate the pose update logic for models in the `Update` method.
    2.  **Verify Pose Calculations:** Review the pose calculations in `AddBodyRecursive`. The relative pose logic may need adjustment to ensure correctness within Mujoco's coordinate frames.
    3.  **Implement Contact Reporting:** Query `mjData` for contact information after each step and publish it using Gazebo's contact sensor components.
    4.  **Implement External Forces:** Add support for applying external wrenches to links via standard Gazebo components.

-   **Longer-Term Goals:**
    -   Support more of Mujoco's features, such as actuators and sensors.
    -   Improve runtime performance, possibly by enabling more granular recompiles (`mj_recompile`) instead of full rebuilds.
    -   Add support for more geometry types and SDF features.
    -   Enhance error handling and reporting.
