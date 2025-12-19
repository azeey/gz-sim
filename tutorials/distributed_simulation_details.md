# Distributed Simulation in Gazebo: A Detailed Guide

This document provides a detailed, technical explanation of the distributed simulation feature in Gazebo. It is intended for developers who want to understand the underlying architecture and data flow. It complements the higher-level overview found in `distributed_simulation.md`.

## 1. High-Level Architecture

Gazebo's distributed simulation employs a **Primary-Secondary (Master-Slave)** architecture.

-   **Primary**: A single, central node that acts as the coordinator. It does not perform physics calculations but is responsible for:
    -   Synchronizing the simulation clock.
    -   Distributing work to secondaries.
    -   Aggregating state changes from all secondaries to maintain a consistent world view.
    -   Managing performer handoffs between secondaries.
    -   Serving as the single point of interaction for GUIs and user commands.

-   **Secondaries**: One or more worker nodes that perform the heavy lifting. Each secondary is responsible for:
    -   Running physics and sensor simulation for a subset of the models in the world.
    -   Loading and unloading parts of the environment (levels) as needed.
    -   Reporting state changes for the models it simulates back to the primary at the end of each step.

This architecture allows the simulation load to be spread across multiple processes or machines, enabling larger and more complex simulations than a single instance could handle.

## 2. Key Concepts

-   **Performer**: A model designated in the SDF file to be a dynamic agent whose location determines which parts of the world are active. The performer is the fundamental **unit of distribution**. The primary assigns each performer to a single secondary.
-   **Level**: A collection of world entities (models, lights, etc.) and a bounding volume. Levels are loaded and unloaded dynamically based on the location of performers. A level becomes active (and its entities are loaded into the simulation) when a performer enters its bounding volume.
-   **Affinity**: The mapping that dictates which secondary is responsible for simulating which performer. The primary is solely responsible for setting and changing affinities.

## 3. Initialization and Startup

The startup process is designed to ensure all nodes begin in a synchronized state.

1.  **Shared World Definition**: All nodes (the primary and all secondaries) are launched and configured to load the **exact same world SDF file**.
2.  **Initial Entity Creation**: Upon loading the SDF, each node's `SimulationRunner` creates all the entities defined within, including all potential levels and all performers. At this initial moment, every node's Entity Component Manager (ECM) contains a representation of the entire world.
3.  **Peer Discovery**: The nodes use `gz-transport` to discover each other. The primary waits until the expected number of secondaries have announced themselves.
4.  **Initial Affinity and Pruning**: In the very first simulation step, the primary calculates the initial performer affinities and communicates them to all secondaries via the `SimulationStep` message.
5.  **ECM Pruning**: Each secondary receives this initial affinity list. It iterates through all the performer models that exist in its local ECM. If a performer is **not** assigned to it, the secondary completely removes the performer's model (and all its child entities) from its local ECM using `RequestRemoveEntity`.

After this initial pruning, each secondary's ECM contains only the static world, all level definitions, and the specific performer models it has been assigned.

## 4. The Simulation Step: A Detailed Walkthrough

One simulation step proceeds as follows:

1.  **Primary Sends Step Command**: The primary publishes a `SimulationStep` message. This message contains the current simulation time and, crucially, any changes to performer affinities.
2.  **Secondaries Receive and Update**: Each secondary receives the `SimulationStep` message.
    -   It first processes any affinity changes, potentially removing a performer it no longer owns or noting that it has gained a new one (see Section 6: Performer Handoff).
    -   It then calls its main `stepFunction`, which is `SimulationRunner::Step`.
3.  **Secondary Simulation**: Inside `SimulationRunner::Step` on each secondary:
    -   The `LevelManager` runs its `UpdateLevelsState` method. It checks the position of only the performers that exist in its local (pruned) ECM.
    -   If a performer has entered a new level's bounding box, the `LevelManager` uses `SdfEntityCreator` to load that level's entities from the in-memory SDF data, creating them in the local ECM.
    -   If a performer has left a level, the entities are removed from the ECM.
    -   The `Physics` system's `Update` method is called. It operates on **all** non-static entities currently in the local ECM. This includes the assigned performers and any entities from active levels. Because unassigned performers and inactive levels don't exist in the local ECM, they are not simulated, ensuring no work is duplicated.
4.  **Secondaries Report State**: After completing its local simulation step, each secondary serializes the state (`ecm->State(...)`) of **only the entities belonging to its assigned performers** and sends this back to the primary in a `step_ack` message.
5.  **Primary Aggregates State**: The primary waits until it has received a `step_ack` from every secondary. It then applies the state updates from all secondaries to its own master ECM using `ecm->SetState()`. This merges all the parallel simulations into one consistent world state on the primary.
6.  **Cycle Repeats**: The primary increments the simulation time and begins the next step.

## 5. Performer Handoff (Moving Between Secondaries)

The handoff process is critical for dynamic load balancing. Here is how a performer `P` moves from `Secondary A` to `Secondary B`.

1.  **Handoff Initiated**: The primary decides `P` should now be managed by `B`. It sends a `SimulationStep` message with the updated affinity (`P -> B`).

2.  **`Secondary A` (Losing Affinity)**:
    -   Receives the `SimulationStep` message.
    -   The `NetworkManagerSecondary` sees the new affinity for `P` is not itself.
    -   It calls `RequestRemoveEntity` on `P`'s model. This triggers the removal of the model and its children from `A`'s local ECM and, consequently, from its physics engine.
    -   The `SystemManager` on `A` sees the entities are removed and unloads any associated plugins.

3.  **`Secondary B` (Gaining Affinity)**:
    -   Receives the same `SimulationStep` message.
    -   The `NetworkManagerSecondary` sees it has gained affinity for `P`.
    -   To bring the performer into existence, the primary sends the full state of `P`'s model to `B` (this is the "upcoming" feature noted in the documentation).
    -   `Secondary B` receives this state data and uses `EntityComponentManager::SetState()` to create `P`'s model and all its children in its local ECM. `SetState` is capable of creating entities and components that don't yet exist.

4.  **Plugin Lifecycle during Handoff**:
    -   When `Secondary B` recreates the model, it does so from the state information provided by the primary. This state includes all the component data.
    -   The `SystemManager` on `B` is responsible for loading the plugins associated with the new entities based on this state information.
    -   Crucially, the plugins are **instantiated anew** on `Secondary B`. They do not retain their internal memory or variable states from `Secondary A`. The plugin's state is completely reset.
    -   Any required persistent state must be handled externally, for example, by having the plugin subscribe to a state topic that the primary might publish.

## 6. Architectural Components

-   `NetworkManager`: The abstract base class.
    -   `NetworkManagerPrimary`: Manages the primary side. Decides affinities, publishes the `SimulationStep`, and aggregates `step_ack` messages.
    -   `NetworkManagerSecondary`: Manages the secondary side. Receives `SimulationStep`, processes affinity changes, and triggers the local simulation run.

-   `SimulationRunner`: The main driver for the simulation loop within a single instance. The `stepFunction` provided to the `NetworkManager` is `SimulationRunner::Step`.

-   `LevelManager`: Responsible for the dynamic loading/unloading of levels. On a secondary, it only sees the performers present in the local ECM, thus only loading levels relevant to its assigned work.

-   `SdfEntityCreator`: A utility used by the `LevelManager` to create entities in the ECM from the parsed `sdf::World` data.

-   `EntityComponentManager` (ECM): The heart of the simulation's state. The entire distributed simulation mechanism is a sophisticated process of keeping the ECM on each secondary selectively pruned to represent only its slice of the world. The `SetState()` method's ability to create entities from a serialized message is key to the handoff process.

## 7. Critical Caveat: Performer-Performer Interaction

A critical design principle of the distributed simulation feature is that **performers that can physically interact must be simulated on the same secondary.**

This is achieved by grouping performers based on the level they currently occupy. The `NetworkManagerPrimary` assigns all performers within the same level to the same secondary, ensuring their interactions can be correctly calculated in a single physics world.

However, this leads to a significant limitation in worlds where no levels are defined:

- In a "no-levels" scenario, the affinity logic in `NetworkManagerPrimary` does **not** assign all performers to a single secondary. Instead, it distributes the performers among the available secondaries in a round-robin fashion.
- This creates a situation where `Performer A` may be simulated on `Secondary A` and `Performer B` on `Secondary B`. Because neither secondary's physics engine is aware of the other's performer, **any physical interaction or collision between them will be missed.**
- This behavior is confirmed by the source code in `NetworkManagerPrimary::PopulateAffinities`.

Therefore, for any simulation where multiple performers may physically contact each other, **you MUST use levels to ensure correct physical interactions.** The level geometry serves as the boundary that tells the primary which performers need to be co-located on the same secondary. Without levels, correct performer-performer interaction is not guaranteed.
