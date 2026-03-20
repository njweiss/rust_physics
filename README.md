# Rust 3D Rigid Body Physics Engine

A custom-built, multithreaded 3D rigid body physics engine written in Rust. This project implements a full physics pipeline from scratch—including broad-phase spatial hashing, narrow-phase GJK/EPA collision detection, and a parallelized sequential impulse solver. 

Beyond standard macroscopic physics, this engine includes features inspired by molecular dynamics, such as compound rigid body support for complex molecular structures and a kinetic thermostat for temperature control. It uses [`nalgebra`](https://nalgebra.rs/) for linear algebra and [`kiss3d`](https://kiss3d.rs/) for real-time visualization.

## Features

### Collision Detection
* **Broad-Phase (Spatial Hashing):** Utilizes a 3D grid-based spatial hashing algorithm to efficiently cull non-colliding object pairs, reducing the algorithmic complexity of dense simulations.
* **Narrow-Phase (GJK & EPA):** Implements the Gilbert-Johnson-Keerthi (GJK) algorithm for boolean collision queries and the Expanding Polytope Algorithm (EPA) for generating precise contact manifolds and penetration depths for arbitrary convex shapes (currently supporting Spheres and OBBs).

### Physics & Constraint Solving
* **Sequential Impulse Solver:** Solves velocity constraints iteratively to handle normal forces, restitution (bounciness), and friction.
* **Parallel Constraint Solving:** Leverages [`rayon`](https://github.com/rayon-rs/rayon) and a custom `UnionFind` implementation to isolate disjoint collision islands, allowing the constraint solver to safely process multiple contacts in parallel.
* **Baumgarte Stabilization:** Prevents object sinking and ensures stable stacking by introducing position correction into the velocity constraints.

### Dynamics & Simulation
* **Compound Rigid Bodies:** Robust support for rigid bodies composed of multiple colliders with their own local offsets and mass properties. The engine automatically calculates the aggregate center of mass and uses the Parallel Axis Theorem to compute the global inertia tensor.
* **Thermostat:** Includes a kinetic temperature control system that scales linear and angular velocities to maintain a target thermodynamic temperature, useful for simulating molecular ensembles.

## Getting Started

### Prerequisites
Make sure you have [Rust and Cargo installed](https://rustup.rs/). You will also need the standard build dependencies for `kiss3d` (e.g., CMake, basic graphics libraries depending on your OS).

### Running the Simulation
Because physics simulations and iterative solvers are computationally expensive, it is heavily recommended to run the project in release mode for a smooth framerate.

```bash
# Clone the repository
git clone [https://github.com/yourusername/rust_physics.git](https://github.com/yourusername/rust_physics.git)
cd rust_physics

# Run the simulation in release mode
cargo run --release
```

### The Demo
By default, `main.rs` drops 50 compound water molecules (using `spawn_water_molecule`) into a bounded box container. The simulation applies the thermostat and resolves inter-molecular collisions in real-time.

## Architecture Overview

* **`world.rs`**: The core simulation hub. Manages the physics pipeline (stepping the simulation, broad-phase culling, island generation, parallel constraint solving, and integration).
* **`rigid_body.rs`**: Defines properties of `RigidBody`, `Collider`, `OBB`, and `AABB`. Handles local-to-global transformations and inertia tensor calculations.
* **`collision.rs`**: The math-heavy narrow-phase logic. Contains the implementations of the GJK algorithm, EPA, and Contact manifold generation.
* **`constraint.rs`**: Defines the `VelocityConstraint` mathematics, calculating Jacobians and applying impulses for normal and frictional responses.
* **`broadphase.rs`**: Defines the broad-phase trait and implements `NSquared` and `SpatialHashing` approaches.
* **`utils.rs`**: Contains helper functions for random generation, safe normalization, and data structures (`UnionFind`) necessary for parallel processing.

## Future Improvements
* Implement continuous collision detection (CCD) to prevent tunneling at high velocities.
* Expand the narrow-phase to support arbitrary convex hulls and meshes.
* Implement a Sleeping mechanism to freeze inactive bodies and save CPU cycles.
* Add rotational friction (rolling/spinning friction).

## License
[MIT License](LICENSE)
