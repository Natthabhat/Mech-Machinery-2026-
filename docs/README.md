# Documentation

Add assembly, setup, operation, troubleshooting, and maintenance instructions. Include required tools, safety notes, revision history, and photographs or diagrams where useful.

## Symbolic math walk-through

Files named in code font but not linked (`delta_symbolic.m`, `delta_kinematics*.m`, `delta_dynamics.m`,
`delta_motor_torque.m`, `delta_sym_*.m`) were used to derive and verify the results; they are not part of this
repository, which keeps only the multibody model and the symbolic animation.

1. [The points of one leg](symbolic-math/01-leg-points.md)
2. [The forearm-length equation](symbolic-math/02-forearm-equation.md)
3. [Solving for q](symbolic-math/03-solving-for-q.md)
4. [Velocity and the Jacobian](symbolic-math/04-velocity-jacobian.md)
5. [Acceleration](symbolic-math/05-acceleration.md)
6. [Dynamics: from motor torque to hand force](symbolic-math/06-dynamics.md)
7. [Putting it together: the animation script](symbolic-math/07-animation-script.md)
