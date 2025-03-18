# KSP Orbit Planner

Trying to make a tool to help me plan orbital maneuvers better, because orbital mechanics is counterintuitive.
Not sure when the last time I actually played KSP was though... I've definitely spent more time on this than
the actual game.

Right now it just simulates the Kerbol system, and unpowered ships. No maneuvering capability yet, but at this
point that's more of a UI problem than a physics or software problem.

## Math

Because KSP uses patched conic approximations, we can take advantage of the fact that the two-body problem has a closed-form solution. The full details can be found in this
repository under the [`latex/` directory](https://github.com/HenrySwanson/ksp-orbit-planner/blob/master/latex/supplementary-math.pdf), but the quick summary is:

- As long as it stays within the same sphere of influence (SOI), a satellite will trace out the same orbit forever
  - These are conic sections: circles, ellipses, parabolas, and hyperbolas. (And also the nasty edge case of radial orbits: those that go straight up-and-down.)
  - To represent all conic sections in a uniform way, we use [Stumpff functions](https://en.wikipedia.org/wiki/Stumpff_function). This allows us to smoothly transition between
    highly-eccentric ellipses, through parabolas, and to hyperbolas, without singularities.
- Given a initial position, and velocity, we can compute where each orbiting body will be after some time $\Delta t$
- An `Orrery` is a collection of initial positions and velocities for each planet/moon/satellite
  - This lets us "turn the crank" on the universe and watch the planets go round
- To model SOI changes, we represent the universe as a sequence of `Orrery`s, punctuated by SOI transitions.
  - At an SOI transition, we create a new `Orrery`, where the relevant satellite is re-rooted to its new parent body.
  - Also, we 'snapshot' the position and velocity of all bodies, and use that to compute the new initial state for the `Orrery`. This helps reduce error by
    preventing the $\Delta t$ we consider from growing too large.
- To detect SOI changes, we use the [Krawczyk-Moore](https://www.math.rug.nl/~gert/documents/2010/interval/moore_test.pdf) test as a root-finding method.
  - It has the wonderful property of being able to detect when a) there is only one root in an interval and b) Newton's method will converge to it.
  - This lets us inspect rather large intervals with the (more expensive) Krawczyk-Moore test, and revert to the (faster) Newton's method as soon as possible,
    without accidentally converging to the wrong root.

This approach allows us to get perfect rewind-and-replay. Since we compute everything from the `Orrery`'s initial state, which is fixed on creation, we can safely scrub back and
forth over time intervals that are of interest without accumulating error. Similarly, because the computation of the next `Orrery` is decoupled from the framerate of the visualization,
our simulation is deterministic.

## Controls
- Q and E: switch focused body
- Comma and Period: slow down and speed up time
- R: reverse direction of time
- Space: pause/unpause
- WASD: move camera around
- +/-: zoom in and out
- Esc: quit

![screenshot](screenshots/mun-encounter.png)
