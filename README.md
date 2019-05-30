# Particle-filter
Particle filter assignment for Udacity.

# Changes from the original code:
* Changed names of private member variables in ParticleFilter (added underscore to the name.)
* Added `std_pos` as ParticleFilter class member to avoid reinitialization on each function call.
* `num_particles` can now be initialized via init().
* Added a new constructor that does the same work as init().


# TODO:
* Implement updateWeights()
* Implement a simulator that works without the need for websockets.
* Implement destructor.
