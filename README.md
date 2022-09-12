# RBOIDS - Boids in Rust
This is an experiment to simulate a school of fish in Rust by implementing the boids algorithm. It is wholly inspired by Sebastian Lague's - [Coding adventure with boids](https://www.youtube.com/watch?v=bqtqltqcQhw).

The experiment has been successful in it's object to,
1. Explore and learn Rust
2. Get dirty with graphics and graphics mathematics
3. Be awed by procedural generation

However as I proceeded through the experiment I felt that this deserved more attention. In turn, it got absorbed into a blog that walks you through making simulating your own school of boids.
___

First feast your eyes on this beauty.

![boids in rust](../assets/presentation.gif?raw=true)

The framerate and resolution took a hit in trying to display it here. Here's a better [version](https://drive.google.com/file/d/1ri4x-jCX8SA9oX8OqDIKtXhYIrEKlGjO/view). Or You can clone this repo and `cargo run` to see the simulation in a much better quality.
___
The blog is written as a three part series.
1. [A fistful of boids](https://github.com/twitu/twitu/blob/main/a-fistful-of-boids.md) - Setting up the scene and basic animation
2. [For a few boids more](https://github.com/twitu/twitu/blob/main/for-a-few-boids-more.md) - Generating boids and obstacle avoidance
3. [The school, the boid and the Rusty](https://github.com/twitu/twitu/blob/main/the-school-the-boid-and-the-rusty.md) - Simulating a flock, parallelism and benchmarking performance

It is also divided into ten tagged stages to mark milestones. You can begin from anyone of them but I would suggest starting from the first post in the series. It should not take more than 1.5 hours to code through the whole series.

I'm a beginner at Rust so I will appreciate any suggestions for improving the code. Any questions and issues related to the blog are also welcome, just file an issue or mail me.

## Troubleshooting
Here I try to address some of the common errors people face when trying to run the project.     
- *`failed to run custom build command for alsa-sys v0.1.2`*: This error occours beacuse the ala-sys dependency requires an external package for compiling correctly. [Refer here](https://github.com/RustAudio/rodio/issues/200) to solve the issue
