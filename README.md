# motionAmigo

**A fast, SIMD-vectorized sampling-based motion planner for robot arms, written in Rust, with Python bindings and a live browser demo.**

> Work in progress. This README is filled in milestone by milestone.

motionAmigo plans collision-free motions for a Franka Emika Panda (7 DoF) with RRT-Connect and
shortcutting. Like [VAMP](https://github.com/KavrakiLab/vamp), it approximates the robot by spheres
and checks several configurations along an edge at once with SIMD instructions. It is an
independent Rust implementation of these ideas, not a port of the VAMP code.

## Scene format

motionAmigo shares its scene format with the sister project spatialAmigo, which resolves spatial
language such as "the mug left of the laptop" in 3D scenes. Together they form the chain
language, target object, collision-free motion.

* Schema: [`schema/scene-v0.1.json`](schema/scene-v0.1.json)
* Example: [`examples/scenes/tabletop.json`](examples/scenes/tabletop.json)

Units are meters, z points up, the frame is right-handed. Each object is an oriented box with a
`center`, full edge lengths `size` and a `yaw` angle about z.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT license](LICENSE-MIT)
at your option. See [NOTICE](NOTICE) for third-party data.
