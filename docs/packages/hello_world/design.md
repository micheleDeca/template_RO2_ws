# hello_world – Design Notes

The `hello_node` class:

- Inherits from `rclpy.node.Node`.
- Creates a timer with a fixed period (1 second).
- Logs a "Hello world" message on each timer callback.

This package is intentionally simple so that the student can:

- add publishers/subscribers
- introduce parameters
- extend the architecture gradually
- use this file to explain design decisions and trade-offs.

