## Design  Ideas  
### Robot's  
- Each robot's reads it's own MAC address (or some other unique value), and uses this as it's ID when communicating with the server
- Why? This way each robot uses the exact same code, there is no hard-coded ID coded into each robot. Good for scalability.  

- Sensor on the robot detects if there is an object and stops moving when detected. The server checks (after that) if the robot can continue moving, and/or has to change its path.
