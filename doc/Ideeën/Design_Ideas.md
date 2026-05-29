## Design  Ideas  
### Robot's  
- Each robot's reads it's own MAC address (or some other unique value), and uses this as it's ID when communicating with the server
- Why? This way each robot uses the exact same code, there is no hard-coded ID coded into each robot. Good for scalability.  

- Sensor on the robot detects if there is an object and stops moving when detected. The server checks (after that) if the robot can continue moving, and/or has to change its path.
---
  - ESP-NOW
  - Why ESP-NOW? its faster and it skips the handshake mqtt uses, also the router isnt used so the path the data makes is way shorter and faster than when youre reliable on the router and the broker. 
<br>
<br>
<br>



### Path planning
#### Code:
- pad berekenen
- als sensor object detect
  - Zijkant;
    - Doorrijden en beetje uitwijken?
  - Voorkant;
    - Stoppen + wachten
    - anticiperen
    - nieuw pad berekenen of oud pad vervolgen
- indien object gedetecteerd wordt, check met (hemelsbreed) afstand of het een andere robot is of niet, en daarop hanteren?
- zou niet moeten gebeuren dat ze elkaar tegen komen? goede pad berekening
<br><br>
- bij samenkomen (Forming Forms), afstand naar andere robots wordt verwaarloost of object detectie verminderd tot correct hanteerbare afstand
