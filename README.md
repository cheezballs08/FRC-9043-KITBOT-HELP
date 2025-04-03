# For the friendly fellows I met at reddit!

## Usage
If using pathplanner this project will produce errors if you don't set up pathplanner.To set up, open the project in Pathplanner GUI and then set the Holonomic mode to off in the settings. 

CTRL + SHIFT + P -> Simulate Robot Code -> Sim GUI

Then open advantageScope -> Files -> Connect to Simulator 

Then come to 3Dfield find the logged pose and drag it to the Poses section.

From the sim GUI, set Joystick[0] to whatever joystick you want to use and activate teleop or auto depending on what you want to do!

## Misc
I ***highly*** recommend you add logging to your robot. DogLog makes this incredibly simple, a link to their docs is at the end of this README.

## Notes
I couldn't get pathplanner to work without sensors so this will probably only help you guys with drive practice and auto without pathplanner. (Driving the robot forwards 3 seconds for example). There are also no collisions so be carefull.

If you guys want to test pathplanner too then add the commented lines back in.

I'd recommend just downloading this project and using this to test sims and use your current codebase for your competition code.

If you decide to implement this to your own code, you must install MapleSim and DogLog for this to work. Don't forget to add this:

```java
@Override
public void simulationPeriodic() {
  SimulatedArena.getInstance().simulationPeriodic();
}
```
to Robot.java

I'd recommend changing the method names to whatever your current subsystem is using so you can change between them more easily. Maybe even implement a common interface!

You can ask me for implementation help anytime you want. Just DM or comment on reddit.

Hope yall the best! 
-9043

DogLog: https://doglog.dev/

Maple SIM: https://shenzhen-robotics-alliance.github.io/maple-sim/

Pathplanner: https://pathplanner.dev/home.html