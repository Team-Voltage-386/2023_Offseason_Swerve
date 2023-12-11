# Important Info

## CANCoder
### Replacing Them
If you replace one, you will HAVE to update a set of things in the code. We store a checklist here, so we can remember to change stuff when this happens.
1. Make all drive motors and encoders consistent i.e. all 4 should be positive when spinning in the same direction e.g. forward
    1. Turn the wheels so they are facing forward from the perspective of the robot.
    2. When you turn the wheels controlled by the drive motor, you want all the drive motors to have positive values when spun in one direction e.g. forward and then you want all the drive motors to have negative values when spun in the opposite direction e.g. backward.
        * Overload the disabledPeriodic in Robot to send Swerve Module data to SmartDashboard. This will allow you to gather the needed constants without enabling the robot.
2. Update the Encoder offset
    * Overload the disabledPeriodic to send Swerve Module data to SmartDashboard to update this offset
    1. Use a long, straight edge to make sure that all the swerve modules are facing the same direction.
    2. Take the values from Smart Dashboard (which should include the CANCoder readings) and plug them into the encoder offsets in Constants. Make sure that you are using the correct units (radians) and that the values are in the correct range (-PI to PI).
3. Make sure the **Turning Encoder** direction matches the **Turning Motor** direction
    * If you notice that the swerve module oscillates irregularly especially with no input and if it's not chasing the target value, this could be the cause i.e. that the encoder and motor disagree about which direction is positive.
## Tuning PID
**Note:** do not use PID for the drive wheel. only the turning motors need it. You are going to end up with uncurable oscillations.
### P (Power)
1. Always start on a very low P value. usually 0.01 is a good bet. Depending on how large you expect the error to be, 0.1 may be acceptable.
2. Double this value until you get some motion.
3. From here, proceed to increase value until you reach a stable oscillation.
    * A stable oscillation means the oscillations do not converge or diverge and just hold a consistent amplitude.
4. When you get this value, cut it in half. This will be your P value for a while. You may fine adjust it later, but as far as most applications go, this is as much accuracy as you will need.
### I (Integral)
1. We found that I is negligibly useful in terms of the swerve drive.
### D (Derivative)
1. We found that D is negligibly useful in terms of the swerve drive
