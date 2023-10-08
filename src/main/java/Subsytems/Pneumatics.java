package Subsytems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.ControllerConstants;

public class Pneumatics {
    private DoubleSolenoid Lift;
    private DoubleSolenoid Cube;
    private DoubleSolenoid Cone;
    private boolean isInConeMode = false;

    public void controls()
    {
        if (ControllerConstants.kManipulator.getRawButton(ControllerConstants.kLeftOptions)) 
        {
            isInConeMode = false;
        }
        if (ControllerConstants.kManipulator.getRawButton(ControllerConstants.kRightOptions)) 
        {
            isInConeMode = true;
        }
        if (ControllerConstants.kManipulator.getRawButton(ControllerConstants.kY))
        {
            if (isInConeMode)
            {
                enableCone();
            }
            else
            {
                enableCube();
            }
        }
        if (ControllerConstants.kManipulator.getRawButton(ControllerConstants.kA))
        {
            if (isInConeMode)
            {
                disableCone();
            }
            else
            {
                disableCube();
            }
        }
        if (ControllerConstants.kManipulator.getRawAxis(ControllerConstants.kRightTrigger)>0.02)
        {
            enableLift();
        }
        else
        {
            disableLift();
        }
    }

    public Pneumatics()
    {
        Lift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
        Cube = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 4, 5);
        Cone = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 1);
    }

    public void enableLift()
    {
        Lift.set(Value.kForward);
    }

    public void disableLift()
    {
        Lift.set(Value.kReverse);
    }

    public void enableCone()
    {
        Cone.set(Value.kForward);
    }

    public void disableCone()
    {
        Cone.set(Value.kReverse);
    }

    public void enableCube()
    {
        Cube.set(Value.kForward);
    }

    public void disableCube()
    {
        Cube.set(Value.kReverse);
    }
}
