package Subsytems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants.ControllerConstants;

public class Pneumatics {
    private DoubleSolenoid Lift;
    private DoubleSolenoid Cube;
    private DoubleSolenoid Cone;
    private boolean ConeClose=false;
    private boolean CubeClose=false;
    private boolean liftOut=true;

    public void controls()
    {
        if (ControllerConstants.kManipulator.getRawButtonPressed(ControllerConstants.kY))
        {
            if (ConeClose)
            {
                ConeClose = false;
                enableCone();
            }
            else
            {
                ConeClose = true;
                disableCone();
            }
        }
        if (ControllerConstants.kManipulator.getRawButtonPressed(ControllerConstants.kX))
        {
            if (CubeClose)
            {
                CubeClose = false;
                enableCube();
            }
            else
            {
                CubeClose = true;
                disableCube();
            }
        }
        if (ControllerConstants.kManipulator.getRawAxis(ControllerConstants.kRightTrigger)>0.02)
        {
            if (!liftOut)
            {
                liftOut=true;
                disableLift();
            }
        }
        else
        {
            if (liftOut)
            {
                liftOut = false;
                enableLift();
            }
        }
    }

    public Pneumatics()
    {
        Lift = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 2, 3);
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
