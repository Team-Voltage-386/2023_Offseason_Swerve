package Subsytems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Pneumatics {
    public DoubleSolenoid Lift;
    public DoubleSolenoid Cube;
    public DoubleSolenoid Cone;

    public Pneumatics()
    {
        Lift = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 2, 3);
        Cube = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 4, 5);
        Cone = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 1);
    }

    
}
