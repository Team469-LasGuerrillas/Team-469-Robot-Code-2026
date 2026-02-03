package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import frc.robot.Constants;

public class AutoClimb {
    
    private static DoubleSupplier nextHookPos = () -> Constants.Climb.L3_POS;

    public static void setNextHookPos(DoubleSupplier newHookPos) {
        nextHookPos = newHookPos;
    }


    public static void autoClimbL3() {
        setNextHookPos(() -> Constants.Climb.L3_POS);
        
    }

    private static void autoClimbL1() {
        setNextHookPos(() -> Constants.Climb.L1_POS);
    }
}
