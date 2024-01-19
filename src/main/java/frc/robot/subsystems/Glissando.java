package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Glissando extends SubsystemBase {
    private CANSparkMax climbRight;
    private CANSparkMax climbLeft;

    private static int stallLimit = 30;
    private static int freeLimit = 25;
    private static int scndLimit = 35;

    public Glissando() {
        climbRight = new CANSparkMax(14, MotorType.kBrushless);
        climbLeft = new CANSparkMax(15, MotorType.kBrushless);

        climbRight.setInverted(false);
        climbLeft.setInverted(true);

        climbRight.setIdleMode(IdleMode.kBrake);
        climbLeft.setIdleMode(IdleMode.kBrake);

        climbRight.setSmartCurrentLimit(stallLimit, freeLimit);
        climbLeft.setSmartCurrentLimit(stallLimit, freeLimit);

        climbRight.setSecondaryCurrentLimit(scndLimit);
        climbLeft.setSecondaryCurrentLimit(scndLimit);
    }

    /**
     * public void climb(){
     * climbRight.set(0.25);
     * climbLeft.set(0.25);
     * }
     * 
     * public void unclimb(){
     * climbRight.set(-0.25);
     * climbLeft.set(-0.25);
     * }
     **/

    public Command climb() {
        return Commands.runOnce(() -> {
            climbRight.set(0.25);
            climbLeft.set(0.25);
        }, this);
    }

    public Command unclimb() {
        return Commands.runOnce(() -> {
            climbRight.set(-0.25);
            climbLeft.set(-0.25);
        }, this);
    }

    public Command pauseClimb() {
        return Commands.runOnce(() -> {
            climbRight.set(0);
            climbLeft.set(0);
        }, this);
    }
}
