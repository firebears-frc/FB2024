// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.sql.DriverPropertyInfo;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Bass;
import frc.robot.subsystems.DownBeat;
import frc.robot.subsystems.Glissando;
import frc.robot.subsystems.UpBeat;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private final Bass m_robotDrive = new Bass();
    private final DownBeat m_intake = new DownBeat();
    private final UpBeat m_shooter = new UpBeat();
    private final Arm m_arm = new Arm();
    //private final Glissando m_climb = new Glissando();
    private Vision vision;
    private final CommandJoystick one = new CommandJoystick(0);
    private final CommandJoystick two = new CommandJoystick(1);
    private final CommandXboxController xboxController = new CommandXboxController(2);
    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("AutoChooser",
            AutoBuilder.buildAutoChooser());

    public RobotContainer() {
        try {
            vision = new Vision(m_robotDrive::visionPose);
        }
        catch(IOException e){
            DriverStation.reportWarning("Unable to initialize vision", e.getStackTrace());
        }
        configureBindings();

        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -MathUtil.applyDeadband(one.getY(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(one.getX(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(two.getX(), OIConstants.kDriveDeadband),
                                true, true),
                        m_robotDrive));
    }

    private void configureBindings() {
        one.trigger().toggleOnTrue(new StartEndCommand(m_robotDrive::setX, () -> {
        }, m_robotDrive));
        two.trigger().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

        xboxController.a().onTrue(m_intake.intakeNote()).onFalse(m_intake.pauseDownBeat());
        xboxController.x().onTrue(m_intake.dischargeNote()).onFalse(m_intake.pauseDownBeat());
        xboxController.y().toggleOnTrue(m_shooter.shootNote());
        xboxController.b().onTrue(m_arm.pickUp()); 
        xboxController.rightBumper().onTrue(m_arm.speakerShoot());
        xboxController.povUp().onTrue(m_arm.stow());
        xboxController.povDown().onTrue(m_arm.ampShoot());
        
        xboxController.leftTrigger().onTrue(Commands.sequence(
            m_arm.ampShoot(),
            m_shooter.ampSpeed()
        )).onFalse(Commands.sequence(  
            m_intake.intakeNote(),
            Commands.waitSeconds(1),
            m_shooter.pauseUpBeat(),
            m_intake.pauseDownBeat(),
            m_arm.pickUp()
        ));

    
        m_arm.setDefaultCommand(
            m_arm.defaultCommand(
                    () -> MathUtil.applyDeadband(
                            xboxController.getLeftY(),
                            0.2
                    )
            )
        );
        
        //xboxController.povUp().onTrue(m_climb.climb()).onFalse(m_climb.pauseClimb());
        //xboxController.povDown().onTrue(m_climb.unclimb()).onFalse(m_climb.pauseClimb());
        //xboxController.b().onTrue(m_shooter.reverseShootNote()).onFalse(m_shooter.pauseUpBeat());

    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
