// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package com.seiford;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.seiford.subsystems.arm.Arm;
import com.seiford.subsystems.arm.ArmIO;
import com.seiford.subsystems.arm.ArmIOSim;
import com.seiford.subsystems.arm.ArmIOSparkMax;
import com.seiford.subsystems.climber.Climber;
import com.seiford.subsystems.climber.ClimberIO;
import com.seiford.subsystems.climber.ClimberIOSim;
import com.seiford.subsystems.climber.ClimberIOSparkMax;
import com.seiford.subsystems.drive.Drive;
import com.seiford.subsystems.drive.GyroIO;
import com.seiford.subsystems.drive.GyroIONavX;
import com.seiford.subsystems.drive.ModuleIO;
import com.seiford.subsystems.drive.ModuleIOMAXSwerve;
import com.seiford.subsystems.drive.ModuleIOSim;
import com.seiford.subsystems.intake.Intake;
import com.seiford.subsystems.intake.IntakeIO;
import com.seiford.subsystems.intake.IntakeIOSim;
import com.seiford.subsystems.intake.IntakeIOSparkMax;
import com.seiford.subsystems.shooter.Shooter;
import com.seiford.subsystems.shooter.ShooterIO;
import com.seiford.subsystems.shooter.ShooterIOSim;
import com.seiford.subsystems.shooter.ShooterIOSparkMax;
import com.seiford.subsystems.vision.Vision;
import com.seiford.subsystems.vision.VisionIO;
import com.seiford.subsystems.vision.VisionIOPhotonVision;
import com.seiford.subsystems.vision.VisionIOSim;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Arm arm;
  private final Shooter shooter;
  private final Intake intake;
  private final Climber climber;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Configuration.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(
            new GyroIONavX(),
            new ModuleIOMAXSwerve(0),
            new ModuleIOMAXSwerve(1),
            new ModuleIOMAXSwerve(2),
            new ModuleIOMAXSwerve(3));
        vision = new Vision(new VisionIOPhotonVision(), drive::addVisionMeasurement);
        arm = new Arm(new ArmIOSparkMax());
        shooter = new Shooter(new ShooterIOSparkMax());
        intake = new Intake(new IntakeIOSparkMax());
        climber = new Climber(new ClimberIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());
        vision = new Vision(new VisionIOSim(drive::getPose), drive::addVisionMeasurement);
        arm = new Arm(new ArmIOSim());
        shooter = new Shooter(new ShooterIOSim());
        intake = new Intake(new IntakeIOSim());
        climber = new Climber(new ClimberIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });
        vision = new Vision(new VisionIO() {
        }, drive::addVisionMeasurement);
        arm = new Arm(new ArmIO() {
        });
        shooter = new Shooter(new ShooterIO() {
        });
        intake = new Intake(new IntakeIO() {
        });
        climber = new Climber(new ClimberIO() {
        });
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommands(Map.of(
        "Shoot", shooter.speaker(),
        "Intake", intake.autoIntake()));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser("7 Center Close3 Top3"));

    // Set up SysId routines
    autoChooser.addOption("Drive SysId (QF)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (QR)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Drive SysId (DF)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (DR)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Shooter SysId (QF)", shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Shooter SysId (QR)", shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Shooter SysId (DF)", shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Shooter SysId (DR)", shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(drive.joystickDrive(
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX()));

    controller.leftStick().toggleOnTrue(drive.turtle());
    controller.rightStick().onTrue(drive.zeroHeading());

    controller.leftTrigger().onTrue(Commands.sequence(
        arm.amp(),
        shooter.amp(),
        arm.intake(),
        shooter.eject(),
        arm.speaker(),
        shooter.speaker(),
        arm.intake(),
        shooter.eject()));

    controller.rightBumper()
        .toggleOnTrue(drive.orbitSpeaker(() -> -controller.getLeftY(), () -> -controller.getLeftX()));
    controller.leftBumper().toggleOnTrue(drive.orbitAmp(() -> -controller.getLeftY(), () -> -controller.getLeftX()));

    controller.y().whileTrue(drive.pathfindSource());
    controller.x().whileTrue(drive.pathfindAmp());
    controller.b().whileTrue(drive.pathfindSpeaker());
    controller.a().whileTrue(drive.pathfindStage());

    controller.povUp().onTrue(climber.climb()).onFalse(climber.stop());
    controller.povDown().onTrue(climber.reverse()).onFalse(climber.stop());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
