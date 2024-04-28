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
import com.seiford.subsystems.conductor.Conductor;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final List<Vision> visions;
  private final Conductor conductor;
  private final Arm arm;
  private final Shooter shooter;
  private final Intake intake;
  private final Climber climber;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Command> shootChooser;
  private final LoggedDashboardChooser<Command> climbChooser;
  private final LoggedDashboardBoolean armMode;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Configuration.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOMAXSwerve(0),
                new ModuleIOMAXSwerve(1),
                new ModuleIOMAXSwerve(2),
                new ModuleIOMAXSwerve(3));
        visions =
            List.of(
                new Vision(
                    new VisionIOPhotonVision("Front"),
                    drive::addVisionMeasurement,
                    Vision.Constants.FRONT_CAMERA_OFFSET,
                    "Front"),
                new Vision(
                    new VisionIOPhotonVision("RearLeft"),
                    drive::addVisionMeasurement,
                    Vision.Constants.REAR_LEFT_CAMERA_OFFSET,
                    "RearLeft"),
                new Vision(
                    new VisionIOPhotonVision("RearLeft"),
                    drive::addVisionMeasurement,
                    Vision.Constants.REAR_RIGHT_CAMERA_OFFSET,
                    "RearRight"));
        conductor = new Conductor(drive::getPose);
        arm = new Arm(new ArmIOSparkMax(), conductor::getArmAngle);
        shooter = new Shooter(new ShooterIOSparkMax(), conductor::getShooterRPM);
        intake = new Intake(new IntakeIOSparkMax());
        climber = new Climber(new ClimberIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        visions =
            List.of(
                new Vision(
                    new VisionIOSim(
                        drive::getPose,
                        Vision.Constants.FRONT_CAMERA_OFFSET,
                        "Front",
                        VisionIOSim.arducamOV2311(Rotation2d.fromDegrees(75))),
                    drive::addVisionMeasurement,
                    Vision.Constants.FRONT_CAMERA_OFFSET,
                    "Front"),
                new Vision(
                    new VisionIOSim(
                        drive::getPose,
                        Vision.Constants.REAR_LEFT_CAMERA_OFFSET,
                        "RearLeft",
                        VisionIOSim.arducamOV2311(Rotation2d.fromDegrees(75))),
                    drive::addVisionMeasurement,
                    Vision.Constants.REAR_LEFT_CAMERA_OFFSET,
                    "RearLeft"),
                new Vision(
                    new VisionIOSim(
                        drive::getPose,
                        Vision.Constants.REAR_RIGHT_CAMERA_OFFSET,
                        "RearRight",
                        VisionIOSim.arducamOV2311(Rotation2d.fromDegrees(75))),
                    drive::addVisionMeasurement,
                    Vision.Constants.REAR_RIGHT_CAMERA_OFFSET,
                    "RearRight"));
        conductor = new Conductor(drive::getPose);
        arm = new Arm(new ArmIOSim(), conductor::getArmAngle);
        shooter = new Shooter(new ShooterIOSim(), conductor::getShooterRPM);
        intake = new Intake(new IntakeIOSim());
        climber = new Climber(new ClimberIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        visions =
            List.of(
                new Vision(
                    new VisionIO() {},
                    drive::addVisionMeasurement,
                    Vision.Constants.FRONT_CAMERA_OFFSET,
                    "Front"),
                new Vision(
                    new VisionIO() {},
                    drive::addVisionMeasurement,
                    Vision.Constants.REAR_LEFT_CAMERA_OFFSET,
                    "RearLeft"),
                new Vision(
                    new VisionIO() {},
                    drive::addVisionMeasurement,
                    Vision.Constants.REAR_RIGHT_CAMERA_OFFSET,
                    "RearRight"));
        conductor = new Conductor(drive::getPose);
        arm = new Arm(new ArmIO() {}, conductor::getArmAngle);
        shooter = new Shooter(new ShooterIO() {}, conductor::getShooterRPM);
        intake = new Intake(new IntakeIO() {});
        climber = new Climber(new ClimberIO() {});
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommands(
        Map.of(
            "PrepareShoot", Commands.parallel(arm.speaker(), shooter.speaker()),
            "Shoot", Commands.sequence(intake.shoot(), Commands.waitSeconds(0.35)),
            "Intake", Commands.parallel(arm.intake(), shooter.stop(), intake.autoIntake())));
    autoChooser =
        new LoggedDashboardChooser<>(
            "Auto Modes", AutoBuilder.buildAutoChooser("7 Center Close3 Top3"));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (QF)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (QR)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Drive SysId (DF)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (DR)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Arm SysId (QF)", arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Arm SysId (QR)", arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Arm SysId (DF)", arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Arm SysId (DR)", arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Shooter SysId (QF)", shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Shooter SysId (QR)", shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Shooter SysId (DF)", shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Shooter SysId (DR)", shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Intake SysId (QF)", intake.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Intake SysId (QR)", intake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Intake SysId (DF)", intake.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Intake SysId (DR)", intake.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    shootChooser = new LoggedDashboardChooser<>("Shoot Modes");
    shootChooser.addDefaultOption("Subwoofer Center", drive.pathfindSubwooferCenter());
    shootChooser.addOption("Subwoofer Left", drive.pathfindSubwooferLeft());
    shootChooser.addOption("Subwoofer Right", drive.pathfindSubwooferRight());
    shootChooser.addOption("Top", drive.pathfindTopShooting());
    shootChooser.addOption("Bottom", drive.pathfindBottomShooting());
    shootChooser.addOption("Far", drive.pathfindFarShooting());
    shootChooser.addOption("Pass", drive.pathfindPassShooting());
    shootChooser.addOption("Podium", drive.pathfindPodium());

    climbChooser = new LoggedDashboardChooser<>("Climb Modes");
    climbChooser.addDefaultOption("Far", drive.pathfindStageFar());
    climbChooser.addOption("Left", drive.pathfindStageLeft());
    climbChooser.addOption("Right", drive.pathfindStageRight());

    armMode = new LoggedDashboardBoolean("Stow Arm", false);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Universal commands
    controller.leftStick().toggleOnTrue(drive.turtle());
    controller.rightStick().onTrue(drive.zeroHeading());

    controller.y().whileTrue(Commands.deferredProxy(climbChooser::get));
    controller.povUp().onTrue(climber.climb()).onFalse(climber.stop());
    controller.povDown().onTrue(climber.reverse()).onFalse(climber.stop());

    // Full manual commands
    drive.setDefaultCommand(
        drive.joystickDrive(
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller
        .leftTrigger()
        .onTrue(Commands.parallel(arm.amp(), shooter.amp()))
        .onFalse(
            Commands.sequence(
                intake.intake(),
                Commands.waitSeconds(1.0),
                shooter.stop(),
                intake.stop(),
                Commands.either(
                    arm.stow(), Commands.sequence(arm.intake(), intake.intake()), armMode::get)));
    controller
        .rightTrigger()
        .onTrue(Commands.parallel(arm.speaker(), shooter.speaker()))
        .onFalse(
            Commands.sequence(
                intake.shoot(),
                Commands.waitSeconds(0.35),
                shooter.stop(),
                intake.stop(),
                Commands.either(
                    arm.stow(), Commands.sequence(arm.intake(), intake.intake()), armMode::get)));

    // Orbit commands
    controller
        .leftBumper()
        .whileTrue(
            Commands.parallel(
                drive.orbitAmp(() -> -controller.getLeftY(), () -> -controller.getLeftX()),
                arm.amp(),
                shooter.amp()))
        .onFalse(
            Commands.sequence(
                intake.intake(), Commands.waitSeconds(0.35), shooter.stop(), intake.stop()));
    controller
        .rightBumper()
        .whileTrue(
            Commands.parallel(
                drive.orbitSpeaker(() -> -controller.getLeftY(), () -> -controller.getLeftX()),
                arm.speaker(),
                shooter.speaker()))
        .onFalse(
            Commands.sequence(
                intake.shoot(), Commands.waitSeconds(0.35), shooter.stop(), intake.stop()));

    // Full auto commands
    controller
        .x()
        .whileTrue(Commands.parallel(drive.pathfindAmp(), arm.amp(), shooter.amp()))
        .onFalse(
            Commands.sequence(
                intake.intake(), Commands.waitSeconds(0.35), shooter.stop(), intake.stop()));
    controller
        .b()
        .whileTrue(
            Commands.parallel(
                Commands.deferredProxy(shootChooser::get), arm.speaker(), shooter.speaker()))
        .onFalse(
            Commands.sequence(
                intake.shoot(), Commands.waitSeconds(0.35), shooter.stop(), intake.stop()));
    controller
        .a()
        .whileTrue(
            Commands.either(
                Commands.sequence(
                    Commands.parallel(drive.pathfindSource(), shooter.stop(), arm.stow()),
                    intake.intake(),
                    arm.intake()),
                Commands.parallel(
                    drive.pathfindSource(), arm.intake(), shooter.stop(), intake.intake()),
                armMode::get))
        .onFalse(Commands.sequence(intake.stop()));
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
