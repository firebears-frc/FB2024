// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DownBeat;
import frc.robot.subsystems.UpBeat;

public class RobotContainer {
  private final DownBeat m_intake = new DownBeat();
  private final UpBeat m_shooter = new UpBeat();
  private final Arm m_arm = new Arm();
  private final UsbCamera usbcamera;
  private final CommandJoystick one = new CommandJoystick(0); // right
  private final CommandJoystick two = new CommandJoystick(1); // left
  private final CommandXboxController xboxController = new CommandXboxController(2);

  public RobotContainer() {
    usbcamera = CameraServer.startAutomaticCapture();
    usbcamera.setResolution(320, 240);
    configureBindings();
  }

  private void configureBindings() {
    xboxController.a().onTrue(m_intake.intakeNote()).onFalse(m_intake.pauseDownBeat());
    xboxController.x().onTrue(m_intake.dischargeNote()).onFalse(m_intake.pauseDownBeat());
    xboxController.y().toggleOnTrue(m_shooter.shootNote());
    xboxController.b().onTrue(m_arm.pickUp());
    xboxController.rightBumper().onTrue(m_arm.speakerShoot());
    xboxController.leftBumper().onTrue(m_arm.ampShoot());

    xboxController
        .leftTrigger()
        .onTrue(Commands.parallel(m_arm.ampShoot(), m_shooter.ampSpeed()))
        .onFalse(
            Commands.sequence(
                m_intake.intakeNote(),
                Commands.waitSeconds(1),
                m_shooter.pauseUpBeat(),
                m_intake.pauseDownBeat(),
                m_arm.pickUp()));

    xboxController
        .rightTrigger()
        .onTrue(Commands.parallel(m_arm.speakerShoot(), m_shooter.autoShoot()))
        .onFalse(
            Commands.sequence(
                m_intake.shootNote(),
                Commands.waitSeconds(.35),
                m_shooter.pauseUpBeat(),
                m_intake.pauseDownBeat(),
                m_arm.pickUp()));

    m_arm.setDefaultCommand(
        m_arm.defaultCommand(() -> MathUtil.applyDeadband(xboxController.getLeftY(), 0.2)));
  }
}
