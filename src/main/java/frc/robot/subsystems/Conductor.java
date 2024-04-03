// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Conductor extends SubsystemBase {

    private static final class Constants {
        public static final Translation2d BLUE_SPEAKER = new Translation2d(0.0, 5.550);
        public static final Translation2d RED_SPEAKER = new Translation2d(16.541, 5.550);
    }

    private static final boolean isRedAlliance = true;

    private final InterpolatingDoubleTreeMap angleInterpolator = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap speedInterpolator = new InterpolatingDoubleTreeMap();
    private final Supplier<Pose2d> poseSupplier;

    @AutoLogOutput(key = "Conductor/SpeakerTranslation")
    private Translation2d speakerTranslation;

    public Conductor(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;

        put(1.00, 14.50, 3600.00); // tuned subwoofer shot
        put(3.40, 30.00, 4800.00); // not tuned podium shot
        put(4.25, 33.75, 5000.00); // partially tuned far side shot
        put(6.25, 35.00, 5000.00); // not tuned far shot
    }

    private void put(double distance, double angle, double speed) {
        angleInterpolator.put(distance, angle);
        speedInterpolator.put(distance, speed);
    }

    @AutoLogOutput(key = "Conductor/SpeakerDistance")
    private double getSpeakerDistance() {
        return Math.abs(speakerTranslation.getNorm());
    }

    @AutoLogOutput(key = "Conductor/SpeakerAngle")
    public Rotation2d getSpeakerAngle() {
        return speakerTranslation.getAngle();
    }

    @AutoLogOutput(key = "Conductor/ArmAngle")
    public Rotation2d getArmAngle() {
        return Rotation2d.fromDegrees(angleInterpolator.get(getSpeakerDistance()));
    }

    @AutoLogOutput(key = "Conductor/ShooterRPM")
    public double getShooterRPM() {
        return speedInterpolator.get(getSpeakerDistance());
    }

    @AutoLogOutput(key = "Conductor/SpeakerPosition")
    private Translation2d getSpeakerPosition() {
        if(isRedAlliance){
            return Constants.RED_SPEAKER;
        } else {
            return Constants.BLUE_SPEAKER;
        }
        //return Util.isRedAlliance() ? Constants.RED_SPEAKER : Constants.BLUE_SPEAKER;
    }

    @Override
    public void periodic() {
        speakerTranslation = poseSupplier.get().getTranslation().minus(getSpeakerPosition());
    }

}