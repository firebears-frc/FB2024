package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    private AddressableLED light_strip;
    private AddressableLEDBuffer light_stripBuffer;

    private boolean isRed;

    public Lights() {
        light_strip = new AddressableLED(9);
        light_stripBuffer = new AddressableLEDBuffer(50);
        light_strip.setLength(light_stripBuffer.getLength());
        light_strip.setData(light_stripBuffer);
        light_strip.start();

        // choose default color
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent() && (ally.get() == Alliance.Blue)) {
                isRed = false;
            }else{
                isRed = true;
            }

        setDefault();

        Logger.recordOutput("Lights/isRedAlliance", isRed);
    }

    private void setDefault() {
        if(isRed){
            setRed();
        }else{
            setBlue();
        }
    }

    public void setRed() {
        for (var i = 0; i < light_stripBuffer.getLength(); i++) {
            light_stripBuffer.setRGB(i, 255, 0, 0);
        }
        light_strip.setData(light_stripBuffer);
    }

    public void setBlue() {
        for (var i = 0; i < light_stripBuffer.getLength(); i++) {
            light_stripBuffer.setRGB(i, 0, 0, 255);
        }
        light_strip.setData(light_stripBuffer);
    }
}