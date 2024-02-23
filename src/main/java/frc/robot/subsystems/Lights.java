package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Lights extends SubsystemBase {
    private AddressableLED light_strip;
    private AddressableLEDBuffer light_stripBuffer;

    private boolean isRed;
    private boolean e = false;

    private DigitalInput sensor;

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

        sensor = new DigitalInput(0);

        Logger.recordOutput("Lights/isRedAlliance", isRed);
    }

    public void setDefault() {
        if(isRed){
            setRed();
        }else{
            setBlue();
        }
    }

    private void setRed(){
        for (var i = 0; i < light_stripBuffer.getLength(); i++) {
            light_stripBuffer.setRGB(i, 255, 0, 0);
        }
        light_strip.setData(light_stripBuffer);
    }

    private void setBlue(){
        for (var i = 0; i < light_stripBuffer.getLength(); i++) {
            light_stripBuffer.setRGB(i, 0, 0, 255);
        }
        light_strip.setData(light_stripBuffer);
    }

    private void setOrange(){
        for (var i = 0; i < light_stripBuffer.getLength(); i++) {
            light_stripBuffer.setRGB(i, 229, 83, 0);
        }
        light_strip.setData(light_stripBuffer);
    }

    @Override
    public void periodic() {
        if(sensor.get()){
            if(!e){
                setOrange();
                e = true;
            }
        }else if(e){
            setDefault();
            e = false;
        }
    }
}