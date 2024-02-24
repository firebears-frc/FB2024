package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    private AddressableLED light_strip;
    private AddressableLEDBuffer light_stripBuffer;
    private Color currentColor;

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

    private void setColor(){
        for (var i = 0; i < light_stripBuffer.getLength(); i++) {
            light_stripBuffer.setLED(i, currentColor);;
        }
        light_strip.setData(light_stripBuffer);
    }

    public void setDefault() {
        if(isRed){
            currentColor = Color.kRed;
            setColor();
        }else{
            currentColor = Color.kBlue;
            setColor();
        }
    }

    @Override
    public void periodic() {
        if(sensor.get()){
            if(!e){
                currentColor = Color.kOrange;
                setColor();
                e = true;
            }
        }else if(e){
            setDefault();
            e = false;
        }
        Logger.recordOutput("Lights/color", currentColor.toString());
    }
}