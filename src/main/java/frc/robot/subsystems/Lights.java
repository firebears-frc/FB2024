package frc.robot.subsystems;

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

    private void setColor(Color color){
        for (var i = 0; i < light_stripBuffer.getLength(); i++) {
            light_stripBuffer.setLED(i, color);;
        }
        light_strip.setData(light_stripBuffer);
    }

    public void setDefault() {
        if(isRed){
            setColor(Color.kRed);
        }else{
            setColor(Color.kBlue);;
        }
    }

    @Override
    public void periodic() {
        if(sensor.get()){
            if(!e){
                setColor(Color.kOrange);
                e = true;
            }
        }else if(e){
            setDefault();
            e = false;
        }
    }
}