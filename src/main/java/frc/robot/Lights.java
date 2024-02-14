package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase{
    private AddressableLED light_strip;
    private AddressableLEDBuffer light_stripBuffer;
    public void robotInit() {
        light_strip = new AddressableLED(9);
        light_stripBuffer = new AddressableLEDBuffer(50);
        light_strip.setLength(light_stripBuffer.getLength());
        light_strip.setData(light_stripBuffer);
        light_strip.start();
        for (var i = 0; i < light_stripBuffer.getLength(); i++) {
            light_stripBuffer.setHSV(i, 1, 225, 135);

        }
    }
    
}

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             

