package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private AddressableLED m_leds = new AddressableLED(0);
    private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(100);

    public LEDs() {
        m_leds.setLength(m_ledBuffer.getLength());
        m_leds.setData(m_ledBuffer);
        m_leds.start();
    }

    public void setRed() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 255, 0, 0);
         }
         m_leds.setData(m_ledBuffer);
    }

    public void setGreen() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 255, 0);
         }
         m_leds.setData(m_ledBuffer);
    }

    public void setBlue() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 255);
         }
         m_leds.setData(m_ledBuffer);
    }

    @Override
    public void periodic() {
        m_leds.setData(m_ledBuffer);
    }

}
