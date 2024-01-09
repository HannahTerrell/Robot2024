package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
    private Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    // constructor. it runs when "new" is called
    public Pneumatics()
    {
        m_compressor.disable();
    }

    public void start()
    {
        m_compressor.enableDigital();
    }

    public void stop()
    {
        m_compressor.disable();
    }
}
