package frc.robot;

import com.mindsensors.CANLight;

public class LEDs {
    // http://www.mindsensors.com/reference/FRC/html/Java/index.html?com/mindsensors/CANLight.html

    // factory default of 3. Can't get configuration tool.
    private static final CANLight m_light = new CANLight(3); 
    private static final int REGISTER_OFF = 0;
    private static final int REGISTER_RED = 1;
    private static final int REGISTER_RED_FLASHING = 2;
    // private static final int REGISTER_GREEN = 3;
    private static final int REGISTER_GREEN_FLASHING = 4;
    // private static final int REGISTER_TEAL = 5;
    // private static final int REGISTER_PURPLE = 6;
    // private static final int REGISTER_WHITE = 7;

    // State
    private static boolean hasNote = false;
    private static boolean intaking = false;
    private static boolean aiming = false;

    static {
        m_light.writeRegister(REGISTER_RED_FLASHING, .25, 1, 0, 0);
        m_light.writeRegister(REGISTER_GREEN_FLASHING, .25, 0, 1, 0);
    }

    public static void setHasNote(boolean value) {
        hasNote = value;
        evalLightState();
    }

    public static void setIntaking(boolean value) {
        intaking = value;
        evalLightState();
    }

    public static void setAiming(boolean value) {
        aiming = value;
        evalLightState();
    }

    public static void reset() {
        hasNote = false;
        intaking = false;
        aiming = false;
        evalLightState();
    }

    private static void evalLightState() {
        if (aiming) {
            m_light.showRegister(REGISTER_GREEN_FLASHING);
        } else if (intaking) {
            m_light.showRegister(REGISTER_RED_FLASHING);
        } else if (hasNote) {
            m_light.showRegister(REGISTER_RED);
        } else {
            m_light.showRegister(REGISTER_OFF);
        }
    }
}