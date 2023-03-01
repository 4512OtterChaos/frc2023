package frc.robot.util.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class PistonSim {
    
    private boolean extended;
    private double extension;
    private double extensionTimeSec;

    private double lastTime = -1;

    /**
     * 
     * @param extended Whether the piston is currently extending or retracting.
     * @param extensionTimeSec The time it takes for the piston to go from retracted to extended.
     */
    public PistonSim(boolean extended, double extensionTimeSec) {
        this.extended = extended;
        extension = extended ? 1 : 0;
        this.extensionTimeSec = extensionTimeSec;
    }
    
    public void setExtended(boolean extended) {
        this.extended = extended;
    }
    /**
     * Manually set the extension length (0 - 1).
     */
    public void setExtension(double extension) {
        this.extension = MathUtil.clamp(extension, 0, 1);
    }
    public void setExtensionTimeSec(double extensionTimeSec) {
        this.extensionTimeSec = extensionTimeSec;
    }

    /**
     * Updates the extension length. Call this periodically.
     */
    public void update() {
        double now = Timer.getFPGATimestamp();
        double delta = (now - lastTime) / extensionTimeSec;
        lastTime = now;
        if(!extended) delta = -delta;
        extension = MathUtil.clamp(extension + delta, 0, 1);
    }

    /**
     * Get the extension length of this piston (0 - 1).
     */
    public double getExtension() {
        return extension;
    }
}
