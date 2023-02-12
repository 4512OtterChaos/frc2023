package frc.robot.subsystems.camera;

import edu.wpi.first.math.util.Units;

public class cameraConstants {
    // Constants such as camera and target height stored. Change per robot and goal!
    public final static double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    public final static double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
    public final static double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    public final static double GOAL_RANGE_METERS = Units.feetToMeters(3);
}
