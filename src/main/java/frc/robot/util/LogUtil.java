package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LogUtil {
    public static void logPose(String key, Pose2d pose){
        double[] array = new double[3];
        array[0] = pose.getX();
        array[1] = pose.getY();
        array[2] = pose.getRotation().getRadians();

        SmartDashboard.putNumberArray(key, array);

    }
}
