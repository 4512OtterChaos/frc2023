package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveDriveAccelLimiter {
    double linearAcceleration;
    double linearDeceleration;
    double rotationalAcceleration;
    double rotationalDeceleration;

    public SwerveDriveAccelLimiter(double linearAcceleration, double linearDeceleration, double rotationalAcceleration, double rotationalDeceleration){
        this.linearAcceleration = linearAcceleration;
        this.linearDeceleration = linearDeceleration;
        this.rotationalAcceleration = rotationalAcceleration;
        this.rotationalDeceleration = rotationalDeceleration;
    }

    public ChassisSpeeds calculate(ChassisSpeeds targetSpeeds, ChassisSpeeds currentSpeeds, double dt){
        double currentXVelocity = currentSpeeds.vxMetersPerSecond;
        double currentYVelocity = currentSpeeds.vyMetersPerSecond;
        double currentRotationVelocity = currentSpeeds.vxMetersPerSecond;
        double targetXAccel = (targetSpeeds.vxMetersPerSecond - currentSpeeds.vxMetersPerSecond)/dt;
        double targetYAccel = (targetSpeeds.vyMetersPerSecond - currentYVelocity)/dt;
        Rotation2d velHeading = new Rotation2d(targetXAccel, targetYAccel);    
        double cosVelHeading = Math.abs(Math.cos(velHeading.getRadians()));
        double sinVelHeading = Math.abs(Math.sin(velHeading.getRadians()));
        double targetRotationalAccel = targetSpeeds.omegaRadiansPerSecond - currentRotationVelocity;
        if (Math.signum(targetXAccel)>0){
            targetXAccel = MathUtil.clamp(targetXAccel, -linearDeceleration*cosVelHeading, linearAcceleration*cosVelHeading);
        }
        else{
            targetXAccel = MathUtil.clamp(targetXAccel, -linearAcceleration*cosVelHeading, linearDeceleration*cosVelHeading);
        }
        if (Math.signum(targetYAccel)>0){
            targetYAccel = MathUtil.clamp(targetYAccel, -linearDeceleration*sinVelHeading, linearAcceleration*sinVelHeading);
        }
        else{
            targetYAccel = MathUtil.clamp(targetYAccel, -linearAcceleration*sinVelHeading, linearDeceleration*sinVelHeading);
        }
        if (Math.signum(targetRotationalAccel)>0){
            targetRotationalAccel = MathUtil.clamp(targetRotationalAccel, -rotationalDeceleration, rotationalAcceleration);
        }
        else{
            targetRotationalAccel = MathUtil.clamp(targetRotationalAccel, -rotationalAcceleration, rotationalDeceleration);
        }
        return new ChassisSpeeds(currentXVelocity+(targetXAccel*dt), currentYVelocity+(targetXAccel*dt), currentRotationVelocity+(targetRotationalAccel)*dt);
    }
}


// public ChassisSpeeds calculate(ChassisSpeeds targetSpeeds, ChassisSpeeds currentSpeeds, double dT){
//     double currentXVelocity = currentSpeeds.vxMetersPerSecond;
//     double currentYVelocity = currentSpeeds.vyMetersPerSecond;
//     double currentRotationVelocity = currentSpeeds.vxMetersPerSecond;
//     double targetXAccel = (targetSpeeds.vxMetersPerSecond - currentSpeeds.vxMetersPerSecond)/dT;
//     double targetYAccel = (targetSpeeds.vyMetersPerSecond - currentYVelocity)/dT;
//     double targetRotationalAccel = targetSpeeds.omegaRadiansPerSecond - currentRotationVelocity;
//     if (Math.signum(targetXAccel)>0){
//         targetXAccel = MathUtil.clamp(targetXAccel, -linearDeceleration, linearAcceleration);
//     }
//     else{
//         targetXAccel = MathUtil.clamp(targetXAccel, -linearAcceleration, linearDeceleration);
//     }
//     if (Math.signum(targetYAccel)>0){
//         targetYAccel = MathUtil.clamp(targetYAccel, -linearDeceleration, linearAcceleration);
//     }
//     else{
//         targetYAccel = MathUtil.clamp(targetYAccel, -linearAcceleration, linearDeceleration);
//     }
//     if (Math.signum(targetRotationalAccel)>0){
//         targetRotationalAccel = MathUtil.clamp(targetRotationalAccel, -rotationalDeceleration, rotationalAcceleration);
//     }
//     else{
//         targetRotationalAccel = MathUtil.clamp(targetRotationalAccel, -rotationalAcceleration, rotationalDeceleration);
//     }
//     return new ChassisSpeeds(currentXVelocity+(targetXAccel*dT), currentYVelocity+(targetYAccel*dT), currentRotationVelocity+(targetRotationalAccel*dT));
// }
