/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package frc.robot.subsystems.camera;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.camera.cameraConstants.*;
public class camera extends SubsystemBase{
    // Change this to match the name of your camera
    PhotonCamera camera = new PhotonCamera("photonvision");
    public void periodic() {}
        
    public void fjfujf(){
        double forwardSpeed;
        double rotationSpeed;
        
        
        // Vision-alignment mode
        // Query the latest result from PhotonVision
        var result = camera.getLatestResult();
    
        if (result.hasTargets()) {
            // First calculate range
            double range =
                    PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(result.getBestTarget().getPitch()));
    
            // // Use this range as the measurement we give to the PID controller.
            // // -1.0 required to ensure positive PID controller effort _increases_ range
            // forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);
    
            // // Also calculate angular power
            // // -1.0 required to ensure positive PID controller effort _increases_ yaw
            // rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
        } else {
            // If we have no targets, stay still.
            forwardSpeed = 0;
            rotationSpeed = 0;
        }
    }
}
