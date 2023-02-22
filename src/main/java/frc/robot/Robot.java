// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
* The VM is configured to automatically run this class, and to call the functions corresponding to
* each mode, as described in the TimedRobot documentation. If you change the name of this class or
* the package after creating this project, you must also update the build.gradle file in the
* project.
*/
public class Robot extends TimedRobot {

    private RobotContainer container;

    @Override
    public void robotInit() {
        container = new RobotContainer();
    }
    
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        container.periodic();
    }
    
    
    @Override
    public void autonomousInit() {
        container.getAutoCommand().schedule();
    }
    
    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }
    
    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {}
    
    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}
    
    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {}
    
    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}
    
    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {}
    
    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
    
    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}
    
    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        container.simulationPeriodic();
    }
}
