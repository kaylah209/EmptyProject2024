// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
import org.carlmontrobotics.subsystems.Shooter;
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private TimeOfFlight distSensor = new TimeOfFlight(10);
  private double DSdepth = 9.97;
  private double DSdetectdistance = 23;
  Shooter shooter = new Shooter();
 
  


  //feedforard stuff



  // Calculates the feedforward for a velocity of 10 units/second and an 
  // acceleration of 20 units/second^2
  // Units are determined by the units of the gains passed in at construction.
 

  // drives the motor to the desired velocity and acceleration calculated 
  // from the feedforward controller


  public boolean hasGamePiece() {
    // return false;
    return getGamePieceDistanceIn() < DSdetectdistance;
  }

  public double getGamePieceDistanceIn() {
    return Units.metersToInches((distSensor.getRange() - DSdepth) / 1000 /* Convert mm to m */);
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

  }

  @Override
  
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Detect Distance", getGamePieceDistanceIn());
    SmartDashboard.putBoolean("Has GUM", hasGamePiece());

    
    
    
   
    
    
    
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();

    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
