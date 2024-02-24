import static org.mockito.Mockito.validateMockitoUsage;

public class Arm {
    public CANSparkMax ArmMotor = MotorControllerFactory.createSparkMax(6, MotorConfig.NEO);
    
    private Joystick operatorJoystick = new Joystick (1);

    //unit conversions
    private final double kArmTick2Deg = 360.0 / 512 * 26 / 42 * 18 / 60 * 18 / 84;

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Arm Encoder Value", armMotor.getSelectedSensorPosition() * kArmTick2Deg);
        suoer.robotPeriodic();
    }
   @Override
    public void robotInit() {
        armMotor.setInverted(false);
        //encoder
        armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        //reset encoders to ZERO

        drive.setDeadband(0.05);
    }
   @Override
   public void robotPeriodic() {
     CommandScheduler.getInstance().run();
   }
 
   @Override
   public void autonomousInit() {
     m_autonomousCommand = m_robotContainer.getAutonomousCommand();
     
     armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
 
     if (m_autonomousCommand != null) {
       m_autonomousCommand.schedule();
     }
   }
 
   @Override
   public void autonomousPeriodic() {
      double leftPosition1 = leftMotor1.getSelectedSensorPosition() * kDriveTick2Feet;
      double leftPosition2 = leftMotor2.getSelectedSensorPosition() * kDriveTick2Feet;
      double rightPosition1 = rightMotor1.getSelectedSensorPosition() * kDriveTick2Feet;
      double rightPosition2 = rightMotor2.getSelectedSensorPosition() * kDriveTick2Feet;

      double distance = (leftPosition1 + leftPosition2 + rightPosition1 + rightPosition2)/4;

      if (distance < 10){
        drive.tankDrive(0.6, 0.6);
      }else {
        drive.tankDrive(0, 0);
      }
   }

   @Override
   public void teleopInit() {
     if (m_autonomousCommand != null) {
       m_autonomousCommand.cancel();
     }
   }
 
   @Override
   public void teleopPeriodic() {
    
    double armPower = - operatorJoystick.getRawAxis(1); 
   }
   armPower *= 0.5;
   armMotor.set(ControlMode.PercentOutput, armPower);
 
   @Override
   public void disabledInit() {}
 
   @Override
   public void disabledPeriodic() {}
 
   @Override
   public void testInit() {
     CommandScheduler.getInstance().cancelAll();
   }
 
   @Override
   public void testPeriodic() {}
 
   @Override
   public void simulationInit() {}
 
   @Override
   public void simulationPeriodic() {}
}

