public class Arm {
    public CANSparkMax ArmMotor = MotorControllerFactory.createSparkMax(6, MotorConfig.NEO);
    
    private Joystick operatorJoystick = new Joystick (1);

    //unit conversions
    private final double kArmTick2Deg = 360.0 / 512 * 26 / 42 * 18 / 60 * 18 / 84;

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Arm Encoder Value", armMotor.getSelectedSensorPosition() * kArmTick2Deg);
    }
   @Override
    public void robotInit() {
        armMotor.setInverted(false);
        //encoder
        armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        //reset encoders to ZERO

    }
   @Override
   public void robotPeriodic() {
     CommandScheduler.getInstance().run();
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

