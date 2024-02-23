public class Drivetrain {
   
   public CANSparkMax rightMotor1 = MotorControllerFactory.createSparkMax(0, MotorConfig.NEO);
   public CANSparkMax rightMotor2 = MotorControllerFactory.createSparkMax(1, MotorConfig.NEO);

   public CANSparkMax leftMotor1 = MotorControllerFactory.createSparkMax(2, MotorConfig.NEO);
   public CANSparkMax leftMotor2 = MotorControllerFactory.createSparkMax(3, MotorConfig.NEO);

   private Joystick driverJoy1 = new Joystick(0);
   private double startTime;
   
   private final double kDriveTick2Feet = 1.0/ 4096 * 6 * Math.PI / 12;
    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Left Drive Encoder Value", leftMotor.getSelectedSensorPosition() * );
    }
    @Override
    public void robotInit() {
        rightMotor1.setInverted(true);
        rightMotor2.setInverted(true);

        leftMotor1.setInverted(true);
        leftMotor2.setInverted(true);
        //encoder
        rightMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        rightMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        leftMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        leftMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        //reset encoders to ZERO
        rightMotor1.setSelectedSensorPosition(0, 0, 10);
        rightMotor2.setSelectedSensorPosition(0, 0, 10);
        leftMotor1.setSelectedSensorPosition(0, 0, 10);
        leftMotor2.setSelectedSensorPosition(0, 0, 10);
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

    double time = Timer.getFPGATimestamp(); 
    if (time - startTime < 3){
     rightMotor1.setSpeed(0.6);
     rightMotor2.setSpeed(0.6);
     leftMotor1.setSpeed(-0.6);
     leftMotor2.setSpeed(-0.6);
    } else {
        rightMotor1.setSpeed(0);
        rightMotor2.setSpeed(0);
        leftMotor1.setSpeed(-0);
        leftMotor2.setSpeed(-0);

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
    double speed = -driverJoy1.getRawAxis(0) * 0.6;
    double turn = driverJoy1.getRawAxis(1) * 0.3;

    double left = speed + turn;
    double right = speed - turn;

     rightMotor1.setSpeed(left);
     rightMotor2.setSpeed(left);
     leftMotor1.setSpeed(-right);
     leftMotor2.setSpeed(-right);
 
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
