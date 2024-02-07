package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.Robot;
import org.carlmontrobotics.RobotContainer;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Shooter extends SubsystemBase {

    public static final double[] pidVals = new double[] {/*/kP/*/ 0.1,/*/kI/*/ 0.0,/*/kD/*/ 0.1 };
    public static final double[] FeedforwardVals = new double[] { /*/kS/*/0.1, /*/kG/*/0.1, /*/kV/*/0.1, /*/kA/*/0.1 };
    
    public static double outtakeRPM = 0;

    CANSparkMax motor = MotorControllerFactory.createSparkMax(0, MotorConfig.NEO_550);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);
    //^^put in after test
    
    
    
    private double volts = 0;
    
    
    public void periodic(){
        SmartDashboard.putNumber("OuttakeRPM", outtakeRPM);
        double rpm = SmartDashboard.getNumber("OuttakeRPM",outtakeRPM);
        volts = feedforward.calculate(rpm, 0);
    }

    



    private final MutableMeasure<Voltage> voltage = mutable(Volts.of(0));

    public void driveMotor(Measure<Voltage> volts) {

        motor.setVoltage(volts.in(Volts));
        
    }



    public void logMotor(SysIdRoutineLog log) {
            log.motor("shooter-motor").voltage(voltage.mut_replace(
            motor.get() * RobotController.getBatteryVoltage(), 
            Volts
        ));
    }
    private final SysIdRoutine routine = 
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                this::driveMotor, 
                this::logMotor, 
                this
            )
        );
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
    public void shoot(){
        motor.setVoltage(volts);
    }


    

        



    public Shooter(){

    }
    
}
