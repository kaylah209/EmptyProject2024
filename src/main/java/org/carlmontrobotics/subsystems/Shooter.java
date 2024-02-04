package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Shooter extends SubsystemBase {
    CANSparkMax motor = MotorControllerFactory.createSparkMax(0, MotorConfig.NEO_550);

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


    

        



    public Shooter(){

    }
    
}
