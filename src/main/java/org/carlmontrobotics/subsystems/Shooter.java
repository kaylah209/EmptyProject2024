package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.Robot;
import org.carlmontrobotics.RobotContainer;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Shooter extends SubsystemBase {
    CANSparkMax motor = MotorControllerFactory.createSparkMax(1, MotorConfig.NEO_550);

    private final MutableMeasure<Voltage> voltage = mutable(Volts.of(0));
    private final MutableMeasure<Velocity<Distance>> velocity = mutable(MetersPerSecond.of(0));
    private final MutableMeasure<Distance> distance = mutable(Meters.of(0));

    public Shooter() {
        motor.getEncoder().setPositionConversionFactor(2 * Math.PI * 2);
    }
    public void driveMotor(Measure<Voltage> volts) {

        motor.setVoltage(volts.in(Volts));

    }

    public void logMotor(SysIdRoutineLog log) {
        log.motor("shooter-motor").voltage(voltage.mut_replace(
                motor.get() * RobotController.getBatteryVoltage(),
                Volts)).linearVelocity(velocity.mut_replace(Units.inchesToMeters(motor.getEncoder().getVelocity()),
                        MetersPerSecond))
                .linearPosition(distance.mut_replace(Units.inchesToMeters(motor.getEncoder().getPosition()), Meters));
    }

    private final SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    this::driveMotor,
                    this::logMotor,
                    this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

}
