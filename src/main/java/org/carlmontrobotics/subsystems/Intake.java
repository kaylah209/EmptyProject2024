package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.*;
import org.carlmontrobotics.Robot;
import org.carlmontrobotics.RobotContainer;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
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

public class Intake extends SubsystemBase {
    CANSparkMax motor = MotorControllerFactory.createSparkMax(motorPort, MotorConfig.NEO); //double check motor port
    SparkPIDController pid = motor.getPIDController();
    RelativeEncoder motorEncoder = motor.getEncoder();

    private final MutableMeasure<Voltage> voltage = mutable(Volts.of(0));
    private final MutableMeasure<Velocity<Angle>> angularVel = mutable(RotationsPerSecond.of(0));
    private final MutableMeasure<Angle> distance = mutable(Rotations.of(0));
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA); //double check constants | PLEASE TUNE IT TOMORROW TO FULLY TEST IT
    private TimeOfFlight distanceSensor = new TimeOfFlight(10); //make sure id port is correct here
    private double dsDepth = 9.97;
    private double detectDistance = 13;
    private boolean hasGamePiece = false;
    public Intake() {
       // pid.setP(/*get from sysid */);
       // pid.setD(/*get from sysid */);
        
        SmartDashboard.putNumber("Target RPM", 0);

        // kinda no works. | motor.getEncoder().setPositionConversionFactor(2 * Math.PI * 2); //This will be different since the wheel diamters will not be the same. THIS IS FOR DISTANCE
        pid.setP(kP);
        pid.setD(kD);
        SmartDashboard.putNumber("motorSpeed", 0);
    }
    public void driveMotor(Measure<Voltage> volts) {
        motor.setVoltage(volts.in(Volts));
    }
    public double gamePieceDistance() {
        return Units.metersToInches((distanceSensor.getRange() - dsDepth) /1000);
    }
    public boolean hasGamePiece() {
        return gamePieceDistance() < detectDistance;
    }
    public void periodic() {
        if(!hasGamePiece) {
            motor.set(SmartDashboard.getNumber("motorSpeed", 0));
        } else {
            pid.setReference(0, CANSparkBase.ControlType.kVelocity, 0, feedforward.calculate(motorEncoder.getVelocity()));
        }
    }

    //Ahead are Sysid tests

    public void logMotor(SysIdRoutineLog log) {
        log.motor("intake-motor").voltage(voltage.mut_replace(
                motor.get() * RobotController.getBatteryVoltage(),
                Volts)).angularVelocity(angularVel.mut_replace((motorEncoder.getVelocity()/60),
                        RotationsPerSecond))
                .angularPosition(distance.mut_replace((motor.getEncoder().getPosition()), Rotations));
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
