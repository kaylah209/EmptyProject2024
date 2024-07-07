package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.Constants.SHOOTER;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Shooter extends SubsystemBase {
    CANSparkBase motor = new CANSparkFlex(10, MotorType.kBrushless);
    //SparkPIDController pidController = motor.getPIDController();
    private final MutableMeasure<Voltage> voltage = mutable(Volts.of(0));
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));
    private final MutableMeasure<Angle> distance = mutable(Rotations.of(0));
    //SimpleMotorFeedforward ff = new SimpleMotorFeedforward(SHOOTER.kS, SHOOTER.kV, SHOOTER.kA);
    double kP = 0.0001;
    double kD = 0;
    double kI = 0;
    double kIZone = 0;
    RelativeEncoder encoder = motor.getEncoder();
    private ShuffleboardTab sysIDTab  = Shuffleboard.getTab("Sysid");
    public Shooter() {
        // pidController.setP(SHOOTER.kP);
        // pidController.setD(SHOOTER.kD);
        // pidController.setI(kI);
        // SmartDashboard.putNumber("Shooter RPM", 0);
        
        // SmartDashboard.putNumber("kP", kP);
        // SmartDashboard.putNumber("kI", kI);
        // // SmartDashboard.putNumber("kIZone", kIZone);
        // SmartDashboard.putNumber("kD", kD);
        // SmartDashboard.putNumber("Feedforward", 0);
        // SmartDashboard.putNumber("plswork", 0);
        // THIS LINE BELOW DOESN'T WORK
        // encoder.setVelocityConversionFactor(1/9999999);
        sysIDTab.add("quasistatic forward", sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        sysIDTab.add("quasistatic backward", sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        sysIDTab.add("dynamic foward", sysIdDynamic(SysIdRoutine.Direction.kForward));
        sysIDTab.add("dynamic backward", sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    public void driveMotor(Measure<Voltage> volts) {
        motor.setVoltage(volts.in(Volts));
    }

    public void logMotor(SysIdRoutineLog log) {
        log.motor("shooter-motor")
                .voltage(voltage.mut_replace(
                        motor.getBusVoltage() * motor.getAppliedOutput(),
                        Volts))
                .angularVelocity(velocity.mut_replace(
                        encoder.getVelocity() / 60,
                        RotationsPerSecond))
                .angularPosition(distance.mut_replace(
                        encoder.getPosition(),
                        Rotations));
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

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Motor velocity", encoder.getVelocity());
        // double targetRPS = SmartDashboard.getNumber("Shooter RPM", 0)/60;
        // kP = SmartDashboard.getNumber("kP", kP);
        // kD = SmartDashboard.getNumber("kD", kD);
        // kI = SmartDashboard.getNumber("kI", kI);

        // if (pidController.getP() != kP) {
        //     pidController.setP(kP);
        // }
        // if (pidController.getD() != kD) {
        //     pidController.setD(kD);
        // }
        // if (pidController.getI() != kI) {
        //     pidController.setI(kI);
        // }

        // SmartDashboard.putNumber("Shooter current RPS", encoder.getVelocity() / 60);
        // double feed = ff.calculate(targetRPS);
        // SmartDashboard.putNumber("Feedforward", feed);

        // // SmartDashboard.putNumber("Error", motor.getBusVoltage() *
        // // motor.getAppliedOutput() - feed);
        // // SmartDashboard.putNumber("Motor Voltage", motor.getBusVoltage() *
        // // motor.getAppliedOutput());

        // // motor.setVoltage(feed);
        // pidController.setReference(targetRPS*60, CANSparkBase.ControlType.kVelocity, 0, feed);
    }

}
