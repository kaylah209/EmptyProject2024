package subystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
    private final CANSparkMax armMotor = MotorControllerFactory.createSparkMax(0, MotorConfig.NEO);
    private SparkPIDController armPid = armMotor.getPIDController();
    private SparkAbsoluteEncoder encoder = armMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    private final double kdt = 0.2;
    public static final double UPPER_ANGLE_LIMIT_RAD = 1.63;
    public static final double LOWER_ANGLE_LIMIT_RAD = -0.5;
    public static final double ARM_DISCONT_RAD = (LOWER_ANGLE_LIMIT_RAD + UPPER_ANGLE_LIMIT_RAD) / 2 - Math.PI;
    private TrapezoidProfile.State currentState = getCurrentArmState();
    private TrapezoidProfile armProfile;
    private TrapezoidProfile.State goalState = new TrapezoidProfile.State(0, 0);
    
    private final ArmFeedforward armFeed = new ArmFeedforward(0.1, 0.1, 0.1);
    public Arm() {
        encoder.setPositionConversionFactor(2 * Math.PI);
        armPid.setP(0.1);
        armPid.setI(0.1);
        armPid.setD(0.1);
    }

    public double getArmPos() {
        return MathUtil.inputModulus(encoder.getPosition(), ARM_DISCONT_RAD,
                ARM_DISCONT_RAD + 2 * Math.PI);
    }

    public double getArmVel() {
        return encoder.getVelocity();
    }


    public TrapezoidProfile.State getCurrentArmState() {
        return new TrapezoidProfile.State(getArmPos(), getArmVel());
    }

    public void moveArm() {
        currentState = getCurrentArmState();
        TrapezoidProfile.State currentGoal = armProfile.calculate(kdt, currentState, goalState);
        double feedForwardVolts = armFeed.calculate(currentGoal.position, currentGoal.velocity);
        armPid.setReference(currentGoal.position, CANSparkBase.ControlType.kPosition, 0, feedForwardVolts);
    }

    @Override
    public void periodic() {
        moveArm();
    }
}
