package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Elevator extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    private final SparkMaxConfig leftMotorConfig;
    private final SparkMaxConfig rightMotorConfig;

    private final double kManualSpeed = 1;
    private final double kP = 5.0;
    private final double DEAD_BAND = 5;
    private final double MIN_OUTPUT = 0.3;
    private final double FEEDFORWARD = 2.0;

    private Double targetPosition = null;

    public Elevator() {
        leftMotor = new SparkMax(Constants.Elevator.LEFT_ELEVATOR_CAN_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.Elevator.RIGHT_ELEVATOR_CAN_ID, MotorType.kBrushless);

        leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        leftMotorConfig.inverted(false);
        leftMotorConfig.softLimit.reverseSoftLimit(10).reverseSoftLimitEnabled(true);
        leftMotorConfig.softLimit.forwardSoftLimit(360).forwardSoftLimitEnabled(true);
        leftMotorConfig.smartCurrentLimit(40);

        rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.inverted(false);
        rightMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        rightMotorConfig.softLimit.reverseSoftLimit(10).reverseSoftLimitEnabled(true);
        rightMotorConfig.softLimit.forwardSoftLimit(360).forwardSoftLimitEnabled(true);
        rightMotorConfig.smartCurrentLimit(40);

        leftMotor.configure(leftMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        rightMotor.configure(rightMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    public double getCurrentPosition() {
        return leftMotor.getEncoder().getPosition();
    }

    public void moveUp() {
        targetPosition = null;
        leftMotor.set(kManualSpeed);
        rightMotor.set(kManualSpeed);
    }

    public void moveDown() {
        targetPosition = null;
        leftMotor.set(-kManualSpeed);
        rightMotor.set(-kManualSpeed);
    }

    public void moveToPosition(double position) {
        double min = 10.0;
        double max = 360.0;
        if (position < min) position = min;
        if (position > max) position = max;
        targetPosition = position;
    }

    public void stop() {
        targetPosition = null;
        leftMotor.set(0);
        rightMotor.set(0);
    }

    @Override
    public void periodic() {
        if (targetPosition == null) return;

        double current = getCurrentPosition();
        double error = targetPosition - current;

        if (Math.abs(error) < DEAD_BAND) {
            stop();
            return;
        }

        double output = (kP * error) + Math.copySign(FEEDFORWARD, error);

        if (Math.abs(output) < MIN_OUTPUT) {
            output = Math.copySign(MIN_OUTPUT, output);
        }
        if (output > kManualSpeed) output = kManualSpeed;
        if (output < -kManualSpeed) output = -kManualSpeed;

        leftMotor.set(output);
        rightMotor.set(output);
    }
}
