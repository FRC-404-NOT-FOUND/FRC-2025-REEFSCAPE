package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SoftLimitConfig;

public class Elevator extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    private final SparkMaxConfig leftMotorConfig;
    private final SparkMaxConfig rightMotorConfig;

    private final double kManualSpeed = 1;
    private final double kP = 5.0; // Increased kP for faster response
    private final double DEAD_BAND = 5; // Increased deadband to prevent overshooting
    private final double MIN_OUTPUT = 0.3; // Minimum output to overcome static friction
    private final double FEEDFORWARD = 2.0; // Added feedforward to increase speed

    public Elevator() {
        leftMotor = new SparkMax(Constants.Elevator.LEFT_ELEVATOR_CAN_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.Elevator.RIGHT_ELEVATOR_CAN_ID, MotorType.kBrushless);

        
        leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        leftMotorConfig.inverted(true);
        leftMotorConfig.softLimit.reverseSoftLimit(0).reverseSoftLimitEnabled(true);
        leftMotorConfig.softLimit.forwardSoftLimit(370).forwardSoftLimitEnabled(true);

        rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.inverted(true);
        rightMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        rightMotorConfig.softLimit.reverseSoftLimit(0).reverseSoftLimitEnabled(true);
        rightMotorConfig.softLimit.forwardSoftLimit(370).forwardSoftLimitEnabled(true);

        leftMotor.configure(leftMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        rightMotor.configure(rightMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        
        
    }

    public double getCurrentPosition() {
        return leftMotor.getEncoder().getPosition();
    }

    public void moveUp() {
        leftMotor.set(kManualSpeed);
        rightMotor.set(kManualSpeed);
    }

    public void moveDown() {
        if (getCurrentPosition() > Constants.Elevator.HOME_POSITION) {
            leftMotor.set(-kManualSpeed);
            rightMotor.set(-kManualSpeed);
        } else {
            stop();
        }
    }

    public void moveToPosition(double position) {
        new Thread(() -> {
            while (true) {
                double current = getCurrentPosition();
                double error = position - current;

                System.out.println("Elevator current: " + current + ", target: " + position + ", error: " + error);

                // Stop the elevator if within deadband
                if (Math.abs(error) < DEAD_BAND) {
                    System.out.println("Elevator reached target. Stopping.");
                    stop();
                    break;
                }

                double output = (kP * error) + FEEDFORWARD;

                // Ensure a minimum output to overcome static friction
                if (Math.abs(output) < MIN_OUTPUT) {
                    output = Math.copySign(MIN_OUTPUT, output);
                }

                // Limit output to max manual speed
                if (output > kManualSpeed) output = kManualSpeed;
                if (output < -kManualSpeed) output = -kManualSpeed;

                leftMotor.set(output);
                rightMotor.set(output);

                try {
                    Thread.sleep(50); // Update every 50ms
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }).start();
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }
}
