package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaePivot extends SubsystemBase {
    private final SparkMax pivotMotor;
    private double targetPosition;
    private boolean isTeleopEnabled = false;

    private final double kP = 0.05;
    private final double kMaxSpeed = 0.2;
    private final double DEAD_BAND = 0.1;

    public AlgaePivot() {
        pivotMotor = new SparkMax(Constants.AlgaePivot.PIVOT_MOTOR_CAN_ID, MotorType.kBrushless);
        targetPosition = Constants.AlgaePivot.UP_POSITION; // Start in the UP position
    }

    public void setTargetPosition(double position) {
        targetPosition = position;
        System.out.println("Algae Pivot target set to: " + position);
    }

    public double getCurrentPosition() {
        return pivotMotor.getEncoder().getPosition();
    }

    public void enableTeleopControl() {
        isTeleopEnabled = true;
        setTargetPosition(Constants.AlgaePivot.DOWN_POSITION); // Move down immediately
        update(); // Force an update to move it
    }

    public void disableTeleopControl() {
        isTeleopEnabled = false;
        setTargetPosition(Constants.AlgaePivot.UP_POSITION); // Keep it UP
    }

    @Override
    public void periodic() {
        update();
    }

    public void update() {
        if (!isTeleopEnabled) return; // Don't update in auto

        double current = getCurrentPosition();
        double error = targetPosition - current;
        double output = 0;

        if (Math.abs(error) > DEAD_BAND) {
            output = kP * error;
            if (output > kMaxSpeed) output = kMaxSpeed;
            if (output < -kMaxSpeed) output = -kMaxSpeed;
        }

        pivotMotor.set(output);
        System.out.println("Pivot current: " + current + ", target: " + targetPosition + ", error: " + error + ", output: " + output);
    }
}