package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    public AlgaeIntake() {
        leftMotor = new SparkMax(Constants.AlgaeIntakeConstants.LEFT_ALGAE_CAN_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.AlgaeIntakeConstants.RIGHT_ALGAE_CAN_ID, MotorType.kBrushless);
        rightMotor.setInverted(true);  // Reverse the right motor for opposite direction intake
    }

    public void startIntake() {
        leftMotor.set(0.35);  // Slight increase to avoid motor overdraw
        rightMotor.set(0.35);  // Reverse direction for intake
    }

    public void holdIntake() {
        leftMotor.set(0.1);  // Slow hold speed
        rightMotor.set(0.1); 
    }

    public void startOuttake() {
        leftMotor.set(-0.2);  // Reduce speed to prevent overcurrent
        rightMotor.set(-0.2);  // Slow speed for outtake to avoid motor stress
    }

    public void stop() {
        leftMotor.set(0);  // Full stop for both motors
        rightMotor.set(0); 
    }
}