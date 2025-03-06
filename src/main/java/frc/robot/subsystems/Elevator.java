package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;



public class Elevator extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    //START OF ADAM CODE
    private final double kP = 0.1;
    private final double kI = 0.0;
    private final double kD = 0.0;
    PIDController pidElevator = new PIDController(kP, kI, kD);

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    //END OF ADAM CODE
    
    private final double kManualSpeed = 6;
    //START OF SPENCER CODE
    //private final double kP = 0.05;
    //END OF SPENCER CODE
    private final double DEAD_BAND = 1.0;

    public Elevator() {
        leftMotor = new SparkMax(Constants.Elevator.LEFT_ELEVATOR_CAN_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.Elevator.RIGHT_ELEVATOR_CAN_ID, MotorType.kBrushless);

        rightMotor.setInverted(true);
        leftMotor.setInverted(true);
        
        //START OF ADAM CODE
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
    
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        //END OF ADAM CODE
    }

    private double getCurrentPosition() {
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
        //BEGINNING OF SPENCER CODE
        /*
        double error = position - getCurrentPosition();
        double output = kP * error;

        if (Math.abs(error) < DEAD_BAND) {
            stop();
            return;
        }

        if (output > kManualSpeed) output = kManualSpeed;
        if (output < -kManualSpeed) output = -kManualSpeed;

        leftMotor.set(output);
        rightMotor.set(output);
        */
        //END OF SPENCER CODE

        //START OF ADAM CODE
        double speed = pidElevator.calculate(leftEncoder.getPosition(), position);
        speed = Math.max(-10, Math.min(10, speed)); // Limit speed

        leftMotor.set(speed);
        rightMotor.set(speed);

        //END OF ADAM CODE
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }
}