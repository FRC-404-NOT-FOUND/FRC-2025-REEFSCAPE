package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Elevator extends SubsystemBase {
    private final Timer e_timer = new Timer();

    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final RelativeEncoder leftEncoder;

    private final SparkMaxConfig leftMotorConfig;
    private final SparkMaxConfig rightMotorConfig;

    private final double kManualSpeed = 0; //0.2 out of testing

    private double kP = 0.4; //still tuning, increase
    private double kI = 0.0;
    private double kD = 0.0;

    private double kV = 0.08;
    private double kA = 0.06;
    private double kS = 0.79;
    private double kG = 0.25;

    private double flatVoltage = 0.0; //testing tool to find kG and kS. can be deleted afterwards. systems of equations type sauce

    private final PIDController e_controller = new PIDController(kP, kI, kD);
    private ElevatorFeedforward e_feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

    private final TrapezoidProfile e_profile = new TrapezoidProfile(
			new TrapezoidProfile.Constraints(4, 4)); // in/s and in/s/s 
    private TrapezoidProfile.State startingState;


    // NetworkTables
    private final NetworkTable elevatorTable;

    public Elevator() {
        leftMotor = new SparkMax(Constants.Elevator.LEFT_ELEVATOR_CAN_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.Elevator.RIGHT_ELEVATOR_CAN_ID, MotorType.kBrushless);

        leftEncoder = leftMotor.getEncoder();
        
        leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        leftMotorConfig.inverted(true);
        leftMotorConfig.softLimit.reverseSoftLimit(3).reverseSoftLimitEnabled(true);
        leftMotorConfig.softLimit.forwardSoftLimit(55).forwardSoftLimitEnabled(true);
        leftMotorConfig.encoder.positionConversionFactor((Math.PI * Constants.Elevator.sprocketDiameter) / 9); // returns inches. 9:1 gearbox
        leftMotorConfig.encoder.velocityConversionFactor((Math.PI * Constants.Elevator.sprocketDiameter) / 9 / 60); // returns inches per second

        rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.inverted(false);
        rightMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        rightMotorConfig.softLimit.reverseSoftLimit(3).reverseSoftLimitEnabled(true);
        rightMotorConfig.softLimit.forwardSoftLimit(55).forwardSoftLimitEnabled(true);
        rightMotorConfig.encoder.positionConversionFactor((Math.PI * Constants.Elevator.sprocketDiameter) / 9); // returns inches. 9:1 gearbox
        rightMotorConfig.encoder.velocityConversionFactor((Math.PI * Constants.Elevator.sprocketDiameter) / 9 / 60); // returns inches per second

        leftMotor.configure(leftMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        rightMotor.configure(rightMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        // NetworkTables setup
        elevatorTable = NetworkTableInstance.getDefault().getTable("Elevator");

        // Initialize tunable constants
        elevatorTable.getEntry("kP").setDouble(kP);
        elevatorTable.getEntry("kI").setDouble(kI);
        elevatorTable.getEntry("kD").setDouble(kD);
        elevatorTable.getEntry("kS").setDouble(kS);
        elevatorTable.getEntry("kG").setDouble(kG);
        elevatorTable.getEntry("kV").setDouble(kV);
        elevatorTable.getEntry("kA").setDouble(kA);
        elevatorTable.getEntry("Test flat voltage").setDouble(flatVoltage);
    }

    @Override
    public void periodic() {
        // Update constants from NetworkTables
        kP = elevatorTable.getEntry("kP").getDouble(kP);
        kI = elevatorTable.getEntry("kI").getDouble(kI);
        kD = elevatorTable.getEntry("kD").getDouble(kD);
        kS = elevatorTable.getEntry("kS").getDouble(kS);
        kG = elevatorTable.getEntry("kG").getDouble(kG);
        kV = elevatorTable.getEntry("kV").getDouble(kV);
        kA = elevatorTable.getEntry("kA").getDouble(kA);
        flatVoltage = elevatorTable.getEntry("Test flat voltage").getDouble(flatVoltage);

        e_controller.setP(kP);
        e_controller.setI(kI);
        e_controller.setD(kD);

        e_feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

        //flatVoltage;

        // Publish actual position for graphing
        elevatorTable.getEntry("ActualPosition").setDouble(leftEncoder.getPosition());
        elevatorTable.getEntry("ActualVelocity").setDouble(leftEncoder.getVelocity());
    }

    public void moveUp() {
        leftMotor.setVoltage(kManualSpeed + flatVoltage);
        rightMotor.setVoltage(kManualSpeed + flatVoltage);
    }

    public void moveDown() {
        leftMotor.setVoltage(-kManualSpeed);
        rightMotor.setVoltage(-kManualSpeed);
    }

    public void resetSetpoint() {
        e_timer.restart();
        startingState = new TrapezoidProfile.State(leftEncoder.getPosition(), leftEncoder.getVelocity());
    }

    public void moveToPosition(double targetPosition) {
        double time = e_timer.get();

        var goalState = new TrapezoidProfile.State(targetPosition, 0);

        TrapezoidProfile.State currentState = e_profile.calculate(time, startingState, goalState);
        TrapezoidProfile.State nextState = e_profile.calculate(time + 0.02, startingState, goalState);

        double motorOutput = 
            flatVoltage
            + e_controller.calculate(leftEncoder.getPosition(), currentState.position)
            + e_feedforward.calculateWithVelocities(currentState.velocity, nextState.velocity);

        leftMotor.setVoltage(motorOutput);
        rightMotor.setVoltage(motorOutput);

        // Publish desired position for graphing
        elevatorTable.getEntry("TargetPosition").setDouble(currentState.position);
        elevatorTable.getEntry("TargetVelocity").setDouble(currentState.velocity);
        elevatorTable.getEntry("AppliedVoltage").setDouble(motorOutput);
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }
}