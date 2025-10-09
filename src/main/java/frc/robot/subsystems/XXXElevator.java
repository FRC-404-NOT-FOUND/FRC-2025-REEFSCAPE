package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;

public class XXXElevator extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final RelativeEncoder leftEncoder;

    private final SparkMaxConfig leftMotorConfig;
    private final SparkMaxConfig rightMotorConfig;

    private final NetworkTable table;

    private final NetworkTableEntry kPEntry, kIEntry, kDEntry;
    private final NetworkTableEntry kSEntry, kGEntry, kVEntry, kAEntry;
    private final NetworkTableEntry profiledSetpointEntry, setpointEntry, encoderPositionEntry, pidOutputEntry, feedforwardOutputEntry;

    private double kP = 0.0, kI = 0.0, kD = 0.0;
    private double kS = 0.0, kG = 0.21, kV = 0.5, kA = 0.0;

    private ElevatorFeedforward feedforward;
    private ProfiledPIDController pidElevator;

    private final double kManualSpeed = 3;
    
    public XXXElevator() {
        leftMotor = new SparkMax(Constants.Elevator.LEFT_ELEVATOR_CAN_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.Elevator.RIGHT_ELEVATOR_CAN_ID, MotorType.kBrushless);
        leftEncoder = leftMotor.getEncoder();

        leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig.encoder.positionConversionFactor(Units.inchesToMeters((Math.PI * 1.896) / 81));
        leftMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        leftMotorConfig.inverted(true);
        leftMotorConfig.softLimit.reverseSoftLimit(0).reverseSoftLimitEnabled(true);

        rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.follow(Constants.Elevator.LEFT_ELEVATOR_CAN_ID);
        rightMotorConfig.inverted(true);
        rightMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        rightMotorConfig.softLimit.reverseSoftLimit(0).reverseSoftLimitEnabled(true);

        leftMotor.configure(leftMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        rightMotor.configure(rightMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        leftEncoder.setPosition(0);

        table = NetworkTableInstance.getDefault().getTable("Elevator");

        kPEntry = table.getEntry("kP");
        kIEntry = table.getEntry("kI");
        kDEntry = table.getEntry("kD");
        kSEntry = table.getEntry("kS");
        kGEntry = table.getEntry("kG");
        kVEntry = table.getEntry("kV");
        kAEntry = table.getEntry("kA");
        profiledSetpointEntry = table.getEntry("Profiled Setpoint");
        setpointEntry = table.getEntry("Setpoint");
        encoderPositionEntry = table.getEntry("Encoder Position");
        pidOutputEntry = table.getEntry("PID Output");
        feedforwardOutputEntry = table.getEntry("Feedforward Output");

        kPEntry.setDouble(kP);
        kIEntry.setDouble(kI);
        kDEntry.setDouble(kD);
        kSEntry.setDouble(kS);
        kGEntry.setDouble(kG);
        kVEntry.setDouble(kV);
        kAEntry.setDouble(kA);
        profiledSetpointEntry.setDouble(0.25);
        setpointEntry.setDouble(0);

        feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
        pidElevator = new ProfiledPIDController(kP, kI, kD, 
            new TrapezoidProfile.Constraints(
                feedforward.maxAchievableVelocity(12, 0), 
                feedforward.maxAchievableAcceleration(12, 0)
            )
        );
    }

    private double getCurrentPosition() {
        return leftEncoder.getPosition();
    }

    @Override
    public void periodic() {
        encoderPositionEntry.setDouble(getCurrentPosition());
    }

    public void moveToPosition(double position) {
        setpointEntry.setDouble(position);

        kP = kPEntry.getDouble(kP);
        kI = kIEntry.getDouble(kI);
        kD = kDEntry.getDouble(kD);
        kS = kSEntry.getDouble(kS);
        kG = kGEntry.getDouble(kG);
        kV = kVEntry.getDouble(kV);
        kA = kAEntry.getDouble(kA);

        feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
        pidElevator.setPID(kP, kI, kD);

        TrapezoidProfile.State goal = new TrapezoidProfile.State(position, 0);
        double pidOutput = pidElevator.calculate(getCurrentPosition(), goal.position);
        profiledSetpointEntry.setDouble(pidElevator.getSetpoint().velocity);
        double feedforwardOutput = feedforward.calculate(pidElevator.getSetpoint().velocity);

        pidOutputEntry.setDouble(pidOutput);
        feedforwardOutputEntry.setDouble(feedforwardOutput);

        leftMotor.setVoltage(pidOutput + feedforwardOutput);
    }
      public void moveUp() {
        leftMotor.set(kManualSpeed);
    }

    public void moveDown() {
        leftMotor.set(-kManualSpeed);
    }

    public void stop() {
        leftMotor.set(0);
    }
}
