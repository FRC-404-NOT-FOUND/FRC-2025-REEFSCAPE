package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

public class Climber extends SubsystemBase{

    private final SparkMax climberMotor;
    private final Servo ratchet;
    private final SparkMaxConfig climberMotorConfig;
    private boolean isLocked;

    public Climber() {
        ratchet = new Servo(Constants.ClimberConstants.CLIMBER_SERVO_CHANNEL);
        climberMotor = new SparkMax(Constants.ClimberConstants.CLIMBER_CAN_ID, MotorType.kBrushless);

        climberMotorConfig = new SparkMaxConfig();
        climberMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        climberMotorConfig.softLimit.reverseSoftLimit(-115).reverseSoftLimitEnabled(true);
        climberMotorConfig.softLimit.forwardSoftLimit(120).forwardSoftLimitEnabled(true);

        climberMotor.configure(climberMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    public void lockClimb() {
            ratchet.set(1); // set angle of the servo to whatever we need to lock
            System.out.println(ratchet.getPosition());
            isLocked = true;
    }
     public void unlockClimb() {
            ratchet.set(0); // set angle of the servo to whatever we need to unlock
            System.out.println(ratchet.getPosition());
            isLocked = false;
    }

    public void moveForward() {
        if(!isLocked) {
            lockClimb();
        }
        climberMotor.set(0.5);
        //System.out.println(climberMotor.getEncoder().getPosition());
    }

    public void moveBackward() {
        if(isLocked) {
            unlockClimb();
            Timer.delay(0.15);
        }
        climberMotor.set(-0.5);
        //System.out.println(climberMotor.getEncoder().getPosition());
    }

    public void stop() {
        climberMotor.set(0);
    }
    
}
