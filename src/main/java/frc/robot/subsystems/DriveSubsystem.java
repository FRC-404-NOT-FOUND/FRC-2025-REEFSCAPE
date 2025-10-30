package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

// OVERALL TODO: Tune auto align PID, write code for pose estimation based on ApilTags
//      -> See lines 64 & 90

public class DriveSubsystem extends SubsystemBase {
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // Gyro
  private final ADIS16448_IMU m_gyro = new ADIS16448_IMU();

  // Field-oriented toggle (default ON)
  private boolean m_fieldRelative = true;
  public boolean isFieldRelative() { return m_fieldRelative; }
  public void toggleFieldRelative() {
    m_fieldRelative = !m_fieldRelative;
    System.out.println("Field Oriented: " + (m_fieldRelative ? "ON" : "OFF"));
  }

  // Odometry
  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getGyroAngleZ()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  private final trajectoryController = new HolonomicDriveController(
      //TODO: tune all three controllers + set Constants
      new PIDController(0, 0, 0), new PIDController(0, 0, 0),
      new ProfiledPIDController(0, 0, 0,
                                new TrapezoidProfile.Constraints(DriveConstants.kMaxAngularVelocity, DriveConstants.kMaxAngularAcceleration)));

  public DriveSubsystem() {
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  @Override
  public void periodic() {
    double angle = m_gyro.getGyroAngleZ();
    System.out.println("Gyro X Angle: " + m_gyro.getGyroAngleX());
    System.out.println("Gyro Y Angle: " + m_gyro.getGyroAngleY());
    System.out.println("Gyro Z Angle: " + angle);
    System.out.println("Gyro is connected: " + m_gyro.isConnected());

    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getGyroAngleZ()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    // TODO: IF apil tag detected then update odometry with estimated distance
    // note: do i also have to add logic to make sure encoder/gyro pos are based off of most recent apil tag? make sure to research

  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getGyroAngleZ()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Drive method.
   * @param xSpeed forward
   * @param ySpeed strafe
   * @param rot    yaw rate
   * @param fieldRelative use field-oriented if true
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered,
                Rotation2d.fromDegrees(-m_gyro.getGyroAngleZ()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

    public void autoAlign(double x, double y, double rot) {
      Pose2D targetPose = new Pose2D(x, y, Rotation2D.fromDegrees(rot)); // Distance = meters; theta = radians; by default
      Pose2D currentPose = new Pose2D(getPose().getX(), getPose().getY(), getPose().getRotation());
      Pose2D errorPose = currentPose.relativeTo(targetPose);

      if(Math.abs(errorPose.getX()) < 0.3 && Math.abs(errorPose.getY()) < 0.3 && Math.abs(errorPose.getRotation().getDegrees()) < 45) { 
          // must be within 0.3 meters and 45 degrees of goal
          //    -> theoretically prevents driver from accidentally pressing button
          //       and attempting auto align from half way across field
          ChassisSpeeds adjustedSpeeds = trajectoryController.calculate(
              currentPose, targetPose, targetPose.getRotation);

          SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(adjustedSpeeds);
          setModuleStates(moduleStates);
      }
      
  }

  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getGyroAngleZ()).getDegrees();
  }

  public double getTurnRate() {
    return m_gyro.getGyroRateZ() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
