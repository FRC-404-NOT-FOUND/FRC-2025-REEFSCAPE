package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.OIConstants;

public class RobotContainer {
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final AlgaeIntake algaeIntake = new AlgaeIntake();
  private final AlgaePivot algaePivot = new AlgaePivot();
  private final Elevator elevator = new Elevator();
  private final Climber climber = new Climber();

  private final XboxController m_driverController =
      new XboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_commandController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  private final Timer autonomousTimer = new Timer();

  public RobotContainer() {
    configureButtonBindings();

    // Default drive: arcade/curvature style with deadbands and field-relative toggle
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    m_robotDrive.isFieldRelative()),
            m_robotDrive));
  }

  public void teleopInit() {
    algaePivot.enableTeleopControl(false); // keep current target unless Robot.teleopInit forces DOWN
    climber.unlockClimb();
    System.out.println("Teleoperated Mode Initialized: Algae pivot teleop control enabled.");
  }

  public void autonomousInit() {
    algaePivot.disableTeleopControl();
    System.out.println("Autonomous Mode Initialized: Algae pivot locked in UP position.");
  }

  public Elevator getElevator() {
    return elevator;
  }

  public AlgaePivot getAlgaePivot() {
    return algaePivot;
  }

  private void configureButtonBindings() {
    // Intake controls
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .onTrue(new RunCommand(algaeIntake::startIntake, algaeIntake));
  
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new RunCommand(algaeIntake::holdIntake, algaeIntake));
  
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .onTrue(
            new InstantCommand(algaeIntake::startOuttake, algaeIntake)
                .andThen(new WaitCommand(1.0))
                .finallyDo(interrupted -> algaeIntake.stop())
        );
  
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(new RunCommand(algaeIntake::stop, algaeIntake));
  
    // Elevator manual jog
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(elevator::moveUp, elevator))
        .onFalse(new RunCommand(elevator::stop, elevator));
  
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new RunCommand(elevator::moveDown, elevator))
        .onFalse(new RunCommand(elevator::stop, elevator));
  
    // Pivot presets
    new JoystickButton(m_driverController, XboxController.Button.kBack.value)
        .onTrue(new InstantCommand(
            () -> algaePivot.setTargetPosition(Constants.AlgaePivot.DOWN_POSITION), algaePivot));
  
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> algaePivot.setTargetPosition(Constants.AlgaePivot.UP_POSITION), algaePivot));
  
    // Triggers -> move elevator to setpoints (press once)
    m_commandController.rightTrigger()
        .onTrue(new InstantCommand(
            () -> elevator.moveToPosition(Constants.Elevator.POS1), elevator));

    m_commandController.leftTrigger()
        .onTrue(new InstantCommand(
            () -> elevator.moveToPosition(Constants.Elevator.POS2), elevator));

    new JoystickButton(m_driverController, XboxController.Button.kRightStick.value)
        .onTrue(new InstantCommand(m_robotDrive::toggleFieldRelative));
  }

  public SequentialCommandGroup getAutonomousCommand() {
    return new SequentialCommandGroup(
        new StartEndCommand(() -> m_robotDrive.drive(0.4, 0, 0, false),
                            () -> m_robotDrive.drive(0, 0, 0, false), m_robotDrive)
            .withTimeout(1.325),
        new WaitCommand(1),
        new StartEndCommand(() -> m_robotDrive.drive(-0.1, 0, 0, false),
                            () -> m_robotDrive.drive(0, 0, 0, false), m_robotDrive)
            .withTimeout(1),

        new InstantCommand(() -> algaePivot.enableTeleopControl(true)), // allow pivot down if desired
        new WaitCommand(0.5),

        new InstantCommand(() -> elevator.moveToPosition(215)),
        new WaitUntilCommand(() -> Math.abs(elevator.getCurrentPosition() - 225) < 10),

        new InstantCommand(algaeIntake::startIntake, algaeIntake),
        new WaitCommand(0.5),

        new StartEndCommand(() -> m_robotDrive.drive(0.1, 0, 0, false),
                            () -> m_robotDrive.drive(0, 0, 0, false), m_robotDrive)
            .withTimeout(1),

        new WaitCommand(0.5),
        new InstantCommand(algaeIntake::holdIntake, algaeIntake),

        new StartEndCommand(() -> m_robotDrive.drive(-0.1, 0, 0, false),
                            () -> m_robotDrive.drive(0, 0, 0, false), m_robotDrive)
            .withTimeout(1),

        new StartEndCommand(() -> m_robotDrive.drive(0, 0, 0.25, false),
                            () -> m_robotDrive.drive(0, 0, 0, false), m_robotDrive)
            .withTimeout(1.0),

        new ParallelCommandGroup(
            new StartEndCommand(() -> m_robotDrive.drive(0.3, 0, 0, false),
                                () -> m_robotDrive.drive(0, 0, 0, false), m_robotDrive)
                .withTimeout(2.65),
            new InstantCommand(() -> elevator.moveToPosition(80))
        ),

        new WaitUntilCommand(() -> Math.abs(elevator.getCurrentPosition() - 80) < 10),
        new InstantCommand(algaeIntake::startOuttake, algaeIntake)
    );
  }
}
