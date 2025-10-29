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
import edu.wpi.first.wpilibj2.command.Commands;
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
    algaePivot.enableTeleopControl(false);
    climber.unlockClimb();
    System.out.println("Teleoperated Mode Initialized: Algae pivot remains in position.");
  }

  public void autonomousInit() {
    algaePivot.disableTeleopControl();
    System.out.println("Autonomous Mode Initialized: Algae pivot locked in up position.");
  }

  public Elevator getElevator() {
    return elevator;
  }

  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .onTrue(new RunCommand(algaeIntake::startIntake, algaeIntake));

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new RunCommand(algaeIntake::holdIntake, algaeIntake));

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .onTrue(new RunCommand(algaeIntake::startOuttake, algaeIntake));

    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(new RunCommand(algaeIntake::stop, algaeIntake));

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(elevator::moveUp, elevator))
        .onFalse(new RunCommand(elevator::stop, elevator));

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new RunCommand(elevator::moveDown, elevator))
        .onFalse(new RunCommand(elevator::stop, elevator));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(() -> algaePivot.setTargetPosition(Constants.AlgaePivot.UP_POSITION)));

    new JoystickButton(m_driverController, XboxController.Button.kBack.value)
        .onTrue(new InstantCommand(() -> algaePivot.setTargetPosition(Constants.AlgaePivot.DOWN_POSITION)));

    m_commandController.leftTrigger()
        .whileTrue(new RunCommand(climber::moveBackward, climber))
        .onFalse(new RunCommand(climber::stop, climber));

    m_commandController.rightTrigger()
        .whileTrue(new RunCommand(climber::moveForward, climber))
        .onFalse(new RunCommand(climber::stop, climber));

    new JoystickButton(m_driverController, XboxController.Button.kRightStick.value)
        .onTrue(new InstantCommand(m_robotDrive::toggleFieldRelative));

    new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value)
        .onTrue(Commands.startRun(elevator::resetTimer, () -> elevator.moveToPosition(10), elevator));
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
        new InstantCommand(() -> algaePivot.enableTeleopControl(true)),
        new WaitCommand(0.5),
        new InstantCommand(() -> elevator.moveToPosition(215)),
        //new WaitUntilCommand(() -> Math.abs(elevator.getCurrentPosition() - 225) < 10),
        new InstantCommand(() -> {
          System.out.println("Intake Started!");
          algaeIntake.startIntake();
        }, algaeIntake),
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
            new InstantCommand(() -> {
              System.out.println("Lowering elevator to position 80...");
              elevator.moveToPosition(80);
            })
        ),
        //new WaitUntilCommand(() -> Math.abs(elevator.getCurrentPosition() - 80) < 10),
        new InstantCommand(algaeIntake::startOuttake, algaeIntake)
    );
  }
}
