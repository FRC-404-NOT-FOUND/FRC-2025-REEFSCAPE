package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.Constants.OIConstants;

public class RobotContainer {
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final AlgaeIntake algaeIntake = new AlgaeIntake();
    private final AlgaePivot algaePivot = new AlgaePivot();
    private final Elevator elevator = new Elevator();

    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    private Timer autonomousTimer = new Timer();

    public RobotContainer() {
        configureButtonBindings();

        m_robotDrive.setDefaultCommand(
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true),
                m_robotDrive));
    }

    public void teleopInit() {
        algaePivot.enableTeleopControl(); // Move algae pivot down
        algaeIntake.holdIntake(); // Keep intake in hold mode when teleop starts
        System.out.println("Teleoperated Mode Initialized: Algae pivot moving to down position, intake in hold mode.");
    }
    

    public Elevator getElevator() {
        return elevator;
    }
    
    public void autonomousInit() {
        algaePivot.disableTeleopControl(); // Keep algae pivot up
        System.out.println("Autonomous Mode Initialized: Algae pivot locked in up position.");
    
        // Move elevator up
        elevator.moveToPosition(Constants.Elevator.HOME_POSITION);
        System.out.println("Autonomous Mode: Elevator moving to position 100.");
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
            .onTrue(new RunCommand(() -> elevator.moveToPosition(100), elevator));
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            // Step 1: Move forward for 1 second
           new StartEndCommand(
             () -> m_robotDrive.drive(0.1, 0, 0, true),  // Start movement
              () -> m_robotDrive.drive(0, 0, 0, true),    // Stop movement
                m_robotDrive
           ).withTimeout(1.0),
    
            // Step 2: Pause for 3 seconds
            new WaitCommand(3.0),
    
            // Step 3: Move backward slightly to create space for elevator movement
            new StartEndCommand(
                () -> m_robotDrive.drive(-0.1, 0, 0, true),  // Start movement
                 () -> m_robotDrive.drive(0, 0, 0, true),    // Stop movement
                   m_robotDrive
              ).withTimeout(1.0),
    
            // Step 4: Move elevator to position 
            new InstantCommand(() -> algaePivot.enableTeleopControl()),
            new InstantCommand(() -> elevator.moveToPosition(50), elevator),
    
            // Step 5: Start intake and move forward slightly
            new RunCommand(() -> {
                algaeIntake.startIntake();
                m_robotDrive.drive(0.1, 0, 0, true); // Small forward movement
            }, algaeIntake, m_robotDrive).withTimeout(1),
    
            // Step 6: Switch to hold mode and move back slightly
            new RunCommand(() -> {
                algaeIntake.holdIntake();
                m_robotDrive.drive(-0.1, 0, 0, true); // Small backward movement
            }, algaeIntake, m_robotDrive).withTimeout(3),
    

            // Step 8: Ensure intake stays in hold mode indefinitely
            new InstantCommand(algaeIntake::holdIntake, algaeIntake)
        );
    }    

}