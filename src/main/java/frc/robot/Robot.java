package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

import org.opencv.core.Core;
import org.opencv.core.Mat;

/**
 * The main robot class, managing the lifecycle of the robot
 * and integrating command-based programming with WPILib's TimedRobot framework.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    /**
     * Initializes the robot and sets up subsystems and command bindings.
     * This is called once when the robot starts.
     */
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();

        UsbCamera camera = CameraServer.startAutomaticCapture();
        camera.setResolution(320, 240);

        new Thread(() -> {
            CvSink cvSink = CameraServer.getVideo();
            CvSource outputStream = CameraServer.putVideo("flocc !", 240, 320);
            Mat source = new Mat();
            Mat rotated = new Mat();

            while (!Thread.interrupted()) {
                if (cvSink.grabFrame(source) == 0) continue;
                Core.rotate(source, rotated, Core.ROTATE_90_CLOCKWISE);
                outputStream.putFrame(rotated);
            }
        }).start();
    }

    /**
     * Runs periodically, handling command scheduling.
     * Ensures that commands are continuously updated and executed.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /**
     * Called once when the robot is disabled.
     * Can be used to reset subsystems if needed.
     */
    @Override
    public void disabledInit() {}

    /**
     * Runs periodically while the robot is disabled.
     * Can be used for diagnostics or logging.
     */
    @Override
    public void disabledPeriodic() {}

    /**
     * Initializes autonomous mode, retrieving and scheduling the autonomous command.
     * Ensures that the command runs only if it is defined.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /**
     * Runs periodically during autonomous mode.
     * Additional autonomous logic can be added here if needed.
     */
    @Override
    public void autonomousPeriodic() {}

    /**
     * Initializes teleoperated mode. Cancels any running autonomous command
     * to prevent conflicts and ensures proper subsystem initialization.
     */
    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        if (m_robotContainer != null) {
            m_robotContainer.teleopInit();
        }
    }

    /**
     * Runs periodically during teleoperated mode.
     * Any teleop-specific updates should be added here.
     */
    @Override
    public void teleopPeriodic() {}

    /**
     * Initializes test mode by canceling all active commands.
     * Ensures that testing starts with a clean state.
     */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * Runs periodically during test mode.
     * Used for running isolated system diagnostics.
     */
    @Override
    public void testPeriodic() {}
}
