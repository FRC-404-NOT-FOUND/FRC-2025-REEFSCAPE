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

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        if (m_robotContainer != null) {
            m_robotContainer.teleopInit();
            
            m_robotContainer.getAlgaePivot().enableTeleopControl(true);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}
}
