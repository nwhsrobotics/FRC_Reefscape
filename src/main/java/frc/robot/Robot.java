package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.CANAssignments;
import frc.robot.Constants.LoggerConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.ImprovedPowerDistribution;

import java.util.List;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    public RobotContainer robotContainer;
    public ImprovedPowerDistribution robotPD;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        Pathfinding.setPathfinder(new LocalADStarAK());
        FollowPathCommand.warmupCommand().schedule();
        PathfindingCommand.warmupCommand().schedule();
        robotPD = new ImprovedPowerDistribution(CANAssignments.PDU_ID, Constants.PDU_TYPE);

        Logger.recordMetadata("version", LoggerConstants.RUNNING_UNDER);
        Logger.recordMetadata("build_commit", BuildConstants.GIT_SHA);
        Logger.recordMetadata("build_branch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("build_git_uncommitted_changes", String.valueOf(BuildConstants.DIRTY));
        Logger.recordMetadata("build_timestamp", BuildConstants.BUILD_DATE);

        switch (LoggerConstants.MODE) {
            case REAL:
                Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
                if (!LoggerConstants.SILENT_NT4) {
                    Logger.addDataReceiver(new NT4Publisher());
                }
                break;
            case SIMULATION:
                Logger.addDataReceiver(new WPILOGWriter(""));
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY:
                setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        Logger.start();

        // if you want to stop the robot, use the boolean returned by this method.
        Constants.CANAssignments.checkAssignments();

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        for(int port = 5800; port <= 5809; port++){
            PortForwarder.add(port, Constants.LimelightConstants.llBack + ".local",port);
            // http://roborio-(teamnum)-FRC.local:5801
            PortForwarder.add(port+10, Constants.LimelightConstants.llFront + ".local",port);
            // http://roborio-(teamnum)-FRC.local:5811
        }
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        robotContainer.swerveSubsystem.autonavigator.disable();
        
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();
        //autonomousCommand.addRequirements(robotContainer.swerveSubsystem);

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        // System.out.println(robotContainer.swerveSubsystem.getPose());
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        
        if (robotContainer.swerveSubsystem.autonavigator.isEnabled()) {
            if (MathUtil.applyDeadband(robotContainer.driver.getLeftX(), OIConstants.kDriveDeadband) != 0 || 
                MathUtil.applyDeadband(robotContainer.driver.getLeftY(), OIConstants.kDriveDeadband) != 0 || 
                MathUtil.applyDeadband(robotContainer.driver.getRightX(), OIConstants.kDriveDeadband) != 0 ||
                MathUtil.applyDeadband(robotContainer.driver.getRightY(), OIConstants.kDriveDeadband) != 0) {
                robotContainer.swerveSubsystem.autonavigator.pauseNavigation();
            } else {
                robotContainer.swerveSubsystem.autonavigator.resumeNavigation();
            }
        }
            
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
