package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class Ramsete extends SequentialCommandGroup {
    
    //Creates new Ramsete
    
    String trajectoryJSON = "pathplanner/generatedJSON/New Path.wpilib.json";
  Trajectory trajectory = new Trajectory();

  public Ramsete() {
    DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
          Constants.SimConstants.kS,
          Constants.SimConstants.kV,
          Constants.SimConstants.kA),
      Constants.SimConstants.kDriveKinematics,
      11);

      TrajectoryConfig config = new TrajectoryConfig(
        Constants.SimConstants.kMaxSpeed,
        Constants.SimConstants.kMaxAcceleration)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.SimConstants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
 
  RamseteCommand ramseteCommand = new RamseteCommand(
    trajectory,
    RobotContainer.dt::getPose,
    new RamseteController(Constants.SimConstants.kRamseteB, Constants.SimConstants.kRamseteZeta),
    new SimpleMotorFeedforward(
        Constants.SimConstants.kS,
        Constants.SimConstants.kV,
        Constants.SimConstants.kA),
    Constants.SimConstants.kDriveKinematics,
    RobotContainer.dt::getWheelSpeeds,
    new PIDController(Constants.SimConstants.kPVel, 0, 0),
    new PIDController(Constants.SimConstants.kPVel, 0, 0),
    // RamseteCommand passes volts to the callback
    RobotContainer.dt::tankDriveVoltage,
    RobotContainer.dt);

    RobotContainer.dt.getField2d().getObject("traj").setTrajectory(trajectory);
    RobotContainer.dt.resetOdometry(trajectory.getInitialPose());

    Command ramc = ramseteCommand.handleInterrupt(() -> RobotContainer.dt.tankDrive(0.0, 0.0))
        .andThen(() -> RobotContainer.dt.tankDrive(0.0, 0.0));


    addCommands(ramc);
    } 

}

