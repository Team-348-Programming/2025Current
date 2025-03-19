// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Auto extends Command {
  /** Creates a new Auto. */
  public Auto(DriveSubsystem subsystem) {
    subsystem = RobotContainer.m_robotDrive;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_robotDrive.resetOdometry(new Pose2d());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*double starttime = Timer.getFPGATimestamp();
    if (starttime - Timer.getFPGATimestamp() < 2) { 
      RobotContainer.m_robotDrive.drive(-0.3, 0, 0, false);
    }
    else if (starttime - Timer.getFPGATimestamp() < 2.0001) {
      RobotContainer.rc_PIDSS.Motor2.getEncoder().setPosition(0);
    }
    else if (starttime - Timer.getFPGATimestamp() < 2.2) {
      new ElevPIDC(RobotContainer.rc_PIDSS, () -> 32);
    }
    else if (starttime - Timer.getFPGATimestamp() < 2.5) {
      RobotContainer.rc_PneumaticsSS.ToggleCoral();
    }
    else if (starttime - Timer.getFPGATimestamp() < 2.6) {
      RobotContainer.rc_PneumaticsSS.ToggleCoral();
    }
    else if (starttime - Timer.getFPGATimestamp() < 2.7) {
      new ElevPIDC(RobotContainer.rc_PIDSS, () -> 1);
    }*/
    RobotContainer.m_robotDrive.drive(-0.3, 0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_robotDrive.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.m_robotDrive.getPose().getX() < -1;
  }
}
