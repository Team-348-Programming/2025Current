// WTF does this do\?
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.*;
import frc.robot.RobotContainer;

public class ElevZeroC extends Command {

  public ElevZeroC(PIDSS subsystem) {
    subsystem = RobotContainer.rc_PIDSS;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    RobotContainer.rc_PIDSS.ElevZero();
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.rc_PIDSS.ElevStop();
    RobotContainer.rc_PIDSS.Motor2.getEncoder().setPosition(0);
  }

  @Override
  public boolean isFinished() {
    return RobotContainer.rc_PIDSS.buttonInterrupted();
  }
}
