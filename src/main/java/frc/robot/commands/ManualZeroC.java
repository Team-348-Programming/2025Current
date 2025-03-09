package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PIDSS;

public class ManualZeroC extends Command{
    
    public ManualZeroC(PIDSS subsystem) {
        subsystem = RobotContainer.rc_PIDSS;
        addRequirements(subsystem);
    }
@Override
  public void initialize() {}

  @Override
  public void execute() {
    RobotContainer.rc_PIDSS.ElevZero();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.rc_PIDSS.Motor2.getEncoder().setPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
