package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralRemoveSS;

public class CoralRemoveC extends Command {

    private double m_speed;

    public CoralRemoveC(CoralRemoveSS subsystem, double speed) {
        subsystem = RobotContainer.rc_coralRemoveSS;
        m_speed = speed;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
      RobotContainer.rc_coralRemoveSS.CoralSpin(m_speed);
    }

    @Override
    public void end(boolean interrupted) {
      RobotContainer.rc_coralRemoveSS.CoralStop();
    }
    
    @Override
    public boolean isFinished() {
      return false;
    }
}
