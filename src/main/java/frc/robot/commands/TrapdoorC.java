package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PneumaticsSS;

public class TrapdoorC extends InstantCommand{
    public TrapdoorC(PneumaticsSS subsystem) {
        subsystem = RobotContainer.rc_PneumaticsSS;
        addRequirements(subsystem);
    }
    
    @Override
    public void initialize() {
        if (RobotContainer.m_driverController.leftTrigger().getAsBoolean() 
        && RobotContainer.m_operatorController.leftTrigger().getAsBoolean()) {
            RobotContainer.rc_PneumaticsSS.ToggleDoor();
        }
    }
}
