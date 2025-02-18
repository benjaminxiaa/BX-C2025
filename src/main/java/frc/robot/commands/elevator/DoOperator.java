package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;

public class DoOperator extends InstantCommand {

    public DoOperator() {
        addRequirements(Elevator.getInstance());
    }

    public void execute() {
        Elevator.getInstance().setDesiredPosition(Elevator.getInstance().getOperatorDesiredPosition());
    }

    @Override
    public void end(boolean interrupted) {
        
    }
    
}