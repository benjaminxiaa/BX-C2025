package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;
import frc.robot.util.MathUtil;

public class DoOperator extends Command {

    public DoOperator() {
        addRequirements(Elevator.getInstance());
    }

    public void execute() {
        Elevator.getInstance().setDesiredPosition(Elevator.getInstance().getOperatorDesiredPosition());
        Elevator.getInstance().moveToPosition();
    }

    public boolean isFinished() {
        return Elevator.getInstance().atDesired();
    }

    @Override
    public void end(boolean interrupted) {
        
    }
    
}