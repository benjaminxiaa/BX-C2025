package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class MoveToPosition extends Command {

    public MoveToPosition() {
        addRequirements(Elevator.getInstance());
    }

    public void execute() {
        Elevator.getInstance().moveToPosition();

    }

    public boolean isFinished() {
        return Elevator.getInstance().atDesired();
    }

    @Override
    public void end(boolean interrupted) {
        Elevator.getInstance().setElevatorPower(0);
    }

}