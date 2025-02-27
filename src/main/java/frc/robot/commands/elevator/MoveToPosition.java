package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class MoveToPosition extends Command {

    public double height;
    public MoveToPosition(double height) {
        addRequirements(Elevator.getInstance());
        this.height = height;
    }

    public void execute() {
        Elevator.getInstance().setDesiredPosition(height);
        Elevator.getInstance().moveToPosition();
        Elevator.getInstance().setManual(false);
    }

    public boolean isFinished() {
        return Elevator.getInstance().atDesired();
    }

    @Override
    public void end(boolean interrupted) {
        Elevator.getInstance().setElevatorPower(0);
    }

}