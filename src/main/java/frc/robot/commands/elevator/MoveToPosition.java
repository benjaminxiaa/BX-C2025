package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;
import frc.robot.util.MathUtil;

public class MoveToPosition extends Command {

    private double height;

    public MoveToPosition(double height) {
        this.height = height;
        addRequirements(Elevator.getInstance());
    }

    public void execute() {
        if (Elevator.getInstance().getPosition() > height)
            Elevator.getInstance().setElevatorPower(-0.6);
        else if (Elevator.getInstance().getPosition() < height)
            Elevator.getInstance().setElevatorPower(0.6);
        
    }

    public boolean isFinished() {
        return MathUtil.compareSetpoint(Elevator.getInstance().getPosition(), height, RobotMap.Elevator.MAX_ERROR);
    }

    @Override
    public void end(boolean interrupted) {
        Elevator.getInstance().setElevatorPower(0);
    }
    
}