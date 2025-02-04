package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

public class ZeroElevator extends Command {
    public ZeroElevator() {
        addRequirements(Elevator.getInstance());
    }

    public void execute() {
        Elevator.getInstance().setElevatorPower(RobotMap.Elevator.ZERO_SPEED);
    }

    public boolean isFinished() {
        return Elevator.getInstance().isLimitHit() || Elevator.getInstance().isStalling();
    }

    public void end(boolean interrupted) {
        Elevator.getInstance().setElevatorPower(0);
        Elevator.getInstance().setSensorPosition(0);
    }
    
}