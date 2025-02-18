package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

public class ElevatorManual extends Command {

    public ElevatorManual() {
        addRequirements(Elevator.getInstance());
    }

    @Override
    public void execute() {
        if (OI.getInstance().getOperator().getUpDPadButtonState()) {
            Elevator.getInstance().setDesiredPosition(Elevator.getInstance().getPosition()+0.2);
            Elevator.getInstance().moveToPosition();
            Elevator.getInstance().setManual(true);
        }
        else if (OI.getInstance().getOperator().getDownDPadButtonState())
        {
            Elevator.getInstance().setDesiredPosition(Elevator.getInstance().getPosition()-0.2);
            Elevator.getInstance().moveToPosition();
            Elevator.getInstance().setManual(true);
        }

        else
        {
            if (Elevator.getInstance().isManual())
            {
                Elevator.getInstance().setDesiredPosition(Elevator.getInstance().getPosition());
            }
            Elevator.getInstance().moveToPosition();
        }
    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Elevator.getInstance().setElevatorPower(0);
    }
}
