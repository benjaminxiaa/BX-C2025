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
        if (OI.getInstance().getDriver().getUpDPadButtonState()) {
            Elevator.getInstance().moveToPosition(Elevator.getInstance().getPosition()+0.2);
            // Elevator.getInstance().setElevatorPower(0.1);
        }
        else if (OI.getInstance().getDriver().getDownDPadButtonState())
        {
            // Elevator.getInstance().setElevatorPower(-0.1);
            Elevator.getInstance().moveToPosition(Elevator.getInstance().getPosition()-0.2);
        }
        else
        {
            Elevator.getInstance().moveToPosition(Elevator.getInstance().getPosition());
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
