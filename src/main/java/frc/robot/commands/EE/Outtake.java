package frc.robot.commands.EE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.EndEffector;

public class Outtake extends Command {
    public Outtake () {
        addRequirements(EndEffector.getInstance());
    }

    public void execute () {
        EndEffector.getInstance().setSpeed(RobotMap.EndEffector.OUTTAKE_SPEED);
    }

    public boolean isFinished () {
        return !EndEffector.getInstance().hasAlgae() && !EndEffector.getInstance().hasCoral();
    }

    public void end (boolean interrupted) {
        EndEffector.getInstance().setSpeed(0);
    }
}
