package frc.robot.commands.EE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.EndEffector;

public class IntakeAlgae extends Command {
    public IntakeAlgae () {
        addRequirements(EndEffector.getInstance());
    }

    public void execute () {
        EndEffector.getInstance().setSpeed(RobotMap.EndEffector.INTAKE_SPEED);
    }

    public boolean isFinished () {
        return EndEffector.getInstance().hasAlgae();
    }

    public void end (boolean interrupted) {
        EndEffector.getInstance().setSpeed(RobotMap.EndEffector.ALGAE_HOLD_SPEED);
    }
}
