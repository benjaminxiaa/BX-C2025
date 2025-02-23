package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;

public class SetOperatorPosition extends InstantCommand {

    private double height;

    public SetOperatorPosition(double height) {
        addRequirements(Elevator.getInstance());
        this.height = height;
    }

    public void execute() {
        Elevator.getInstance().setOperatorDesiredPosition(height);
        Elevator.getInstance().setManual(false);
    }

    @Override
    public void end(boolean interrupted) {
        
    }
    
}