package org.firstinspires.ftc.teamcode.commands.presets;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.liftcommands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

//Brings everything to rest position
public class RetractOuttakeCommand extends ParallelCommandGroup {

    public RetractOuttakeCommand(Slides lift){
        addCommands(
            new InstantCommand(),
            new InstantCommand(),
            new SequentialCommandGroup(
                    new WaitCommand(800),
                    new LiftPositionCommand(lift, Slides.SlidePositions.DOWN.position, false),
                    new WaitCommand(100),
                    new InstantCommand()
            )
//            new InstantCommand(outtake::open)
        );

        addRequirements(lift);
    }
}
