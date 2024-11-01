package org.firstinspires.ftc.teamcode.commands.presets;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.armcommands.ManualArmCommand;
import org.firstinspires.ftc.teamcode.commands.armcommands.ProfiledArmCommand;
import org.firstinspires.ftc.teamcode.commands.liftcommands.ProfiledLiftCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

public class testCommand extends ParallelCommandGroup {
    private ManualArmCommand manualArmCommand;
    public testCommand(Slides lift, Arm arm, Claw claw) {
        addCommands(
                new InstantCommand(),
                new InstantCommand(),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new ProfiledLiftCommand(lift, Slides.SlidePositions.STAGE_3.position, true),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new ArmToScoringCommand(arm, claw, ArmToScoringCommand.Presets.BASKET_HIGH)
                                )
                        ),
                        new WaitCommand(800),
                        new InstantCommand()
                )
        );

        addRequirements(lift, arm, claw);
    }
}
