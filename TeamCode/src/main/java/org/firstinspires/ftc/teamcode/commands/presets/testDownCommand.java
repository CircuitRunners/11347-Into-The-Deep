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

public class testDownCommand extends ParallelCommandGroup {
    private ManualArmCommand manualArmCommand;
    public testDownCommand(Slides lift, Arm arm, Claw claw) {
        addCommands(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new ProfiledLiftCommand(lift, Slides.SlidePositions.STAGE_0.position, false),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ProfiledArmCommand(arm, Arm.ArmPositions.REST.position, true, false, true, 3500)
                                )
                        ),
                        new WaitCommand(800)
                )
        );

        addRequirements(lift, arm, claw);
    }
}
