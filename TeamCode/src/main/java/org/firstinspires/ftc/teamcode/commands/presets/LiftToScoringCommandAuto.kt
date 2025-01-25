//package org.firstinspires.ftc.teamcode.commands.presets
//
//import com.arcrobotics.ftclib.command.InstantCommand
//import com.arcrobotics.ftclib.command.ParallelCommandGroup
//import com.arcrobotics.ftclib.command.SequentialCommandGroup
//import com.arcrobotics.ftclib.command.WaitCommand
//import org.firstinspires.ftc.teamcode.commands.liftcommands.ProfiledLiftCommand
//import org.firstinspires.ftc.teamcode.subsystems.Slides
//import org.firstinspires.ftc.teamcode.subsystems.Slides.SlidePositions
//
//class LiftToScoringCommandAuto(lift: Slides, preset: Presets) : ParallelCommandGroup() {
//
//    enum class Presets {
//        AUTO
//    }
//
//    init {
//        addCommands(
//                ParallelCommandGroup(
//                        SequentialCommandGroup(
//                                // Change this ms to change when the arm comes up
//                                WaitCommand(800),
//                                InstantCommand({
//
//                                }),
//                        ),
//                        InstantCommand({
//
//                        }),
//                        when (preset) {
//                            Presets.AUTO ->
//                                ProfiledLiftCommand(lift, SlidePositions.AUTO.position, true)
//
//                        }
//                )
//        )
//        addRequirements(lift)
//    }
//}
