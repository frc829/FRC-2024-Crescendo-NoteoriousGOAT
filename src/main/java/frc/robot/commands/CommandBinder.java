package frc.robot.commands;

public class CommandBinder {

        private CommandBinder() {
        }

        // public static final Consumer<Trigger> bindManualPickupCommand = (trigger) -> {
        //         Command manualPickupCommand = CommandCreator.createPickupCommand.get();
        //         manualPickupCommand.setName("Manual Pickup");
        //         trigger.whileTrue(manualPickupCommand);
        // };

        // public static final Consumer<Trigger> bindManualFenderShootCommand = (trigger) -> {
        //         Command manualShooterCommand = CommandCreator.createFenderShootCommand.get();
        //         manualShooterCommand.setName("Manual Fender Shoot");
        //         trigger.whileTrue(manualShooterCommand);
        // };

        // public static final Consumer<Trigger> bindManualResetSteerEncodersCommand = (trigger) -> {
        //         trigger.onTrue(RobotContainer.driveSubsystem.createSteerEncodersResetCommand.get());
        // };

        // public static final Consumer<Trigger> bindManualModuleZeroCommand = (trigger) -> {
        //         trigger.whileTrue(RobotContainer.driveSubsystem.zeroModulesCommand);
        // };


        // public static final Function<Pose2d, Function<PathConstraints, Function<Double, Function<Double, Consumer<Trigger>>>>> bindPathFindToPoseCommandToTrigger = (
        //                 targetPose) -> (constraints) -> (
        //                                 goalEndVelocityMPS) -> (rotationDelayDistance) -> (trigger) -> {
        //                                         Supplier<Command> pathFindToPoseCommand = CommandCreator.createPathFindToPoseCommand
        //                                                         .apply(targetPose)
        //                                                         .apply(constraints)
        //                                                         .apply(goalEndVelocityMPS)
        //                                                         .apply(rotationDelayDistance);

        //                                         trigger.whileTrue(Commands
        //                                                         .deferredProxy(pathFindToPoseCommand));
        //                                         trigger.onFalse(RobotContainer.driveSubsystem.fieldCentricOriginCommand);
        //                                 };

        // public static final Function<Supplier<Optional<Pose2d>>, Function<PathConstraints, Function<Double, Function<Double, Function<Boolean, Consumer<Trigger>>>>>> bindPathFindToSuppliedPoseCommandToTrigger = (
        //                 targetPose) -> (constraints) -> (
        //                                 goalEndVelocityMPS) -> (rotationDelayDistance) -> (
        //                                                 pathFlip) -> (trigger) -> {
        //                                                         Supplier<Command> setPathFindCommand = CommandCreator.createSetPathFindCommand
        //                                                                         .apply(targetPose)
        //                                                                         .apply(constraints)
        //                                                                         .apply(goalEndVelocityMPS)
        //                                                                         .apply(rotationDelayDistance)
        //                                                                         .apply(pathFlip);

        //                                                         trigger.whileTrue(Commands
        //                                                                         .deferredProxy(setPathFindCommand));
        //                                                         trigger.onFalse(RobotContainer.driveSubsystem.fieldCentricOriginCommand);
        //                                                 };

        // public static final Function<Translation2d, Consumer<Trigger>> bindPointToLocationCommandToTrigger = (
        //                 location) -> (trigger) -> {
        //                         Command pointToLocationCommand = RobotContainer.driveSubsystem.createPointToLocationCommand
        //                                         .apply(location);
        //                         trigger.whileTrue(pointToLocationCommand);
        //                 };
}
