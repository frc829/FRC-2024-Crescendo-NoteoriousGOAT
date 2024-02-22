package com.compLevel0;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.types.animations.ctre.ColorFlow;

public class LEDController {

    public final Supplier<Optional<String>> currentAnimation;
    public final Supplier<Optional<String>> currentColor;
    public final Consumer<ColorFlow> setColorFlowAnimation;

    public final Runnable update;

    private LEDController(
            Supplier<Optional<String>> currentAnimation,
            Supplier<Optional<String>> currentColor,
            Consumer<ColorFlow> setColorFlowAnimation,
            Runnable update) {
        this.currentAnimation = currentAnimation;
        this.currentColor = currentColor;
        this.setColorFlowAnimation = setColorFlowAnimation;
        this.update = update;

    }

}
