package com.compLevel0;

import java.util.Optional;
import java.util.function.Supplier;

public class LEDController {

    public final Supplier<Optional<String>> currentAnimation;
    public final Supplier<Optional<String>> currentColor;

    public final Runnable update;

    private LEDController(
            Supplier<Optional<String>> currentAnimation,
            Supplier<Optional<String>> currentColor,
            Runnable update) {
        this.currentAnimation = currentAnimation;
        this.currentColor = currentColor;
        this.update = update;

    }

}
