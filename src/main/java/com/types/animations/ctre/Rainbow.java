package com.types.animations.ctre;

import com.ctre.phoenix.led.RainbowAnimation;

public class Rainbow {
    public final double brightness;
    public final double speed;
    public final int numLed;
    public final int ledOffset;
    public final boolean reverse;
    public final RainbowAnimation ctreVersion;

    public Rainbow(
            double brightness,
            double speed,
            int numLed,
            boolean reverse,
            int ledOffset) {
        this.brightness = brightness;
        this.speed = speed;
        this.numLed = numLed;
        this.reverse = reverse;
        this.ledOffset = ledOffset;
        ctreVersion = new RainbowAnimation(brightness, speed, numLed, reverse, ledOffset);
    }

}
