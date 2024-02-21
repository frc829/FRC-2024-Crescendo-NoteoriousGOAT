package com.types.animations.ctre;

import com.ctre.phoenix.led.RgbFadeAnimation;

public class RgbFade {
    public final double brightness;
    public final double speed;
    public final int numLed;
    public final int ledOffset;
    public final RgbFadeAnimation ctreVersion;

    public RgbFade(
            double brightness,
            double speed,
            int numLed,
            int ledOffset) {
        this.brightness = brightness;
        this.speed = speed;
        this.numLed = numLed;
        this.ledOffset = ledOffset;
        ctreVersion = new RgbFadeAnimation(brightness, speed, numLed, ledOffset);
    }

}
