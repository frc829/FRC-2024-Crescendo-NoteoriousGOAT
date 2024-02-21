package com.types.animations.ctre;

import com.ctre.phoenix.led.StrobeAnimation;

public class Strobe {

    public final int r;
    public final int g;
    public final int b;
    public final int w;
    public final double speed;
    public final int numLed;
    public final int ledOffset;
    public final StrobeAnimation ctreVersion;

    public Strobe(
            int r,
            int g,
            int b,
            int w,
            double speed,
            int numLed,
            int ledOffset) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.w = w;
        this.speed = speed;
        this.numLed = numLed;
        this.ledOffset = ledOffset;
        ctreVersion = new StrobeAnimation(r, g, b, w, speed, numLed, ledOffset);
    }

}
