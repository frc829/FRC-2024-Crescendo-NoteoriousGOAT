package com.types.animations.ctre;

import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

public class Larson {

    public final int r;
    public final int g;
    public final int b;
    public final int w;
    public final double speed;
    public final int numLed;
    public final int size;
    public final int ledOffset;
    public final String bounceMode;
    public final LarsonAnimation ctreVersion;

    public Larson(
            int r,
            int g,
            int b,
            int w,
            double speed,
            int numLed,
            int size,
            int ledOffset,
            String bounceMode) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.w = w;
        this.speed = speed;
        this.numLed = numLed;
        this.size = size;
        this.ledOffset = ledOffset;
        this.bounceMode = bounceMode;
        if (bounceMode == "Front") {
            ctreVersion = new LarsonAnimation(r, g, b, w, speed, numLed, BounceMode.Front, numLed, ledOffset);
        } else if (bounceMode == "Center") {
            ctreVersion = new LarsonAnimation(r, g, b, w, speed, numLed, BounceMode.Center, numLed, ledOffset);
        } else {
            ctreVersion = new LarsonAnimation(r, g, b, w, speed, numLed, BounceMode.Back, numLed, ledOffset);
        }
    }

}
