package com.seiford.util.spark;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ExternalFollower;

public interface FollowingConfiguration {
    public void apply(CANSparkBase motor);

    public static FollowingConfiguration spark(int deviceID, boolean inverted) {
        return external(ExternalFollower.kFollowerSpark, deviceID, inverted);
    }

    public static FollowingConfiguration external(ExternalFollower leader, int deviceID, boolean inverted) {
        return new FollowingConfiguration() {
            @Override
            public void apply(CANSparkBase motor) {
                Util.configureCheckAndVerify(ignored -> motor.follow(leader, deviceID, inverted), motor::isFollower,
                        true, "follow");
            }
        };
    }
}
