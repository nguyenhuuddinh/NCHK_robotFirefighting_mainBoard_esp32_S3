#pragma once

struct OdomData {
    float x;
    float y;
    float yaw;
};

struct VelocityCmd {
    float linear_x;
    float angular_z;
};
