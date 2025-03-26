#ifndef PILOTE_MODE_H
#define PILOTE_MODE_H

void setup_pilote_mode();
void loop_pilote_mode();
void calibrateSensors();
void setupMotores();
void applyControl(float tau_x, float tau_y, float tau_z);
void meansensors();
void calibration();

#endif // PILOTE_MODE_H