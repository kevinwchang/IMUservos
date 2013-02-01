#ifndef DCM_H_
#define DCM_H_

extern float G_Dt;

extern float pitch;
extern float yaw;

void Normalize(void);
void Drift_correction(void);
void Matrix_update(void);
void Euler_angles(void);

#endif /* DCM_H_ */