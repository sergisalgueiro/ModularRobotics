#ifndef INCLUDE_MOTORS_H_
#define INCLUDE_MOTORS_H_

#ifdef __cplusplus
extern "C" {
#endif

void forwardLeftMotor(int speedFL);
void forwardRightMotor(int speedFR);
void BackLeftMotor(int speedBL);
void BackRightMotor(int speedBR);
void StopLeftMotor(void);
void StopRightMotor(void);
void TurnRight(int turnL, int turnspeedL);
void TurnLeft(int turnR, int turnspeedR);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_MOTORS_H_ */
