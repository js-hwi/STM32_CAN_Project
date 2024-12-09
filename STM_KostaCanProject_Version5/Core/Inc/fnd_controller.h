/*
 * fnd_controller.h
 *
 *  Created on: Dec 2, 2024
 *      Author: KOSTA
 */
#ifndef SRC_FND_CONTROLLER_H_
#define SRC_FND_CONTROLLER_H_


#define HIGH 1
#define LOW 0
#define false 0
#define true 1

void FND_INIT();


void functionIdle(void);
void functionRunning(void);

void send(uint8_t x);
void send_port(unsigned char X, unsigned char port);

void digit4_show(int n, int replay, uint8_t showZero);
void digit4_replay(int n, int replay);
void digit4(int n);

void digit4showZero_replay(int n, int replay);
void digit4showZero(int n);

void digit2(int n, int port, int replay);
void digit2_port(int n, int port);


#endif /* SRC_FND_CONTROLLER_H_ */
