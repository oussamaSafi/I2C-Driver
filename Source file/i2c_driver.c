/*
 * i2c_driver.c
 *
 *  Created on: Nov 18, 2023
 *      Author: Oussama Safi
 */
#include "i2c_driver.h"

void I2C_init_master(int I2C_PINS,unsigned short i2c_mode){
	/* pin configuration
	------------------------------------------------------------------------*/
	// enable PortB clock
	RCC->APB2ENR|=RCC_APB2ENR_IOPBEN;
	// enable AF clock
	RCC->APB2ENR|= RCC_APB2ENR_AFIOEN;
if(I2C_PINS == DEFAULT_PINS){
	//pin PB7 setup
	//------------------
	// AF output OPEN DRAIN
	GPIOB->CRL |=GPIO_CRL_CNF7_0;
	GPIOB->CRL |=GPIO_CRL_CNF7_1;
	// 50MHZ output mode
	GPIOB->CRL|= GPIO_CRL_MODE7_0;
	GPIOB->CRL|= GPIO_CRL_MODE7_1;

	//pin PB6 setup
	//------------------
	// AF output OPEN DRAIN
	GPIOB->CRL |=GPIO_CRL_CNF6_0;
	GPIOB->CRL |=GPIO_CRL_CNF6_1;
	// 50MHZ output mode
	GPIOB->CRL|= GPIO_CRL_MODE6_0;
	GPIOB->CRL|= GPIO_CRL_MODE6_1;
} else if(I2C_PINS == REMAP_PINS){
	// REMAP I2C1
	AFIO->MAPR |= AFIO_MAPR_I2C1_REMAP;
	//pin PB9 setup
	//------------------
	// AF output OPEN DRAIN
	GPIOB->CRH |=GPIO_CRH_CNF9_0;
	GPIOB->CRH |=GPIO_CRH_CNF9_1;
	// 50MHZ output mode
	GPIOB->CRH|= GPIO_CRH_MODE9_0;
	GPIOB->CRH|= GPIO_CRH_MODE9_1;

	//pin PB8 setup
	//------------------
	// AF output OPEN DRAIN
	GPIOB->CRH |=GPIO_CRH_CNF8_0;
	GPIOB->CRH |=GPIO_CRH_CNF8_1;
	// 50MHZ output mode
	GPIOB->CRH|= GPIO_CRH_MODE8_0;
	GPIOB->CRH|= GPIO_CRH_MODE8_1;
}
	/* i2c configuration
	------------------------------------------------------------------------*/
	//enable I2C clock
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	// i2c reset
	I2C1->CR1 |=I2C_CR1_SWRST;
	I2C1->CR1 &= ~I2C_CR1_SWRST;
	//FREQ setup
	I2C1->CR2 |= 36;
	// SM mode selection
	I2C1->CCR &= ~I2C_CCR_FS;
	// CCR setup CCR =(Tr(SCL) + Tw(SCLH)) / Tpclk1
	I2C1->CCR =i2c_mode;
	// TRISE setup TRISE = (Tr(SCL)/Tpclk1) + 1
	I2C1->TRISE |=37<<0;
	// enable I2C peripheral
	I2C1->CR1 |=I2C_CR1_PE;

	//Generate a stop condition if the line is busy
	if((I2C1->SR2 & I2C_SR2_BUSY) == 1)
	I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C_start(){
	unsigned short temp;
	// generate a start condition
	I2C1->CR1 |=I2C_CR1_START;
	// wait for SB to set
	while (!(I2C1->SR1 & I2C_SR1_SB)){
	};
	// clear SB bit
	temp =I2C1->SR1;
}

void I2C_address(uint8_t address, uint8_t rw){
		// write address to the data register
		I2C1->DR = (address<<1)|rw;
		//wait until ADDR sets
		while(!(I2C1->SR1 & I2C_SR1_ADDR)){
		}
}

void I2C_master_write_byte(uint8_t data){
	unsigned short temp;


	// ADDR=1, cleared by reading SR1 register followed by reading SR2
	temp = I2C1->SR1;
	temp =I2C1->SR2;
	// TxE=1, shift register empty, data register empty, write Data1 in DR.
	while((I2C1->SR1 & I2C_SR1_TXE) == 0){}
	I2C1->DR = data;
	// TxE=1, shift register not empty, data register empty, cleared by writing DR register
	while((I2C1->SR1 & I2C_SR1_TXE) == 0){}
}

void I2C_stop(){
	unsigned short temp;
	//generate stop condition
	temp = I2C1->SR1;
	temp = I2C1->SR2;
	I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C_master_write_data(uint8_t address, uint8_t data[]){
	int i = 0;
	I2C_start();
	I2C_address(address, 0);

	while(data[i] != '\0'){
		I2C_master_write_byte(data[i]);
		i++;
	}
	I2C_stop();
}
void send_NACK(){
	I2C1->CR1 &= ~I2C_CR1_ACK;
}

char I2C_master_rx_byte(){
	char temp;
	// clear ADDR
	temp = I2C1->SR1;
	temp = I2C1->SR2;
	// set the ACK bit
	I2C1->CR1 |=I2C_CR1_ACK;
	// receive data
	temp = I2C1->DR;
	send_NACK();
	return temp;
}

void I2C_master_rx_data(uint8_t address,char data[], unsigned short size){
	unsigned short i=0;
	char temp;
	I2C_start();
	I2C_address(address, 1);

	// clear ADDR
	temp = I2C1->SR1;
	temp = I2C1->SR2;
	data[0] = I2C1->DR;
	I2C1->CR1 |=I2C_CR1_ACK;
	for(i=0;i<size;i++){
		// waait until RXNE is set
		while(!(I2C1->SR1 & I2C_SR1_RXNE)){
		}
		data[i] =I2C1->DR;
	}
	send_NACK();
	I2C_stop();
}

void I2C_init_slave(int I2C_PINS,unsigned short i2c_mode, uint8_t address){
	/* pin configuration
		------------------------------------------------------------------------*/
		// enable PortB clock
		RCC->APB2ENR|=RCC_APB2ENR_IOPBEN;
		// enable AF clock
		RCC->APB2ENR|= RCC_APB2ENR_AFIOEN;

		if(I2C_PINS == DEFAULT_PINS){
			//pin PB7 setup
			//------------------
			// AF output OPEN DRAIN
			GPIOB->CRL |=GPIO_CRL_CNF7_0;
			GPIOB->CRL |=GPIO_CRL_CNF7_1;
			// 50MHZ output mode
			GPIOB->CRL|= GPIO_CRL_MODE7_0;
			GPIOB->CRL|= GPIO_CRL_MODE7_1;

			//pin PB6 setup
			//------------------
			// AF output OPEN DRAIN
			GPIOB->CRL |=GPIO_CRL_CNF6_0;
			GPIOB->CRL |=GPIO_CRL_CNF6_1;
			// 50MHZ output mode
			GPIOB->CRL|= GPIO_CRL_MODE6_0;
			GPIOB->CRL|= GPIO_CRL_MODE6_1;
		} else if(I2C_PINS == REMAP_PINS){
			// REMAP I2C1
			AFIO->MAPR |= AFIO_MAPR_I2C1_REMAP;
			//pin PB9 setup
				//------------------
				// AF output OPEN DRAIN
				GPIOB->CRH |=GPIO_CRH_CNF9_0;
				GPIOB->CRH |=GPIO_CRH_CNF9_1;
				// 50MHZ output mode
				GPIOB->CRH|= GPIO_CRH_MODE9_0;
				GPIOB->CRH|= GPIO_CRH_MODE9_1;

				//pin PB8 setup
				//------------------
				// AF output OPEN DRAIN
				GPIOB->CRH |=GPIO_CRH_CNF8_0;
				GPIOB->CRH |=GPIO_CRH_CNF8_1;
				// 50MHZ output mode
				GPIOB->CRH|= GPIO_CRH_MODE8_0;
				GPIOB->CRH|= GPIO_CRH_MODE8_1;
		}

		/* i2c configuration
		------------------------------------------------------------------------*/
		//enable I2C clock
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
		__disable_irq();
		// i2c reset
		I2C1->CR1 |=I2C_CR1_SWRST;
		I2C1->CR1 &= ~I2C_CR1_SWRST;
		// disable I2C
		I2C1->CR1 &= ~I2C_CR1_PE;
		//FREQ setup
		I2C1->CR2 |= 36;
		// SM mode selection
		I2C1->CCR &= ~I2C_CCR_FS;
		// CCR setup CCR =(Tr(SCL) + Tw(SCLH)) / Tpclk1
		I2C1->CCR =i2c_mode;

		// TRISE setup TRISE = (Tr(SCL)/Tpclk1) + 1
		I2C1->TRISE |=37<<0;

		// address setup
		//----------------------------------
		// 7-bit mode selection
		I2C1->OAR1 &= ~I2C_OAR1_ADDMODE;
		// bit 14 Should always be kept at 1 by software. (776) reference manual
		I2C1->OAR1 |= ( (1)<<14 );
		// select single address mode
		I2C1->OAR2 &= ~I2C_OAR2_ENDUAL;

		//setting the address value
		I2C1->OAR1 |= (address<<1);

		// enable I2C event interrupt

		NVIC_EnableIRQ(I2C1_EV_IRQn);
		I2C1->CR2|= I2C_CR2_ITEVTEN |I2C_CR2_ITBUFEN;
		// ACK set


		__enable_irq();
		// enable I2C peripheral
		I2C1->CR1 |=I2C_CR1_PE;
		I2C1->CR1 |= I2C_CR1_ACK;
}

uint8_t I2C_slave_receive_byte(){
	uint8_t Rx;
	if(~(I2C1->SR2 & I2C_SR2_TRA)){
			volatile unsigned short temp;
				if((I2C1->SR1 & I2C_SR1_ADDR)){
						// clear ADDR
						temp = I2C1->SR1;
						temp = I2C1->SR2;
						//Rx = I2C1->DR;
					}
				if(I2C1->SR1 & I2C_SR1_RXNE){
					Rx = I2C1->DR;
				}


				if((I2C1->SR1 & I2C_SR1_STOPF)){
					temp = I2C1->SR1;
					I2C1->CR1 |= (I2C_CR1_ACK | I2C_CR1_PE);
				}


				if((I2C1->SR1 & I2C_SR1_ADDR)){
					// clear ADDR
					temp = I2C1->SR1;
					temp = I2C1->SR2;
				}

				if((I2C1->SR1 & I2C_SR1_BTF)){
					// clear BTF
					temp = I2C1->SR1;
					temp = I2C1->DR;
				}
		}
	return Rx;
}

void I2C_slave_receive_data(char Rx_Buffer[],unsigned short size){
	volatile static unsigned short i=-1;
	if(~(I2C1->SR2 & I2C_SR2_TRA)){
		volatile unsigned short temp;
		if((I2C1->SR1 & I2C_SR1_ADDR)){
				// clear ADDR
				temp = I2C1->SR1;
				temp = I2C1->SR2;
				//Rx = I2C1->DR;
			}
		if(I2C1->SR1 & I2C_SR1_RXNE){
			i++;
			if(i<size){
				Rx_Buffer[i] = I2C1->DR;
			}
		}


		if((I2C1->SR1 & I2C_SR1_STOPF)){
			i=0;
			temp = I2C1->SR1;
			I2C1->CR1 |= (I2C_CR1_ACK | I2C_CR1_PE);

		}


		if((I2C1->SR1 & I2C_SR1_ADDR)){
			// clear ADDR
			temp = I2C1->SR1;
			temp = I2C1->SR2;
		}

		if((I2C1->SR1 & I2C_SR1_BTF)){
			// clear BTF
			temp = I2C1->SR1;
			temp = I2C1->DR;
		}
	}
}

void I2C_slave_transmit_byte(uint8_t data){
	volatile unsigned short temp;
	if((I2C1->SR2 & I2C_SR2_TRA)){
		if(I2C1->SR1 & I2C_SR1_ADDR){
			// clear ADDR
			temp = I2C1->SR1;
			temp = I2C1->SR2;
		}

		if(I2C1->SR1 & I2C_SR1_TXE){
			I2C1->DR = data;
		}

		if(I2C1->SR1 & I2C_SR1_AF){
			I2C1->SR1 &= ~I2C_SR1_AF;
		}
	}
}

void I2C_slave_transmit_data(char data[]){
	volatile unsigned short temp;
	volatile static unsigned short i=-1;
	if((I2C1->SR2 & I2C_SR2_TRA)){
		if(I2C1->SR1 & I2C_SR1_ADDR){
			// clear ADDR
			temp = I2C1->SR1;
			temp = I2C1->SR2;
		}
		if(I2C1->SR1 & I2C_SR1_TXE){
			i++;
			I2C1->DR = data[i];
		}

		if(I2C1->SR1 & I2C_SR1_AF){
			i=0;
			I2C1->SR1 &= ~I2C_SR1_AF;
		}

	}
}

