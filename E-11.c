// Código combinado: Motor de passos controlado por teclado matricial
// PB12 à PB15 em IN1 à IN4; PA0 alterna entre os três modos; PA1 alterna o sentido de rotação
// PA4 a PA7 -> pinos de 5 a 8 de varredura das colunas do teclado; PA0 a PA3 -> pinos de 1 a 4 como linhas de IRQ EXTIx
// PA9 Tx da USART1

#include "stm32f1xx.h" // Biblioteca STM

char aguarda = 0;   // temporizador do antibounce
char sentido = 1;   // 0->anti-horário; 1->horário
char modo = 0;      // 0->onda; 1->mais torque; 2->mais preciso

#define FREQ_PASSO_BASE 100 // base para frequência do motor de passo
int passo = 0;      // passo corrente
char onda[4] = {0b0001, 0b0010, 0b0100, 0b1000};
char mais_torque[4] = {0b0011, 0b0110, 0b1100, 0b1001};
char mais_preciso[8] = {0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001,};

int coluna = 0;
char tecla_press = 0;
unsigned int transmitir = 0;
int potencia = 100; // Potência inicial de 100%

const char matriz_teclas[4][4] = {
    {   '1',  '2',  '3',  'A'},  // valor em ASCII
    {   '4',  '5',  '6',  'B'},
    {   '7',  '8',  '9',  'C'},
    {   '*',  '0',  '#',  'D'}
};

void ConfiguraMotorPotencia(int potencia) {
    SysTick->LOAD = (8000000 * potencia / 100) / FREQ_PASSO_BASE; // Configura a potência do motor
}

void SysTick_Handler(void) {  /* Trata da IRQ do SysTick */ 
    unsigned int total_passos;         
    GPIOC->ODR ^= (1<<13);  // LED toggle (troca estado do LED) 
    /* Verifica modo */
    if(modo == 2)
        total_passos = 8;   // Se modo mais preciso
    else
        total_passos = 4;   // Se onda ou mais torque
    /* Verifica sentido e busca o próximo passo */
    if(sentido){ // Horário
        if((++passo) >= total_passos)
            passo = 0;
    }
    else{   // Anti-horário
        if((--passo) < 0)
            passo = total_passos-1;
    }
    /* Escreve o passo nas saídas PB12 a PB15 */
    switch(modo) {
        case 0: // onda
            GPIOB->ODR = (GPIOB->ODR & 0x0FFF) | (onda[passo]<<12);
            break;
        case 1: // mais torque
            GPIOB->ODR = (GPIOB->ODR & 0x0FFF) | (mais_torque[passo]<<12);
            break;
        default: // mais preciso
            GPIOB->ODR = (GPIOB->ODR & 0x0FFF) | (mais_preciso[passo]<<12);
    }
    /* antibounce dos push-buttons */
    if(aguarda>0) 
        aguarda--;
    else
        EXTI->IMR |= EXTI_IMR_IM0 | EXTI_IMR_IM1; // Hab. a int. do EXTI0 e EXTI1 
}

void EXTI0_IRQHandler (void) {  /* Tratamento da IRQ do EXTI0 */
    EXTI->PR = EXTI_PR_PIF0;    // Apaga flag sinalizadora da IRQ
    EXTI->IMR &= ~EXTI_IMR_IM0; // Desab. a int. do EXTI0
    aguarda = FREQ_PASSO_BASE/2+1;     // Aguarda 0,5s p/ reab. IRQ.
    if ((++modo)>2) /* Altera o modo: 0->onda; 1->mais torque; 2->mais preciso */
        modo = 0;   
}

void EXTI1_IRQHandler (void) {  /* Tratamento da IRQ do EXTI1 */
    EXTI->PR = EXTI_PR_PIF1;    
    EXTI->IMR &= ~EXTI_IMR_IM1; 
    aguarda = FREQ_PASSO_BASE/2+1;
    sentido = !sentido;     // Inverte o sentido (0 anti-horário; 1 horário)                    
}

void EXTI2_IRQHandler (void) {  /* Linha 2 (PA2) */
    EXTI->PR = EXTI_PR_PIF2;                              
    if(GPIOA->IDR & (1<<2)){
        tecla_press = matriz_teclas[2][coluna];
        EXTI->IMR &= ~EXTI_IMR_IM2; 
        aguarda = FREQ_PASSO_BASE/2;
        transmitir = 1;
    }      
}

void EXTI3_IRQHandler (void) {  /* Linha 3 (PA3) */
    EXTI->PR = EXTI_PR_PIF3;                  
    if(GPIOA->IDR & (1<<3)){                     
        tecla_press = matriz_teclas[3][coluna];
        EXTI->IMR &= ~EXTI_IMR_IM3;
        aguarda = FREQ_PASSO_BASE/2;
        transmitir = 1;
    }     
}

void EXTI0_IRQHandler_Input (void) {  /* Linha 0 (PA0) */
    EXTI->PR = EXTI_PR_PIF0;    // Apaga flag sinalizadora de interrupção
    if(GPIOA->IDR & (1<<0)){ // PA0 = 1?
        tecla_press = matriz_teclas[0][coluna];
        EXTI->IMR &= ~EXTI_IMR_IM0; // Desab. a int. do EXTI0
        aguarda = FREQ_PASSO_BASE/2; // Variável para aguardar 1s para reab. int.
        transmitir = 1;
    }
}
void EXTI1_IRQHandler_Input (void) {  /* Linha 1 (PA1) */
    EXTI->PR = EXTI_PR_PIF1; 
    if(GPIOA->IDR & (1<<1)){
        tecla_press = matriz_teclas[1][coluna];
        EXTI->IMR &= ~EXTI_IMR_IM1;
        aguarda = FREQ_PASSO_BASE/2;
        transmitir = 1;
    }
}

void SysTick_Handler_Input (void) {   /* Trata IRQ do SysTick (a cada 10ms) */
    if (++coluna > 3)
        coluna = 0;
    GPIOA->ODR = (GPIOA->ODR & ~(0x000000F0)) | (1 << (coluna+4));

    if(aguarda == FREQ_PASSO_BASE/2){
        GPIOC->ODR ^= (1<<13);  // Toggle LED onboard
        //transmitir = 1;
    }

    if(aguarda > 0){ // Anti bounce
        aguarda--;
        if (aguarda == 0)
            // Hab. a int. do EXTI0 a EXTI3
            EXTI->IMR |= EXTI_IMR_IM0 | EXTI_IMR_IM1 | EXTI_IMR_IM2 | EXTI_IMR_IM3;  
    }   
}

// Lógica para input/output
void AjustaPotenciaMotor(char tecla) {
    if (tecla >= '0' && tecla <= '9') {
        potencia = (tecla - '0') * 10;
    } else {
        potencia = 100; // Para qualquer outro input (A, B, C, D, #, *), seta 100%
    }
    ConfiguraMotorPotencia(potencia);
}

int main(void){ /* Funcao principal */

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;  // Habilita o clock dos periféricos do bus APB2
    GPIOC->CRH |= GPIO_CRH_MODE13_1;  // Configura pino PC13 como saída open-drain 2 MHz para LED
    GPIOB->CRH = 0x22224444;  // Configura PB12 a PB15 como saída push-pull de 2MHz para controlar o motor de passos
    GPIOA->CRL = 0x88888888;  // Configura PA0 a PA3 como entradas flutuantes para detectar as linhas do teclado matricial
    GPIOA->CRL |= 0x00008800;  // Configura PA4 a PA7 como saída push-pull de 10MHz para varredura das colunas do teclado matricial
    GPIOA->ODR |= (1<<4);  // Inicia varredura da coluna 0 do teclado matricial

    // Configura o SysTick para gerar IRQ a cada 10ms
    SysTick->LOAD = 8000000 / FREQ_PASSO_BASE;  // Configura SysTick com base na frequência do motor
    SysTick->VAL = 0;           // Limpa contador corrente
    SysTick->CTRL = 7;          // Hab. SysTick e sua IRQ c/ fonte de clock interna


    AFIO->EXTICR[0] = AFIO_EXTICR1_EXTI0_PA | AFIO_EXTICR1_EXTI1_PA | AFIO_EXTICR1_EXTI2_PA | AFIO_EXTICR1_EXTI3_PA;  // Configura as interrupções das linhas 0 a 3 (teclado matricial)
    EXTI->IMR = EXTI_IMR_IM0 | EXTI_IMR_IM1 | EXTI_IMR_IM2 | EXTI_IMR_IM3;   // Habilita interrupções
    EXTI->RTSR = EXTI_RTSR_TR0 | EXTI_RTSR_TR1 | EXTI_RTSR_TR2 | EXTI_RTSR_TR3;   // IRQ na borda de subida

    // Habilita as IRQs correspondentes
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);

    while(1){
        if (transmitir) {
            AjustaPotenciaMotor(tecla_press);   // Ajusta a potência do motor com base na tecla pressionada
            transmitir = 0;    // Reseta flag de transmissão
        }
    }
}

