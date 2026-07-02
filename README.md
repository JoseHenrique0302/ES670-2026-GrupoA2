# ES670 - Projeto de Sistemas Embarcados: Projeto Final

Robô diferencial **seguidor de linha**, construído sobre uma **NUCLEO-G474RE** (STM32G4)
rodando **FreeRTOS**. O robô lê 5 sensores infravermelhos para seguir uma trilha preta
sobre fundo branco, controla os motores em malha fechada (encoder + PID), evita colisões
com um sensor ultrassônico e pode ser operado tanto pelos **botões físicos** quanto por
**Bluetooth** (módulo HC-05), incluindo o app Android descrito mais abaixo.

## Grupo A2
* **José Henrique Lima Dias** - RA: 237020
* **Isabelle de Arruda Castilho** - RA: 206798

## Quadro Miro
Acesse o nosso quadro de planejamento e modelagem no Miro através do link:
[Miro - Grupo A2](https://miro.com/app/board/uXjVHaiffog=/?share_link_id=61809044268)

---

## Sobre o repositório

Este repositório contém o **firmware** (STM32CubeIDE, HAL + FreeRTOS/CMSIS-RTOS v2)
desenvolvido para o Projeto Final da disciplina ES670 - Projeto de Sistemas Embarcados
(Sistema de Aquisição - Robô). O app Android usado para controlar o robô por Bluetooth
fica em um repositório separado — ver a seção [App Android](#app-android-de-controle)
e o item "app (.zip)" nos arquivos entregues junto com o projeto.

---

## Hardware

| Função | Periférico / Pino |
|---|---|
| Motor esquerdo | TIM1_CH1 (PC0) |
| Motor direito | TIM1_CH2 (PC1) |
| Encoder esquerdo | TIM16_CH1 (PB4) — 1 canal, 20 PPR |
| Encoder direito | TIM17_CH1 (PB5) — 1 canal, 20 PPR |
| Sensores de linha (5x IR) | ADC1 (bateria/opamp), ADC2 (3 canais), ADC3, ADC5 |
| Sensor ultrassônico (HC-SR04) | Trigger TIM20_CH1 (PB2), Eco TIM3 |
| Buzzer | TIM8_CH1 (PA15) |
| Display LCD 16x2 | I2C2 |
| Botões físicos | UP/RIGHT/LEFT/DOWN/ENTER (EXTI + debounce por TIM7) |
| Bluetooth (HC-05) | USART3 (PB10 = TX, PB11 = RX), 115200 8N1 |
| USB / terminal serial | LPUART1 (VCP do ST-Link), 115200 8N1 |
| LED vermelho | indica modo MANUAL aceso / AUTÔNOMO apagado |
| LED azul | aceso durante a calibração dos sensores |

Parâmetros físicos usados na odometria: roda com perímetro medido de **0,21 m**,
distância entre rodas (wheelbase) de **133 mm**, encoder de **20 pulsos por
revolução por roda** (1 canal — a direção da roda é inferida pelo sinal do comando
de motor, não pelo hardware do encoder).

---

## Arquitetura do firmware (FreeRTOS)

O código de aplicação fica em [Core/Src/app_freertos.c](Core/Src/app_freertos.c),
organizado nas seguintes tasks:

| Task | Período | Responsabilidade |
|---|---|---|
| `vTaskMotor` | 10 ms | Malha de controle dos motores (PID de velocidade por roda usando os encoders) |
| `vTaskSegueLinha` | 50 ms | Lê os 5 sensores IR e a bateria; no modo AUTÔNOMO roda o PID de seguimento de linha, detecta cruzamentos e o fim da pista |
| `vTaskCalibracao` | sob demanda | Calibra o min/max de cada sensor IR (fica suspensa até ser acionada) |
| `vTaskOdometria` | 50 ms | Integra os pulsos dos encoders em pose (X, Y, θ) e velocidade |
| `vTaskLCD` | 1 Hz | Exibe posição, velocidade, distância e bateria no display 16x2 |
| `vTaskTrocarModo` | orientada a evento | Trata os botões físicos e os comandos de modo vindos do Bluetooth/USB, alterna MANUAL/AUTÔNOMO |
| `vTaskUART` (BTComm) | orientada a evento + 1 Hz | Recebe comandos (USB e Bluetooth) e envia telemetria a cada 1 s, nas duas portas simultaneamente |
| `vTaskUltraBuzz` | periódica | Lê o ultrassom (com filtro de mediana) e aciona o buzzer conforme a proximidade de obstáculos |

O controle de linha usa um **PID com erro de centroide** calculado a partir dos 5
sensores binarizados/normalizados, com detecção dedicada de **cruzamentos** (anda reto
por uma janela curta ao detectar uma linha perpendicular, para não sair da pista) e de
**fim de pista** (para os motores e volta ao modo MANUAL quando os 5 sensores ficam em
branco por 600 ms seguidos).

---

## Como usar o robô

### 1. Ligar e calibrar
1. Ligue o robô sobre uma superfície neutra (fora da pista).
2. Aperte o botão **LEFT** para iniciar a calibração (o LED azul acende).
3. Enquanto o LED azul estiver aceso (≈ 3 s), **deslize o robô sobre a linha**,
   passando a barra de sensores por cima do preto e do branco, para o driver capturar
   o contraste. O LED azul apaga ao final — a calibração está pronta.

### 2. Modos de operação (botões físicos)
| Botão | Ação |
|---|---|
| **ENTER** | Alterna entre modo AUTÔNOMO e MANUAL |
| **UP** | Força o modo AUTÔNOMO (robô passa a seguir a linha sozinho) |
| **DOWN** | Força o modo MANUAL (para o seguimento de linha; motores só respondem a comandos manuais) |
| **LEFT** | Inicia a calibração dos sensores |
| **RIGHT** | Teste de motor (anda para frente a 30% de duty por alguns instantes); se houver uma emergência ativa, o mesmo botão a limpa em vez de testar o motor |

O LED vermelho aceso indica modo **MANUAL**; apagado indica **AUTÔNOMO**.

### 3. Posicionar na pista e seguir a linha
Com a calibração feita, posicione o robô com os sensores sobre a linha preta e aperte
**UP** (ou **ENTER**, se estiver em MANUAL) para entrar no modo AUTÔNOMO. O robô passa a
seguir a linha sozinho, atravessa cruzamentos em linha reta e **para automaticamente**
ao detectar o fim da pista (5 sensores em branco por 600 ms).

### 4. Controle remoto (USB ou Bluetooth)
O robô aceita os mesmos comandos ASCII (terminados em `\n`) tanto pela **USB**
(terminal serial a 115200 8N1, via VCP do ST-Link) quanto pelo **Bluetooth** (módulo
HC-05 na USART3, mesmo baud rate — basta parear o HC-05 no celular, senha padrão
`1234` ou `0000`):

| Comando | Efeito |
|---|---|
| `MANUAL` | Entra em modo manual |
| `AUTO` | Entra em modo autônomo (seguir linha) |
| `MOTOR <esq> <dir>` | Define a velocidade das rodas (floats de -1.0 a 1.0, negativo = ré). Só tem efeito em modo MANUAL |
| `STOP` | Para os dois motores imediatamente |
| `CALIBRATE` | Inicia a calibração dos sensores |
| `SET_PID <kp> <ki> <kd>` | Ajusta os ganhos do PID de seguimento de linha |
| `GET_PID` | Responde com `PID:<kp> <ki> <kd>` |

A cada 1 segundo o robô transmite uma linha de telemetria (nas duas portas):
```
X=0.12 Y=-0.03 Th=0.45 V=0.20 Vavg=0.18 Dist=1.34 Bat=87% BatRaw=1234 Mode=0 Calib=1
```

### 5. Segurança
- O sensor ultrassônico apita o buzzer quando um obstáculo se aproxima (zona de alerta
  a partir de 20 cm).
- O botão **RIGHT** limpa uma condição de emergência ativa, quando houver.

---

## App Android de controle

O robô também pode ser controlado por um **app Android** feito em Flutter, que se
conecta ao HC-05 por Bluetooth Clássico (SPP) e fala exatamente o protocolo ASCII
descrito acima: modo MANUAL/AUTÔNOMO, joystick para controle manual, botão de
calibração e ajuste dos ganhos de PID, além de exibir a telemetria recebida em tempo
real (posição, velocidade, bateria etc).

- **Repositório do app:** https://github.com/JoseHenrique0302/robo-control-app
- **Entrega:** o projeto do app também é enviado em um arquivo `.zip` junto com este
  projeto. Os arquivos `README.md` e `TUTORIAL_PARA_RODAR.md` daquele repositório
  explicam como clonar, gerar o scaffolding Android (`flutter create .`) e rodar o app
  em um celular físico (o Bluetooth Clássico não funciona em emulador).
- Para conectar: pareie o HC-05 nas configurações de Bluetooth do celular
  (senha `1234` ou `0000`), abra o app, conecte no dispositivo pareado e use a tela de
  controle — os mesmos comandos (`MANUAL`, `AUTO`, `MOTOR`, `STOP`, `CALIBRATE`,
  `SET_PID`, `GET_PID`) e a mesma telemetria descritos acima.

---

## Compilação

O firmware é desenvolvido no **STM32CubeIDE** (projeto gerado a partir de um `.ioc`
CubeMX, com o código de aplicação nos blocos `USER CODE` de
[Core/Src/app_freertos.c](Core/Src/app_freertos.c) e demais arquivos em `Core/`).
Abra a pasta do repositório como projeto existente no STM32CubeIDE, compile e grave na
NUCLEO-G474RE via ST-Link.
