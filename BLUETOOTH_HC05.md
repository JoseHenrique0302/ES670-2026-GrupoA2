# Bridge Bluetooth (HC-05) — contexto pronto para implementar

> Objetivo: fazer o firmware falar com o **app Android** (Bluetooth Clássico SPP)
> em vez de (ou além de) falar pela **USB**. O protocolo já está pronto e é o mesmo;
> só falta trocar a **porta** de `LPUART1` (USB/VCP) para `USART3` (HC-05).
>
> Estado atual (branch `branchJose`): comandos e telemetria usam `hlpuart1`
> (= LPUART1 = VCP do ST-Link, via USB). `USART3` **não** está configurada.

---

## 0. Hardware (já montado, conferir)

```
HC-05 TX  → STM32 PB11 (USART3_RX)
HC-05 RX  → STM32 PB10 (USART3_TX)   (HC-05 é 3,3V nos pinos de dados; OK direto)
HC-05 VCC → 5V     |     HC-05 GND → GND
```
> Conferir: o HC-05 piscando = não pareado/sem conexão; aceso fixo = conectado.

---

## 1. Configuração na IOC (STM32CubeMX)

1. **Connectivity → USART3 → Mode = Asynchronous.**
2. Parameter Settings:
   - Baud Rate: **115200**
   - Word Length: **8 bits**, Parity: **None**, Stop Bits: **1** (8N1)
   - (igual ao LPUART1 atual — ver `usart.c`)
3. Pinos (devem cair em **PB10 = USART3_TX** e **PB11 = USART3_RX**; ajustar no chip view se necessário).
4. **NVIC Settings → habilitar "USART3 global interrupt"** (a recepção é por interrupção, igual ao LPUART1 hoje).
5. **NÃO precisa de DMA** (o padrão atual usa `HAL_UART_Receive_IT`, 1 byte por vez).
6. `Project → Generate Code`.

> Depois de gerar: `usart.c` terá `MX_USART3_UART_Init()` e `usart.h` terá
> `extern UART_HandleTypeDef huart3;`. O `stm32g4xx_it.c` terá `USART3_IRQHandler`
> chamando `HAL_UART_IRQHandler(&huart3)`. Nada disso some ao regerar.

---

## 2. Código — troca protegida por `#define` (USB ⇄ HC-05)

A ideia: um único interruptor decide a porta, e o USB continua funcionando até
ativarmos o Bluetooth. **Tudo dentro de blocos `USER CODE`, sobrevive à regeração.**

### 2.1. Definir o interruptor e o alias da UART
No `app_freertos.c`, no bloco `/* USER CODE BEGIN PD */` (junto dos outros `#define`):
```c
// 0 = comandos/telemetria pela USB (LPUART1).  1 = pelo HC-05 (USART3).
#define BT_USE_USART3   0

#if (BT_USE_USART3 != 0)
  extern UART_HandleTypeDef huart3;
  #define BT_UART  huart3
#else
  #define BT_UART  hlpuart1
#endif
```

### 2.2. Trocar as 4 referências a `hlpuart1` por `BT_UART`
São exatamente estes pontos (linhas aproximadas na branch atual):

| Onde | Hoje | Fica |
|---|---|---|
| Arma recepção (USER CODE RTOS_SEMAPHORES) | `HAL_UART_Receive_IT(&hlpuart1, &gucBtRxByte, 1);` | `HAL_UART_Receive_IT(&BT_UART, &gucBtRxByte, 1);` |
| TX do `GET_PID` (vTaskUART) | `HAL_UART_Transmit(&hlpuart1, ...)` | `HAL_UART_Transmit(&BT_UART, ...)` |
| TX da telemetria (vTaskUART) | `HAL_UART_Transmit(&hlpuart1, ...)` | `HAL_UART_Transmit(&BT_UART, ...)` |

### 2.3. Ajustar o callback de recepção (instância correta)
No `HAL_UART_RxCpltCallback` (bloco USER CODE Application), trocar a checagem fixa
`LPUART1` por uma que segue o `#define`:
```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == BT_UART.Instance)   // <- antes era == LPUART1
    {
        gucBtRxRing[gusBtRxHead] = gucBtRxByte;
        gusBtRxHead = (uint16_t)((gusBtRxHead + 1U) % BT_RX_RING_SIZE);
        osSemaphoreRelease(semBTRxReadyHandle);
        HAL_UART_Receive_IT(huart, &gucBtRxByte, 1);
    }
}
```

> Pronto. Com `BT_USE_USART3 = 0` nada muda (continua USB). Trocando para `1` e
> tendo a USART3 na IOC, todo o tráfego passa a ir/vir pelo HC-05 — **sem mexer em
> mais nada** (mesmos comandos, mesma telemetria, mesmo parsing).

---

## 3. Configurar o HC-05 (baud 115200)

O HC-05 de fábrica costuma vir a **9600**. Duas opções:
- **(A)** Deixar a USART3 da IOC em **9600** (mais simples, sem mexer no módulo). Funciona,
  só é mais lento para a telemetria.
- **(B)** Reprogramar o HC-05 para **115200** via comandos AT (modo AT: segurar o botão
  do módulo ao ligar; LED pisca devagar):
  ```
  AT
  AT+UART=115200,0,0
  AT+NAME=RoboES670     (opcional, nome que aparece no celular)
  AT+PSWD="1234"        (senha de pareamento, opcional)
  ```
  Depois, manter a USART3 em 115200 (igual ao LPUART1).

> Recomendado: **(B) 115200**, para casar com o resto do projeto.

---

## 4. Teste (passo a passo)

1. Com `BT_USE_USART3 = 0`: confirmar que **tudo ainda funciona pela USB** (terminal
   serial 115200 → `MANUAL`, `MOTOR 0.3 0.3`, telemetria chegando). Garante que não quebrou nada.
2. Mudar `BT_USE_USART3 = 1`, recompilar e gravar.
3. No celular: **parear** o HC-05 (senha `1234`/`0000`).
4. Abrir o app → conectar no HC-05 → enviar `MANUAL` e mexer no joystick.
5. Verificar a telemetria chegando no painel do app a cada 1 s.

### Se não funcionar
- **Nada chega:** TX/RX trocados (HC-05 TX ↔ STM32 RX). Inverter PB10/PB11.
- **Lixo/caracteres errados:** baud do HC-05 ≠ baud da USART3. Igualar (115200 dos dois lados).
- **App não acha o HC-05:** parear primeiro nas configurações de Bluetooth do Android.
- **USB parou de responder:** esperado — com `BT_USE_USART3 = 1` o tráfego foi para o HC-05.
  Para testar pelos dois ao mesmo tempo seria preciso duplicar os TX (não necessário agora).

---

## 5. Resumo do que falta (checklist)
- [ ] IOC: habilitar USART3 (8N1, 115200, NVIC ON) em PB10/PB11 → Generate Code
- [ ] Código: adicionar o `#define BT_USE_USART3` + alias `BT_UART` (seção 2.1)
- [ ] Código: trocar as 4 refs `hlpuart1` → `BT_UART` (2.2) e o callback (2.3)
- [ ] HC-05: baud 115200 (ou deixar tudo em 9600)
- [ ] Testar USB (flag 0) e depois Bluetooth (flag 1)

> Protocolo e telemetria **já estão prontos e compatíveis com o app** — nada a mudar lá.
> Ver também o app em `../robo-control-app/` (AGENTS.md descreve o mesmo protocolo).
