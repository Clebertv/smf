# Braço robótico Lab-Volt 5250

Braço Lab-Volt 5250 com o controle original substituído por um Arduino Mega 2560 e pontes H. O repositório reúne o firmware, um simulador 3D para treinar os alunos e uma ponte serial para comandar o braço real pelo app do Claude.

## Conteúdo

| Pasta | O que tem |
| --- | --- |
| `firmware/BracoTeclado/` | Programa do Arduino. Controle pelo teclado do PC (PuTTY, 9600 baud) e programas automáticos com comandos `#`. |
| `simulador/` | `bancada-5250.html`: simulador 3D que roda a mesma lógica do firmware, para as aulas. `miguel-5250.html`: versão de demonstração com controle por voz. `treino-5250.html`: só simulador, com 10 exercícios avaliados automaticamente. |
| `ponte-serial/` | Servidor MCP em Python que liga o app do Claude para desktop à porta COM do Arduino. Instruções em `LEIA-ME.txt`. |

## Como usar o simulador

Abra o arquivo `.html` no Chrome ou no Edge. Sem o braço ligado, ele funciona como simulador. Com o Arduino no USB, o botão **Conectar braço real** envia os comandos para a porta COM.

O `treino-5250.html` não conecta ao braço real. Ele traz exercícios com metas, tempo e contagem de colisões: conhecer os eixos, pegar e colocar, ponto sorteado, primeiro programa, linha contínua com esteira, separação por cor, empilhamento, paletização 2 × 2, precisão e produção cronometrada. O progresso fica salvo no navegador do aluno.

O controle por voz do `miguel-5250.html` usa o reconhecimento de voz do Chrome, que precisa de internet. Comece cada comando dizendo "Miguel", por exemplo: "Miguel, abra a garra".

## Teclas

| Teclas | Eixo |
| --- | --- |
| Q / A | Base (recuar / avançar) |
| W / S | Ombro (recuar / avançar) |
| E / D | Cotovelo (recuar / avançar) |
| R / F | Movimento do punho (recuar / avançar) |
| T / G | Giro do punho (recuar / avançar) |
| Y / H | Garra (fechar / abrir) |
| Espaço | Parar tudo |
| Z Z | Zeramento |
| P | Mostrar posições |

## Faixas de posição (firmware 15/10)

Todo eixo começa em 0 na chave de fim de curso.

| Eixo | Zero | Máximo |
| --- | --- | --- |
| Base | 0 | 53000 |
| Ombro | 0 | 18000 |
| Cotovelo | 0 | 20010 |
| Movimento do punho | 0 | 10680 |
| Giro do punho | 0 | 23410 |
| Garra | 0 (aberta) | 26095 (fechada) |

## Situação (14/10)

- Firmware versão 15/10: todos os eixos começam em 0, nomes das teclas padronizados (Q/W/E/R/T recuam, A/S/D/F/G avançam) e garra corrigida (Y fecha, H abre). Ainda falta gravá-lo e testá-lo no braço.
- Ponte H do giro do punho substituída em 15/10: todos os eixos funcionam nos dois sentidos.
- Ângulos e cursos do simulador são aproximados. Medidas reais: ombro → cotovelo 35 cm, cotovelo → punho 23 cm.
