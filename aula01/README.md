# Aula 01

Utilizando [código python](./aula1.py) para movimentar um robô na simulação do turtlesim

## Como carregar o simulador

### Roscore

O comando roscore inicia o núcleo do ROS 1, que é essencial para a comunicação entre os nós do sistema. Ele sobe:

- o rosmaster (coordena a troca de mensagens),

- o servidor de parâmetros (guarda configurações),

- e o rosout (sistema de logs).

> É o primeiro comando que deve ser executado antes de rodar outros nós em ROS 1.

No terminal execute

```bash
roscore
```

### Rosrun

O comando rosrun turtlesim turtlesim_node serve para iniciar o nó da tartaruga no simulador Turtlesim, que é um simulador simples usado para aprender os conceitos básicos do ROS.
Quebra do comando:

- rosrun: executa um nó de um pacote específico.

- turtlesim: nome do pacote.

- turtlesim_node: nome do nó que será executado.

> Esse comando abre uma janela gráfica com uma tartaruga que você pode controlar enviando comandos, como mover para frente, girar, etc.

Em outra aba do terminal execute

```bash
rosrun turtlesim turtlesim_node
```

### rqt_graph

O comando **rqt_graph** abre uma interface gráfica que mostra como os nós ROS estão se comunicando entre si, ou seja, a estrutura de tópicos, publicadores e assinantes em tempo real.

O que ele faz:

- Exibe um grafo com:

  - Nós (nodes),

  - Tópicos (topics),

  - Direções de mensagens (de quem publica para quem assina).

Em outra aba do terminal execute

```bash
rqt_graph
```

> Esse comando é útil para verificar toda a comunicação que está acontecendo

### rostopic

| Subcomando      | O que faz                                    | Exemplo                                                                  |
| --------------- | -------------------------------------------- | ------------------------------------------------------------------------ |
| `rostopic list` | Lista todos os tópicos ativos                | `rostopic list`                                                          |
| `rostopic echo` | Mostra as mensagens publicadas em um tópico  | `rostopic echo /turtle1/pose`                                            |
| `rostopic info` | Mostra info sobre um tópico (tipo, nós etc.) | `rostopic info /turtle1/pose`                                            |
| `rostopic type` | Mostra o tipo de mensagem de um tópico       | `rostopic type /turtle1/pose`                                            |
| `rostopic pub`  | Publica mensagens manualmente em um tópico   | `rostopic pub /turtle1/cmd_vel geometry_msgs/Twist ...` |

> Para o último comando dê um tab para que o autocomplete preencha os dados faltantes, você encontrará parâmetros x, y e z para velocidade linear & angular. Altere esses dados e o robô utilizará dessas velocidades por um breve periódo de tempo.

Em outra aba do terminal execute conforme a tabela acima

```bash
rostopic ...
```

### Resumo da função do código

O código implementa um controlador que move a tartaruga do turtlesim por várias coordenadas, corrigindo seu ângulo e posição usando feedback da pose atual.

- Recebe a posição atual da tartaruga com **rospy.Subscriber**.

- Calcula a direção e velocidade necessárias para chegar a um ponto com o controlador.

- Publica comandos de movimento com **rospy.Publisher**.

- Repete isso para várias coordenadas, fazendo a tartaruga "desenhar" um caminho no ambiente.
