# Navegador Autônomo com Exploração para TurtleBot

Este pacote ROS implementa um sistema de navegação autônoma para um robô TurtleBot num ambiente simulado no Gazebo. O sistema combina uma estratégia de **exploração baseada em fronteiras** com um planeador de caminho **A*** para alcançar um destino pré-definido num mapa desconhecido.

O robô irá primeiro explorar o seu ambiente para construir um mapa. Assim que o destino final for descoberto e considerado acessível, ele irá calcular e seguir o caminho mais curto para lá chegar.

## Funcionalidades Principais

1. **Mapa de Ocupação**: Utiliza o pacote `gmapping` do ROS para construir um mapa 2D do ambiente em tempo real a partir dos dados de um sensor laser.
2. **Exploração Autônoma**: Se o destino final não estiver numa área conhecida do mapa, o robô irá navegar autonomamente para as bordas entre o espaço conhecido e o desconhecido (fronteiras) para expandir o mapa.
3. **Planeamento de Caminho (A*)**: Uma vez que o destino se torna acessível, o algoritmo A* é usado para calcular o caminho mais curto e seguro, contornando os obstáculos.
4. **Controlo Robusto**: Um controlador guia o robô ao longo do caminho planeado e inclui comportamentos de recuperação (como rodar sobre si mesmo) caso fique preso.

---

## Pré-requisitos

Antes de começar, certifique-se de que tem o seguinte software instalado:

* Ubuntu 20.04 (Focal Fossa)
* ROS Noetic
* Gazebo (geralmente instalado com a versão completa do ROS)
* Pacotes do simulador TurtleBot:

    ```bash
    sudo apt-get update
    sudo apt-get install ros-noetic-turtlebot ros-noetic-turtlebot-apps ros-noetic-turtlebot-interactions ros-noetic-turtlebot-simulator ros-noetic-kobuki-ftdi ros-noetic-ar-track-alvar-msgs
    ```

---

## Guia de Instalação e Execução

Siga estes passos para configurar e executar o projeto.

1. Coloque o Pacote no seu Workspace

    Copie ou mova a pasta `navegador_autonomo` para o diretório `src` do seu Catkin workspace (ex: `~/catkin_ws/src`).

    Se não tiver um workspace, pode criar um com os seguintes comandos:

    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make
    ```

2. Compile o Pacote

    Abra um terminal, navegue até à raiz do seu Catkin workspace e execute catkin_make para compilar o pacote.

    ``` bash
    cd ~/catkin_ws/
    catkin_make
    ```

3. Configure o Ambiente do Terminal

    Antes de executar o pacote, você precisa de carregar as variáveis de ambiente do seu workspace. Faça isso executando o seguinte comando.

    ``` bash
    source ~/catkin_ws/devel/setup.bash
    ```

    Dica: Para evitar ter que digitar este comando toda vez que abrir um novo terminal, adicione-o ao seu ficheiro .bashrc:

    ``` bash
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    ```

4. Execute o Sistema Completo

    Com tudo configurado, pode iniciar a simulação completa (Gazebo, GMapping, RViz e o nó de navegação) com um único comando:

    ``` bash
    roslaunch navegador_autonomo navegacao_completa.launch
    ```

    O que Esperar

    Após executar o comando roslaunch, várias janelas irão abrir:

        Gazebo: Mostrando o robô TurtleBot no ambiente sala.world.
        
        RViz: Uma ferramenta de visualização onde você verá:
            O robô.

            O mapa a ser construído em tempo real.

            O caminho planeado (uma linha verde) publicado no tópico /planned_path.

    O robô começará a mover-se sozinho, explorando o ambiente em direção às fronteiras do mapa. Se ele ficar preso, ele irá rodar sobre si mesmo para obter uma nova perspetiva e tentar um novo caminho. Assim que o destino final (x=4.0, y=0.0) for mapeado como uma área livre, ele traçará o caminho final e navegará até lá.
    Como Alterar o Destino

    Para mudar o destino final do robô, basta editar as seguintes linhas no topo do ficheiro navegador_autonomo/scripts/navegador_final.py:

    self.goal_x = 4.0

    self.goal_y = 0.0
